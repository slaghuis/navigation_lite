// Copyright (c) 2021 Xeni Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* **********************************************************************
 * Subscribe to tf2 map->base_link for orientation and pose data
 * Subscribe to sensor_msgs/msg/Range
 *
 * Action Server responding to navgation_interfaces/action/FollowWaypoints
 *   called only by the Navigation Server
 * Publishes cmd_vel as geometry_msgs/msg/Twist to effect motion.
 * Motion would be:
 *   - Amend yaw, to point to the next waypoint
 *   - Increase foreward velocity to reach desitnation, using a PID 
 *       controller to govern speed
 *   - If an obstacle is encountered, stop and fail (requesting recovery) <--- TO BE IMPLEMENTED
 *
 * Action Server responsing to navigation_interfaces/action/Spin
 *   called only by the Navigation Server
 * Publishes cmd_vel as geometry_msgs/msg/Twist to effect motion.
 * Motion would be to increase yaw velocity using a PID controller until 
 *   target yaw is reached.
 *
 * Action Server responding to navigation_interfaces/action/Wait
 *  called only by the Navigation Server
 * Maintains original position (x,y,z,yaw) through the use of PID 
 *   controllers for a set time.
 *
 * A Mutex lock governs that only one action server can control the drone
 *  at a time.  Who knows what would happen if another node starts sending 
 *  out cmd_vel messages?
 *
 * ***********************************************************************/
#include <functional>
#include <memory>
#include <thread>
#include <mutex>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "navigation_interfaces/action/follow_waypoints.hpp"
#include "navigation_interfaces/action/spin.hpp"
#include "navigation_interfaces/action/wait.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "navigation_lite/visibility_control.h"
#include "navigation_lite/pid.h"

static const float DEFAULT_MAX_SPEED_XY = 0.25;         // Maximum horizontal speed, in m/s
static const float DEFAULT_MAX_SPEED_Z = 0.33;          // Maximum vertical speed, in m/s
static const float DEFAULT_MAX_YAW_SPEED = 0.25;        // Maximum yaw speed in radians/s
static const float DEFAULT_WAYPOINT_RADIUS_ERROR = 0.3; // Acceptable XY distance to waypoint deemed as close enough

namespace navigation_lite
{
class RecoveryServer : public rclcpp::Node
{
public:
  
  using Spin = navigation_interfaces::action::Spin;
  using GoalHandleSpin = rclcpp_action::ServerGoalHandle<Spin>;

  using Wait = navigation_interfaces::action::Wait;
  using GoalHandleWait = rclcpp_action::ServerGoalHandle<Wait>;

  NAVIGATION_LITE_PUBLIC
  explicit RecoveryServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("recovery_server", options)
  {            
    one_off_timer_ = this->create_wall_timer(
      1000ms, std::bind(&RecoveryServer::init, this));
  }
    
private:    
  std::mutex server_mutex;   // Only allow one Action Server to address the drone at a time
  
  // Node Parameters
  float max_yaw_speed_;
  float max_speed_xy_;
  float max_speed_z_;
  
  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp::TimerBase::SharedPtr one_off_timer_;

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Last (transformed) pose of the drone
  std::shared_ptr<geometry_msgs::msg::Pose> last_pose = std::make_shared<geometry_msgs::msg::Pose>();
  double last_yaw;
  
  void init() {
    using namespace std::placeholders;
    
    // Only run this once.  Stop the timer that triggered this.
    this->one_off_timer_->cancel();
       
    // Read node parameters    
    this->declare_parameter<float>("max_speed_xy", DEFAULT_MAX_SPEED_XY);
    this->get_parameter("max_speed_XY", max_speed_xy_);

    this->declare_parameter<float>("max_speed_z", DEFAULT_MAX_SPEED_Z);
    this->get_parameter("max_speed_Z", max_speed_z_);

    this->declare_parameter<float>("max_yaw_speed", DEFAULT_MAX_YAW_SPEED);
    this->get_parameter("max_yaw_speed", max_yaw_speed_);
        
    // Create a transform listener
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Call on_timer function every half a second (Is this enouh to ensure smooth motion?
    timer_ = this->create_wall_timer(
      500ms, std::bind(&RecoveryServer::on_timer, this));
    RCLCPP_DEBUG(this->get_logger(), "Transform Listener [map->base_link] started");  

    // Create drone velocity publisher
    publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", 1);

    // Create the three action servers     
    this->spin_action_server_ = rclcpp_action::create_server<Spin>(
      this,
      "nav_lite/spin",
      std::bind(&RecoveryServer::spin_handle_goal, this, _1, _2),
      std::bind(&RecoveryServer::spin_handle_cancel, this, _1),
      std::bind(&RecoveryServer::spin_handle_accepted, this, _1));
    RCLCPP_INFO(this->get_logger(), "Action Server [nav_lite/spin] started");
  
    this->wait_action_server_ = rclcpp_action::create_server<Wait>(
      this,
      "nav_lite/wait",
      std::bind(&RecoveryServer::wait_handle_goal, this, _1, _2),
      std::bind(&RecoveryServer::wait_handle_cancel, this, _1),
      std::bind(&RecoveryServer::wait_handle_accepted, this, _1));   
    RCLCPP_INFO(this->get_logger(), "Action Server [nav_lite/wait] started");
   }   

// Transformation Listener/////////////////////////////////////////////////////
  void on_timer()
  {
    // Store frame names in variables that will be used to
    // compute transformations

    std::string source_frameid = "map";
    std::string target_frameid = "base_link";

    geometry_msgs::msg::TransformStamped transformStamped;

    // Look up for the transformation between map and base_link frames
    // and save the last position
    try {
      transformStamped = tf_buffer_->lookupTransform(
        target_frameid, source_frameid,
        tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        this->get_logger(), "Could not transform %s to %s: %s",
        target_frameid.c_str(), source_frameid.c_str(), ex.what());
      return;
    }

    last_pose->position.x = transformStamped.transform.translation.x;  // Foreward of origin
    last_pose->position.y = transformStamped.transform.translation.y;  // Left of origin
    last_pose->position.z = transformStamped.transform.translation.z;  // Above origin
    
    last_pose->orientation.x = transformStamped.transform.rotation.x;  // Quaterion
    last_pose->orientation.y = transformStamped.transform.rotation.y;
    last_pose->orientation.z = transformStamped.transform.rotation.z;
    last_pose->orientation.w = transformStamped.transform.rotation.w;
    
    // Calculate yaw
    // Orientation quaternion
    tf2::Quaternion q(
      last_pose->orientation.x,
      last_pose->orientation.y,
      last_pose->orientation.z,
      last_pose->orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch;
    m.getRPY(roll, pitch, last_yaw);

  }
    
  // SPIN /////////////////////////////////////////////////////////////////////////////////////////  
  rclcpp_action::Server<Spin>::SharedPtr spin_action_server_;

  rclcpp_action::GoalResponse spin_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Spin::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to rotate to %.2f radians", goal->target_yaw);
    (void)uuid;
    if(server_mutex.try_lock()) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Another thread is commanding the drone now.  Rejecting Spin request.");
      return rclcpp_action::GoalResponse::REJECT; 
    }
  }

  rclcpp_action::CancelResponse spin_handle_cancel(
    const std::shared_ptr<GoalHandleSpin> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void spin_handle_accepted(const std::shared_ptr<GoalHandleSpin> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RecoveryServer::spin_execute, this, _1), goal_handle}.detach();
  }

  void spin_execute(const std::shared_ptr<GoalHandleSpin> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Spin::Feedback>();
    auto & angular_distance_traveled = feedback->angular_distance_traveled;
    auto result = std::make_shared<Spin::Result>();
    
    geometry_msgs::msg::Twist setpoint = geometry_msgs::msg::Twist();
    std::shared_ptr<PID> pid_yaw = std::make_shared<PID>(0.5, max_yaw_speed_, -max_yaw_speed_, 0.7, 0.00, 0);

        // Set up a message
    setpoint.angular.x = 0.0;
    setpoint.angular.y = 0.0;
    setpoint.angular.z = 0.0;

    setpoint.linear.x = 0.0;
    setpoint.linear.y = 0.0;  
    setpoint.linear.z = 0.0; 
    
    auto start_yaw = last_yaw;
    auto start_time = steady_clock_.now();
    bool keep_on_spinning = true;
    while (rclcpp::ok() && keep_on_spinning) {
          
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->total_elapsed_time = steady_clock_.now() - start_time ;      
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        server_mutex.unlock();
        return;
      }

      // Ask the drone to turn
      setpoint.angular.z = pid_yaw->calculate( goal->target_yaw, last_yaw);         // calculate yaw      
      keep_on_spinning = (abs(setpoint.angular.z) > 0.02);
      publisher_->publish(setpoint);  

      // Publish some feedback
      angular_distance_traveled = abs(last_yaw - start_yaw);
      goal_handle->publish_feedback(feedback);
                    
      // Don't flood the flight controller
      loop_rate.sleep();
    };
    setpoint.angular.z = 0.0;  // Stop Motion
    publisher_->publish(setpoint);  
    
    // Check if goal is done
    if (rclcpp::ok()) {
      result->total_elapsed_time = steady_clock_.now() - start_time ;      
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
    server_mutex.unlock();
  }  
  
  // WAIT /////////////////////////////////////////////////////////////////////////////////////
  
  rclcpp_action::Server<Wait>::SharedPtr wait_action_server_;

  rclcpp_action::GoalResponse wait_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Wait::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to wait for %d seconds and %d nanoeconds", goal->time.sec, goal->time.nanosec);
    (void)uuid;
    if(server_mutex.try_lock()) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Another thread is commanding the drone now.  Rejecting Wait request.");
      return rclcpp_action::GoalResponse::REJECT; 
    }
  }

  rclcpp_action::CancelResponse wait_handle_cancel(
    const std::shared_ptr<GoalHandleWait> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void wait_handle_accepted(const std::shared_ptr<GoalHandleWait> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RecoveryServer::wait_execute, this, _1), goal_handle}.detach();
  }

  void wait_execute(const std::shared_ptr<GoalHandleWait> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Wait::Feedback>();
    auto & time_left = feedback->time_left;
    auto result = std::make_shared<Wait::Result>();

    geometry_msgs::msg::Twist setpoint = geometry_msgs::msg::Twist();
    // Do one include PID controllers to negate wind movig the drone? 
    // std::shared_ptr<PID> pid_yaw = std::make_shared<PID>(0.5, max_yaw_speed_, -max_yaw_speed_, 0.7, 0.00, 0);
    
    // Set up a message
    setpoint.angular.x = 0.0;
    setpoint.angular.y = 0.0;
    setpoint.angular.z = 0.0;

    setpoint.linear.x = 0.0;
    setpoint.linear.y = 0.0;  
    setpoint.linear.z = 0.0; 
    
    auto start_time = now();
    bool keep_on_waiting = true;
    while (rclcpp::ok() && keep_on_waiting) {
          
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->total_elapsed_time = now() - start_time;      
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        server_mutex.unlock();
        return;
      }

      // Ask the drone to do nothing
      //setpoint.angular.z = pid_yaw->calculate( goal->target_yaw, last_yaw);         // calculate yaw      
      publisher_->publish(setpoint);  
      
      keep_on_waiting = ((start_time + goal->time) < now());

      // Publish some feedback
      time_left = ((start_time + goal->time) - now());
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Time left %d sec %d nanosec", time_left.sec, time_left.nanosec);      
      keep_on_waiting = ((time_left.sec > 0) && (time_left.nanosec > 500000));  // remember a loop_rate.sleep() still comes!
      
      // Don't flood the flight controller
      loop_rate.sleep();
    };
    setpoint.angular.z = 0.0;  // Stop Motion
    publisher_->publish(setpoint);  

    // Check if goal is done
    if (rclcpp::ok()) {
      result->total_elapsed_time = now() - start_time;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
    server_mutex.unlock();
  }  
 
  
};  // class RecoveryServer

}  // namespace navigation_lite

RCLCPP_COMPONENTS_REGISTER_NODE(navigation_lite::RecoveryServer)
