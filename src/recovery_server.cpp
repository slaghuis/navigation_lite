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
 * Subscribe to tf2 odom->base_link for orientation and pose data
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
#include "navigation_lite/pid.hpp"
#include "navigation_lite/holddown_timer.hpp"

static const float DEFAULT_MAX_SPEED_XY = 2.0;          // Maximum horizontal speed, in m/s
static const float DEFAULT_MAX_SPEED_Z = 0.33;          // Maximum vertical speed, in m/s
static const float DEFAULT_MAX_YAW_SPEED = 0.5;         // Maximum yaw speed in radians/s 
static const float DEFAULT_WAYPOINT_RADIUS_ERROR = 0.3; // Acceptable XY distance to waypoint deemed as close enough
static const float DEFAULT_YAW_THRESHOLD = 0.025;       // Acceptible YAW to start foreward acceleration
static const float DEFAULT_ALTITUDE_THRESHOLD = 0.3;    // Acceptible Z distance to altitude deemed as close enough 
static const int DEFAULT_HOLDDOWN = 2;                  // Time to ensure stability in flight is attained

inline double getDiff2Angles(const double x, const double y, const double c)
{
  // c can be PI (for radians) or 180.0 (for degrees);
  double d =  fabs(fmod(fabs(x - y), 2*c));
  double r = d > c ? c*2 - d : d;
  
  double sign = (x-y >= 0.0 && x-y <= c) || (x-y <= -c && x-y> -2*c) ? 1.0 : -1.0;
  return sign * r;                                           

}

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
  float waypoint_radius_error_;
  float yaw_threshold_;
  float altitude_threshold_;
  int holddown_;
  
  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp::TimerBase::SharedPtr one_off_timer_;

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // PID Controllers  
  std::shared_ptr<PID> pid_x;
  std::shared_ptr<PID> pid_y;
  std::shared_ptr<PID> pid_z;
  std::shared_ptr<PID> pid_yaw;
  
  // Holddown Timer
  std::shared_ptr<HolddownTimer> holddown_timer;
  
  // Global Variables
  bool tune_x_, tune_y_, tune_z_, tune_yaw_, calculate_yaw_;
  bool waypoint_is_close_, altitude_is_close_, pose_is_close_;
  float target_x_, target_y_, target_z_, target_yaw_;
  
  // Last (transformed) pose of the drone
  std::shared_ptr<geometry_msgs::msg::Pose> last_pose = std::make_shared<geometry_msgs::msg::Pose>();
  
  void init() {
    using namespace std::placeholders;
    
    // Only run this once.  Stop the timer that triggered this.
    this->one_off_timer_->cancel();
       
    // Declare node parameters    
    this->declare_parameter<float>("max_speed_xy", DEFAULT_MAX_SPEED_XY);
    this->declare_parameter<float>("max_speed_z", DEFAULT_MAX_SPEED_Z);
    this->declare_parameter<float>("max_yaw_speed", DEFAULT_MAX_YAW_SPEED);
    this->declare_parameter("pid_xy", std::vector<double>{0.7, 0.0, 0.0});
    this->declare_parameter("pid_z", std::vector<double>{0.7, 0.0, 0.0});
    this->declare_parameter("pid_yaw", std::vector<double>{0.7, 0.0, 0.0});    
    this->declare_parameter<float>("waypoint_radius_error", DEFAULT_WAYPOINT_RADIUS_ERROR);
    this->declare_parameter<float>("yaw_threshold", DEFAULT_YAW_THRESHOLD);
    this->declare_parameter<float>("altitude_threshold", DEFAULT_ALTITUDE_THRESHOLD);
    this->declare_parameter<int>("holddown", DEFAULT_HOLDDOWN);

    // Read the parameters
    this->get_parameter("max_yaw_speed", max_yaw_speed_);
    this->get_parameter("max_speed_xy", max_speed_xy_);
    this->get_parameter("max_speed_z", max_speed_z_);
    this->get_parameter("waypoint_radius_error", waypoint_radius_error_);
    this->get_parameter("yaw_threshold", yaw_threshold_);
    this->get_parameter("altitude_threshold", altitude_threshold_);
    this->get_parameter("holddown", holddown_); 
    
    rclcpp::Parameter pid_xy_settings_param = this->get_parameter("pid_xy");
    std::vector<double> pid_xy_settings = pid_xy_settings_param.as_double_array(); 
    pid_x   = std::make_shared<PID>(0.5, max_speed_xy_, -max_speed_xy_, (float)pid_xy_settings[0], (float)pid_xy_settings[1], (float)pid_xy_settings[2]);
    pid_y   = std::make_shared<PID>(0.5, max_speed_xy_, -max_speed_xy_, (float)pid_xy_settings[0], (float)pid_xy_settings[1], (float)pid_xy_settings[2]);

    rclcpp::Parameter pid_z_settings_param = this->get_parameter("pid_z");
    std::vector<double> pid_z_settings = pid_z_settings_param.as_double_array(); 
    pid_z   = std::make_shared<PID>(0.5, max_speed_z_, -max_speed_z_, (float)pid_z_settings[0], (float)pid_z_settings[1], (float)pid_z_settings[2]);

    rclcpp::Parameter pid_yaw_settings_param = this->get_parameter("pid_yaw");
    std::vector<double> pid_yaw_settings = pid_yaw_settings_param.as_double_array(); 
    pid_yaw   = std::make_shared<PID>(0.5, max_yaw_speed_, -max_yaw_speed_, (float)pid_yaw_settings[0], (float)pid_yaw_settings[1], (float)pid_yaw_settings[2]);

    // Create a transform listener
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Set up the hoddown timer
    holddown_timer = std::make_shared<HolddownTimer>(holddown_);
    
    // Call on_timer function every half a second (Is this enough to ensure smooth motion?
    timer_ = this->create_wall_timer(
      500ms, std::bind(&RecoveryServer::on_timer, this));
    RCLCPP_DEBUG(this->get_logger(), "Transform Listener [map->base_link] started");  

    // Create drone velocity publisher
    publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", 1);

    // Create the two action servers     
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

    std::string source_frameid = "base_link";         // Odometry is published in odom frame
    std::string target_frameid = "odom";    // The drone is base_link frame.  

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
    
    // Save the last pose for feedback
    last_pose->position.x = transformStamped.transform.translation.x;  // Foreward of origin
    last_pose->position.y = transformStamped.transform.translation.y;  // Left of origin
    last_pose->position.z = transformStamped.transform.translation.z;  // Above origin
    
    last_pose->orientation.x = transformStamped.transform.rotation.x;  // Quaterion
    last_pose->orientation.y = transformStamped.transform.rotation.y;
    last_pose->orientation.z = transformStamped.transform.rotation.z;
    last_pose->orientation.w = transformStamped.transform.rotation.w;

    // Calculate deviation from required position and pose
    // See REP-103.  
    // The body standard is
    // x foreward, y left, z up.
    //  The Cartesian representation is east north up, 
    // x east, y north, z up. 
    // and the compass turns CLOCKWISE.
    // Since we work in the base link frame Front LEFT Up and the compass turns CLOCKWISE,
    // We have to swing y around.
    float err_x = target_x_- transformStamped.transform.translation.x; 
    float err_y = transformStamped.transform.translation.y - target_y_;
    float err_dist = sqrt(pow(err_x,2) + pow(err_y,2));        
    waypoint_is_close_ = (err_dist < waypoint_radius_error_);

    float err_z = transformStamped.transform.translation.z - target_z_;
    altitude_is_close_ = ( abs(err_z) < altitude_threshold_);
       
    geometry_msgs::msg::Twist setpoint = geometry_msgs::msg::Twist();
    setpoint.angular.x = 0.0;
    setpoint.angular.y = 0.0;
    
    if(tune_yaw_) {
      // Calculate yaw
      
      double yaw_to_target = (err_x == 0.0) ? atan(err_y / 0.00001) : atan(err_y / err_x);  //Avoid division by zero
      
      if (err_x < 0.0) {
        if(err_y > 0.0) {
          yaw_to_target += M_PI;
        } else {
          yaw_to_target -= M_PI;
        }
      }      
      target_yaw_ = (calculate_yaw_)?yaw_to_target:target_yaw_;
    
      RCLCPP_DEBUG(this->get_logger(), "Drone at %.2f,%.2f,%.2f going to %.2f,%.2f Target Yaw %.2f", 
          transformStamped.transform.translation.x,
          transformStamped.transform.translation.y,
          transformStamped.transform.translation.z,
          target_x_,
          target_y_,
          target_yaw_);
          
      // Orientation quaternion
      tf2::Quaternion q(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w);

      // 3x3 Rotation matrix from quaternion
      tf2::Matrix3x3 m(q);

      // Roll Pitch and Yaw from rotation matrix
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      
      double yaw_error = getDiff2Angles(target_yaw_, yaw, M_PI);
      pose_is_close_ = yaw_error < yaw_threshold_;
      
      setpoint.angular.z = pid_yaw->calculate(0, yaw_error);         // correct yaw error down to zero
      RCLCPP_DEBUG(this->get_logger(), "Yaw at %.2f, going to %.2f Speeed:%.2f", 
          yaw,
          target_yaw_,
          setpoint.angular.z);
    } else {
      setpoint.angular.z = 0.0;
    }
    
    if(tune_x_ ) {
      setpoint.linear.x = pid_x->calculate(0, -err_x);  // fly
    } else {
      setpoint.linear.x = 0.0;
    }

    if(tune_y_ ) {
      setpoint.linear.y = pid_y->calculate(0, -err_y);  // fly
    } else {
      setpoint.linear.y = 0.0;
    }
    
    if(tune_z_) {
      setpoint.linear.z = pid_z->calculate(0, err_z);  // correct altitude
    } else {
      setpoint.linear.z = 0.0;
    }
    
    // Ask the drone to turn
    if( tune_x_ || tune_y_ || tune_z_ || tune_yaw_)
      publisher_->publish(setpoint);

  }
    
  /* ************************************************************
   * Set the global parameters to invoke velocity settings by the 
   * transfom timer callback above
   * ************************************************************/
  bool stop_movement() {
    rclcpp::Rate loop_rate(2);
    RCLCPP_INFO(this->get_logger(), "stopping movement");
    tune_x_ = false;
    tune_z_ = false;
    tune_yaw_ = false;
    
    geometry_msgs::msg::Twist setpoint = geometry_msgs::msg::Twist();
    
    setpoint.angular.x = 0.0;
    setpoint.angular.y = 0.0;
    setpoint.angular.z = 0.0;

    setpoint.linear.x = 0.0;
    setpoint.linear.y = 0.0;  
    setpoint.linear.z = 0.0;  
    publisher_->publish(setpoint);        
    loop_rate.sleep();    
    publisher_->publish(setpoint); // Just to be sure :-)       
    
    return true;
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
    
    // Set the global variable
    target_yaw_ = goal->target_yaw;
    target_z_ = last_pose->position.z;  // Maintain altitude
    
    // Calculate current yaw
    // Orientation quaternion
    tf2::Quaternion q(
      last_pose->orientation.x,
      last_pose->orientation.y,
      last_pose->orientation.z,
      last_pose->orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Set Scope of work
    tune_x_ = false;
    tune_z_ = true;
    tune_yaw_ = true;
    calculate_yaw_ = false;
    
    // Reset PID Controllers
    pid_x->restart_control();
    pid_yaw->restart_control();
    pid_z->restart_control();

    // Reset Global Indicators
    waypoint_is_close_ = false;
    pose_is_close_ = false;
    altitude_is_close_ = false;
        
    auto start_yaw = yaw;
    auto start_time = steady_clock_.now();    
    
    while ( rclcpp::ok() && !holddown_timer->test(pose_is_close_)) {
    
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->total_elapsed_time = steady_clock_.now() - start_time ;      
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        server_mutex.unlock();
        return;
      }
      
      // Publish some feedback
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
      m.getRPY(roll, pitch, yaw);
      
      angular_distance_traveled =  getDiff2Angles(start_yaw, yaw, M_PI);
      goal_handle->publish_feedback(feedback);
      
      // Dont flood the flight controller
      // The transform listener will tick every loop and update the status of the global variables.
      loop_rate.sleep();
    }
    
    stop_movement();
    
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
    
    // Calculate current yaw
    // Orientation quaternion
    tf2::Quaternion q(
      last_pose->orientation.x,
      last_pose->orientation.y,
      last_pose->orientation.z,
      last_pose->orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Set the global variable
    target_x_ = last_pose->position.x;  // Maintain position
    target_y_ = last_pose->position.y; 
    target_z_ = last_pose->position.z;  // Maintain altitude
    target_yaw_ = yaw;                  // Maintain yaw   

    // Set Scope of work
    tune_x_ = true;
    tune_y_ = true;
    tune_z_ = true;
    tune_yaw_ = true;
    calculate_yaw_ = false;
    
    // Reset PID Controllers
    pid_x->restart_control();
    pid_y->restart_control();
    pid_z->restart_control();
    pid_yaw->restart_control();
    
    // Reset Global Indicators
    waypoint_is_close_ = false;
    pose_is_close_ = false;
    altitude_is_close_ = false;
        
    auto start_time = steady_clock_.now();    
    bool keep_on_waiting = true;
    while (rclcpp::ok() && keep_on_waiting ) {
          
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->total_elapsed_time = now() - start_time;      
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        server_mutex.unlock();
        return;
      }
      
      // Publish some feedback
      time_left = ((start_time + goal->time) - now());
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Time left %d sec %d nanosec", time_left.sec, time_left.nanosec);      
      keep_on_waiting = ((time_left.sec > 0) && (time_left.nanosec > 500000));  // remember a loop_rate.sleep() still comes!
      
      // Give the other processes some time, because time is what this action does best.
      loop_rate.sleep();
      
    };

    stop_movement();
    
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
