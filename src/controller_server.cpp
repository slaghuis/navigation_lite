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
 * Subscribe to tf2 odom->base_link for position and pose data
 * Subscribe to sensor_msgs/msg/Range
 *
 * Action Server responding to navgation_interfaces/action/FollowWaypoints
 *   called only by the Navigation Server
 * Publishes cmd_vel as geometry_msgs/msg/Twist to effect motion.
 * Motion would be:
 *   - Amend yaw, to point to the next waypoint
 *   - Increase foreward velocity to reach desitnation, using a PID 
 *       controller to govern speed
 *   - If an obstacle is encountered, stop and fail (requesting recovery)
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

inline double getAbsoluteDiff2Angles(const double x, const double y, const double c)
{
  // c can be PI (for radians) or 180.0 (for degrees);
  return c - fabs(fmod(fabs(x - y), 2*c) - c);
}

namespace navigation_lite
{
class ControllerServer : public rclcpp::Node
{
public:
  using FollowWaypoints = navigation_interfaces::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ServerGoalHandle<FollowWaypoints>;

  NAVIGATION_LITE_PUBLIC
  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("controller_server", options)
  {            
    one_off_timer_ = this->create_wall_timer(
      1000ms, std::bind(&ControllerServer::init, this));
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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  // PID Controllers  
  std::shared_ptr<PID> pid_x;
  std::shared_ptr<PID> pid_z;
  std::shared_ptr<PID> pid_yaw;
  
  // Holddown Timer
  std::shared_ptr<HolddownTimer> holddown_timer;
  
  // Global Variables
  bool tune_x_, tune_z_, tune_yaw_, calculate_yaw_;
  bool waypoint_is_close_, altitude_is_close_, pose_is_close_;
  float target_x_, target_y_, target_z_, target_yaw_;
  
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
      500ms, std::bind(&ControllerServer::on_timer, this));
    RCLCPP_DEBUG(this->get_logger(), "Transform Listener [map->base_link] started");  

    // Create drone velocity publisher
    publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", 1);

    // Create the action server
    this->action_server_ = rclcpp_action::create_server<FollowWaypoints>(
      this,
      "nav_lite/follow_waypoints",
      std::bind(&ControllerServer::handle_goal, this, _1, _2),
      std::bind(&ControllerServer::handle_cancel, this, _1),
      std::bind(&ControllerServer::handle_accepted, this, _1));
    RCLCPP_INFO(this->get_logger(), "Action Server [nav_lite/follow_waypoints] started");
    
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
    setpoint.linear.y = 0.0;
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
      
      double yaw_error = getAbsoluteDiff2Angles(target_yaw_, yaw, M_PI);
      pose_is_close_ = yaw_error < yaw_threshold_;
      
      setpoint.angular.z = pid_yaw->calculate(0, yaw_error);         // correct yaw error down to zero
      RCLCPP_DEBUG(this->get_logger(), "Yaw at %.2f, going to %.2f Speeed:%.2f", 
          yaw,
          target_yaw_,
          setpoint.angular.z);
    } else {
      setpoint.angular.z = 0.0;
    }
    
    if(tune_x_ && pose_is_close_) {                    // alternatively check against setpoint.angular.z
      setpoint.linear.x = pid_x->calculate(0, -err_dist);  // fly
      RCLCPP_DEBUG(this->get_logger(), "Drone at %.2f,%.2f going to %.2f,%.2f Delta:%.2f, Speed:%.2f", 
          transformStamped.transform.translation.x,
          transformStamped.transform.translation.y,
          target_x_,
          target_y_,
          err_dist,
          setpoint.linear.x);
    } else {
      setpoint.linear.x = 0.0;
    }

    if(tune_z_) {
      setpoint.linear.z = pid_z->calculate(0, err_z);  // correct altitude
    } else {
      setpoint.linear.z = 0.0;
    }
    
    // Ask the drone to turn
    if( tune_x_ || tune_z_ || tune_yaw_)
      publisher_->publish(setpoint);
    
  }
  
  /* ************************************************************
   * Set the global parameters to invoke velocity settings by the 
   * transfom timer callback above
   * ************************************************************/
  bool fly_to_waypoint(geometry_msgs::msg::PoseStamped wp) {
    rclcpp::Rate loop_rate(3);
    
    target_x_ = wp.pose.position.x;
    target_y_ = wp.pose.position.y;
    target_z_ = wp.pose.position.z;
    
    // Set Scope of work
    tune_x_ = false;
    tune_z_ = true;
    tune_yaw_ = true;
    calculate_yaw_ = true;
    
    // Reset Global Indicators
    waypoint_is_close_ = false;
    pose_is_close_ = false;
    altitude_is_close_ = false;
    
    // Reset PID Controllers
    pid_x->restart_control();
    pid_yaw->restart_control();
    pid_z->restart_control();
    
    RCLCPP_INFO(this->get_logger(), "fly_to_waypoint:Correcting yaw START");
    while ( !holddown_timer->test(pose_is_close_) ) {
      // Dont flood the flight controller
        loop_rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "fly_to_waypoint:Correcting yaw END");
    
    // Set Scope of work
    tune_x_ = true;
    tune_z_ = false;
    tune_yaw_ = true;
    calculate_yaw_ = true;       
    RCLCPP_INFO(this->get_logger(), "fly_to_waypoint:move START");
//    while ( !holddown_timer->test(waypoint_is_close_ && altitude_is_close_ && pose_is_close_ )) {
    while ( !holddown_timer->test(waypoint_is_close_ && altitude_is_close_ )) {
      // Dont flood the flight controller
        loop_rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "fly_to_waypoint:move END");
    return true;
  }
  
  bool correct_yaw(geometry_msgs::msg::PoseStamped wp) {
    rclcpp::Rate loop_rate(3);
    RCLCPP_INFO(this->get_logger(), "correct_yaw:START");    
    target_x_ = wp.pose.position.x;
    target_y_ = wp.pose.position.y;
    target_z_ = wp.pose.position.z;
    
    
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
        
    while (!holddown_timer->test(pose_is_close_)) {
      // Dont flood the flight controller
      // The transform listener will tick every loop and update the status of the global variables.
        loop_rate.sleep();
    }
    
    tune_x_ = false;    
    while (!holddown_timer->test(waypoint_is_close_ && altitude_is_close_ && pose_is_close_ )) {
      // Dont flood the flight controller
        loop_rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "correct_yaw:END");
    return true;
  }
  
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
  
// FollowWayPoint Action Server /////////////////////////////////////////////////////////////////
  rclcpp_action::Server<FollowWaypoints>::SharedPtr action_server_;
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowWaypoints::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to follow %d waypoints", goal->poses.size());
    (void)uuid;
    if(server_mutex.try_lock()) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Another thread is commanding the drone now.  Rejecting request.");
      return rclcpp_action::GoalResponse::REJECT; 
    }  
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ControllerServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(2);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FollowWaypoints::Feedback>();
    auto & current_waypoint = feedback->current_waypoint;
    auto result = std::make_shared<FollowWaypoints::Result>();
    
            
    RCLCPP_DEBUG(this->get_logger(), "Received %d waypoints.", goal->poses.size());        
    current_waypoint = 0;
    for (geometry_msgs::msg::PoseStamped wp : goal->poses ) {
       
      if (!rclcpp::ok()) {
        // Something is amis. Record some feedback and break out of the loop.
        for( long unsigned int i = current_waypoint; i < goal->poses.size(); i++) {        
          result->missed_waypoints.push_back(i);
        }
        break;
      }
      
            // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        for( long unsigned int i = current_waypoint; i < goal->poses.size(); i++) {        
          result->missed_waypoints.push_back(i);
        }
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        stop_movement();
        server_mutex.unlock();
        return;
      }

      fly_to_waypoint(wp);
            
      // Publish feedback
      goal_handle->publish_feedback(feedback);  // Current waypoint
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %d", current_waypoint);
      
      loop_rate.sleep();
      current_waypoint++;
    }
    
    if (rclcpp::ok() ) correct_yaw(goal->poses.back());      
    if (rclcpp::ok() ) stop_movement();

    // Check if goal is done
    if (rclcpp::ok()) {
      // no feedback to record.  The feedback is an empty vector
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
    
    server_mutex.unlock();
    RCLCPP_INFO(this->get_logger(), "MISSION COMPLETE");
  }
   
  
};  // class ControllerServer

}  // namespace navigation_lite

RCLCPP_COMPONENTS_REGISTER_NODE(navigation_lite::ControllerServer)
