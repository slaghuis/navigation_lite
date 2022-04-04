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
static const float DEFAULT_MAX_ACCEL_XY = 0.2; 
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
  float max_accel_xy_;
  float max_speed_z_;
  float waypoint_radius_error_;
  float yaw_threshold_;
  float altitude_threshold_;
  int holddown_;
  double freq_;
  double yaw_control_limit_;
  
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
    
  // Last (transformed) pose of the drone
  std::shared_ptr<geometry_msgs::msg::Pose> last_pose = std::make_shared<geometry_msgs::msg::Pose>();
  
  void init() {
    using namespace std::placeholders;
    
    // Only run this once.  Stop the timer that triggered this.
    this->one_off_timer_->cancel();
       
    // Declare and read some node parameters    
    freq_ = this->declare_parameter("frequency", 10.0);     // Control frequency in Hz.  Must be bigger than 2 Hz
    
    max_yaw_speed_ = this->declare_parameter<float>("max_speed_xy", DEFAULT_MAX_SPEED_XY);
    max_accel_xy_ = this->declare_parameter<float>("max_accel_xy", DEFAULT_MAX_ACCEL_XY);
    max_speed_z_   = this->declare_parameter<float>("max_speed_z", DEFAULT_MAX_SPEED_Z);
    max_yaw_speed_ = this->declare_parameter<float>("max_yaw_speed", DEFAULT_MAX_YAW_SPEED);
    
    waypoint_radius_error_ = this->declare_parameter<float>("waypoint_radius_error", DEFAULT_WAYPOINT_RADIUS_ERROR);
    yaw_threshold_ = this->declare_parameter<float>("yaw_threshold", DEFAULT_YAW_THRESHOLD);
    altitude_threshold_ = this->declare_parameter<float>("altitude_threshold", DEFAULT_ALTITUDE_THRESHOLD);
    holddown_ = this->declare_parameter<int>("holddown", DEFAULT_HOLDDOWN);
    
    // The grid size of the map is still hard coded, thus this is treated as a constant
    yaw_control_limit_ = 1.0;   // Distance from waypoint where control moves to X and Y PID rather than Yaw and X 
    
    // Read the other parameters
    this->declare_parameter("pid_xy", std::vector<double>{0.7, 0.0, 0.0});
    rclcpp::Parameter pid_xy_settings_param = this->get_parameter("pid_xy");
    std::vector<double> pid_xy_settings = pid_xy_settings_param.as_double_array(); 
    pid_x   = std::make_shared<PID>(1.0 / freq_ , max_speed_xy_, -max_speed_xy_, (float)pid_xy_settings[0], (float)pid_xy_settings[1], (float)pid_xy_settings[2]);
    pid_y   = std::make_shared<PID>(1.0 / freq_ , max_speed_xy_, -max_speed_xy_, (float)pid_xy_settings[0], (float)pid_xy_settings[1], (float)pid_xy_settings[2]);

    this->declare_parameter("pid_z", std::vector<double>{0.7, 0.0, 0.0});
    rclcpp::Parameter pid_z_settings_param = this->get_parameter("pid_z");
    std::vector<double> pid_z_settings = pid_z_settings_param.as_double_array(); 
    pid_z   = std::make_shared<PID>(1.0 / freq_, max_speed_z_, -max_speed_z_, (float)pid_z_settings[0], (float)pid_z_settings[1], (float)pid_z_settings[2]);

    this->declare_parameter("pid_yaw", std::vector<double>{0.7, 0.0, 0.0});  
    rclcpp::Parameter pid_yaw_settings_param = this->get_parameter("pid_yaw");
    std::vector<double> pid_yaw_settings = pid_yaw_settings_param.as_double_array(); 
    pid_yaw   = std::make_shared<PID>(1.0 / freq_, max_yaw_speed_, -max_yaw_speed_, (float)pid_yaw_settings[0], (float)pid_yaw_settings[1], (float)pid_yaw_settings[2]);
    
    // Create a transform listener
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Set up the hoddown timer
    holddown_timer = std::make_shared<HolddownTimer>(holddown_);
    
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
    
    server_mutex.unlock();

   }   
    
  // FLIGHT CONTROL ////////////////////////////////////////////////////////////////////////////////////////////////
  bool fly_to_waypoint(geometry_msgs::msg::PoseStamped wp) {
    rclcpp::Rate loop_rate( freq_ );

    bool waypoint_is_close_, altitude_is_close_, pose_is_close_;
    float target_x_, target_y_, target_z_;

    target_x_ = wp.pose.position.x;
    target_y_ = wp.pose.position.y;
    target_z_ = wp.pose.position.z;
   
    geometry_msgs::msg::Twist setpoint = geometry_msgs::msg::Twist();
    setpoint.linear.x = 0.0;
    setpoint.linear.y = 0.0;
    setpoint.linear.z = 0.0;
    setpoint.angular.x = 0.0;
    setpoint.angular.y = 0.0;
    setpoint.angular.z = 0.0;

    double x, y, z, w;
    float err_x, err_y, err_z, err_dist;
    double yaw_to_target;
    double yaw_error;
   

    // First correct the yaw        
    pid_yaw->restart_control();

    do {
      read_position(&x, &y, &z, &w);  // Current position according to tf2

      err_x = target_x_ - x; 
      err_y = target_y_ - y;
      
      if ( sqrt(pow(err_x,2) + pow(err_y,2)) < yaw_control_limit_ ) {
        break;
      }
      yaw_to_target = atan2(err_y, err_x);
      
      yaw_error = getDiff2Angles(yaw_to_target, w, M_PI);
      pose_is_close_ = (fabs(yaw_error) < yaw_threshold_);
      if (pose_is_close_) {
        break;
      }
      setpoint.angular.z = pid_yaw->calculate(0, -yaw_error);         // correct yaw error down to zero  
      
      publisher_->publish(setpoint);
      loop_rate.sleep();  // Give the drone time to move
    } while (!pose_is_close_);  

    // Now that we are ponting, keep on adjusting yaw, but include altitude and foreward velocity
    pid_x->restart_control();
    pid_y->restart_control();
    pid_z->restart_control();
    double last_v_x = 0.0;
    double last_v_y = 0.0;
    do {
      read_position(&x, &y, &z, &w);  // Current position according to tf2

      err_x = target_x_ - x; 
      err_y = target_y_ - y;
      err_z = target_z_ - z;

      err_dist = sqrt(pow(err_x,2) + pow(err_y,2));        
      waypoint_is_close_ = (err_dist < waypoint_radius_error_);

      altitude_is_close_ = ( abs(err_z) < altitude_threshold_);
      setpoint.linear.z = pid_z->calculate(0, -err_z);                // correct altitude
      
      if ( sqrt(pow(err_x,2) + pow(err_y,2)) < yaw_control_limit_ ) {
        // Control via X and Y PID rather than yaw and thrust.
        
        setpoint.linear.x = pid_x->calculate(0, -err_x);              // fly
        setpoint.linear.y = pid_y->calculate(0, -err_y);
      } else {
        // Control with yaw and thrust. 
        yaw_to_target = atan2(err_y, err_x);      
        yaw_error = getDiff2Angles(yaw_to_target, w, M_PI);
        pose_is_close_ = (fabs(yaw_error) < yaw_threshold_);
        
        setpoint.angular.z = pid_yaw->calculate(0, -yaw_error);         // correct yaw error down to zero  
        if( pose_is_close_ ) {
          setpoint.linear.x = pid_x->calculate(0, -err_dist);              // fly
        } else {
          // Avoid flying in a doughnut.  First correct yaw.
          setpoint.linear.x = 0.0;
        }
        setpoint.linear.y = 0.0;
      }
      
      // Govern acceleration, and decellaration.  The latter should be governed by a well 
      // tuned PID but then not all control is done via a PID.  Often velocity is forced
      // to 0 which is an abrupt stop.
      if (setpoint.linear.x > last_v_x ) {  // Acceleration
        setpoint.linear.x = min(setpoint.linear.x, last_v_x + (max_accel_xy_ / freq_));
      } else {                              // Decelleration
        setpoint.linear.x = max(setpoint.linear.x, last_v_x - (max_accel_xy_ / freq_));
      }
      if (setpoint.linear.y > last_v_y ) {
        setpoint.linear.y = min(setpoint.linear.y, last_v_y + (max_accel_xy_ / freq_));
      } else {
        setpoint.linear.y = max(setpoint.linear.y, last_v_y - (max_accel_xy_ / freq_));
      }
      last_v_x = setpoint.linear.x;
      last_v_y = setpoint.linear.y;
      
      publisher_->publish(setpoint);
      loop_rate.sleep();  // Give the drone time to move
    }  while (!(waypoint_is_close_ && altitude_is_close_)); 
    
    return true;
  }
  
  
  bool correct_yaw(geometry_msgs::msg::PoseStamped wp) {
  
    rclcpp::Rate loop_rate( freq_ );
    
    // Orientation quaternion
    tf2::Quaternion q(
        wp.pose.orientation.x,
        wp.pose.orientation.y,
        wp.pose.orientation.z,
        wp.pose.orientation.w);
        
    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw; 
    m.getRPY(roll, pitch, yaw);
         
    bool  pose_is_close_;
    
    geometry_msgs::msg::Twist setpoint = geometry_msgs::msg::Twist();
    setpoint.linear.x = 0.0;
    setpoint.linear.y = 0.0;
    setpoint.linear.z = 0.0;
    setpoint.angular.x = 0.0;
    setpoint.angular.y = 0.0;
    setpoint.angular.z = 0.0;

    double x, y, z, w;
    double yaw_error;
      
    pid_yaw->restart_control();

    do {
      read_position(&x, &y, &z, &w);  
      
      yaw_error = getDiff2Angles(yaw, w, M_PI);
      pose_is_close_ = (fabs(yaw_error) < yaw_threshold_);
      if (pose_is_close_) {
        break;
      }
      setpoint.angular.z = pid_yaw->calculate(0, -yaw_error);         // correct yaw error down to zero  
      
      publisher_->publish(setpoint);
      loop_rate.sleep();  // Give the drone time to move
    } while (!pose_is_close_); 
    
    return true;
  }
  
  bool stop_movement() {
    rclcpp::Rate loop_rate(2);
    
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
    RCLCPP_DEBUG(this->get_logger(), "Received request to rotate to %.2f radians", goal->target_yaw);
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
    RCLCPP_DEBUG(this->get_logger(), "Received request to cancel goal");
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
    RCLCPP_DEBUG(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Spin::Feedback>();
    auto & angular_distance_traveled = feedback->angular_distance_traveled;
    auto result = std::make_shared<Spin::Result>();
    rclcpp::Rate loop_rate( freq_ );    
        
    geometry_msgs::msg::Twist setpoint = geometry_msgs::msg::Twist();
    setpoint.linear.x = 0.0;
    setpoint.linear.y = 0.0;
    setpoint.linear.z = 0.0;
    setpoint.angular.x = 0.0;
    setpoint.angular.y = 0.0;
    setpoint.angular.z = 0.0;

    double x, y, z, w, start_w;
    double yaw_error;
    read_position(&x, &y, &z, &start_w);
    
    pid_yaw->restart_control();
    auto start_time = steady_clock_.now();
    
    bool  pose_is_close_;
    do {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        stop_movement();
        result->total_elapsed_time = steady_clock_.now() - start_time;      
        goal_handle->canceled(result);
        RCLCPP_DEBUG(this->get_logger(), "Goal canceled");
        server_mutex.unlock();
        return;
      }

      read_position(&x, &y, &z, &w);  
      
      yaw_error = getDiff2Angles(goal->target_yaw, w, M_PI);
      pose_is_close_ = (fabs(yaw_error) < yaw_threshold_);
            
      if (pose_is_close_) {
        break;
      }
      setpoint.angular.z = pid_yaw->calculate(0, -yaw_error);         // correct yaw error down to zero  
      
      publisher_->publish(setpoint);
      // Publish some feedback
      angular_distance_traveled =  getDiff2Angles(start_w, w, M_PI);
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();  // Give the drone time to move
    } while (!pose_is_close_); 

    stop_movement();
    server_mutex.unlock();
    
    // Mark goal as done
    if (rclcpp::ok()) {
      result->total_elapsed_time = steady_clock_.now() - start_time ;      
      goal_handle->succeed(result);
      RCLCPP_DEBUG(this->get_logger(), "Spin Goal succeeded");
    }
    
  }  
  
  // WAIT /////////////////////////////////////////////////////////////////////////////////////
  
  rclcpp_action::Server<Wait>::SharedPtr wait_action_server_;

  rclcpp_action::GoalResponse wait_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Wait::Goal> goal)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received request to wait for %d seconds and %d nanoeconds", goal->time.sec, goal->time.nanosec);
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
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Wait::Feedback>();
    auto & time_left = feedback->time_left;
    auto result = std::make_shared<Wait::Result>();
    
    // Read the current position using tf2
    double cx, cy, cz, cw;
    read_position(&cx, &cy, &cz, &cw);
    
    geometry_msgs::msg::PoseStamped wp;
    
    wp.pose.position.x = cx;
    wp.pose.position.y = cy;
    wp.pose.position.z = cz;
    
    tf2::Quaternion q;
    q.setRPY(0,0,cw);
    wp.pose.orientation.x = q.x();
    wp.pose.orientation.y = q.y();
    wp.pose.orientation.z = q.z();
    wp.pose.orientation.w = q.w();
            
    auto start_time = steady_clock_.now();    
    bool keep_on_waiting = true;
    while (rclcpp::ok() && keep_on_waiting ) {
          
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->total_elapsed_time = steady_clock_.now() - start_time;      
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        server_mutex.unlock();
        return;
      }
      
      // Publish some feedback
      time_left = ((start_time + goal->time) - steady_clock_.now());
      goal_handle->publish_feedback(feedback);
      RCLCPP_DEBUG(this->get_logger(), "Time left %d sec %d nanosec", time_left.sec, time_left.nanosec);      
      keep_on_waiting = ((time_left.sec > 0) && (time_left.nanosec > 500000));  // remember a loop_rate.sleep() still comes!
      
      fly_to_waypoint( wp );
    };

    stop_movement();
    server_mutex.unlock();
    
    // Check if goal is done
    if (rclcpp::ok()) {
      result->total_elapsed_time = steady_clock_.now() - start_time;
      goal_handle->succeed(result);
    }
    
  }  
  
  bool read_position(double *x, double *y, double *z, double *w)
  {
    std::string from_frame = "base_link_ned"; //map_frame_.c_str();
    std::string to_frame = "map";
    
    geometry_msgs::msg::TransformStamped transformStamped;
    
    // Look up for the transformation between map and base_link frames
    // and save the last position in the 'map' frame
    try {
      transformStamped = tf_buffer_->lookupTransform(
        to_frame, from_frame,
        tf2::TimePointZero);
        *x = transformStamped.transform.translation.x;
        *y = transformStamped.transform.translation.y;
        *z = transformStamped.transform.translation.z;
    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        this->get_logger(), "Could not transform %s to %s: %s",
        to_frame.c_str(), from_frame.c_str(), ex.what());
      return false;  
    }
    
    // Orientation quaternion
    tf2::Quaternion q(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch; 
    m.getRPY(roll, pitch, *w);
   

    return true;
  }
 
  
};  // class RecoveryServer

}  // namespace navigation_lite

RCLCPP_COMPONENTS_REGISTER_NODE(navigation_lite::RecoveryServer)
