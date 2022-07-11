// Copyright (c) 2022 Eric Slaghuis
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
 * Subscribe to tf2 map->base_link for position and pose data
 * Subscribe to nav_lite/map [octomap_msgs::msg::Octomap] 
 *
 * Action Server responding to navgation_interfaces/action/FollowPath
 *   called only by the Navigation Server
 * Publishes cmd_vel as geometry_msgs/msg/Twist to effect motion.
 * The motion strategy would be determined by the specified PLUGIN.
 * See available PLUGINS in the controller_plugins package
 *
 * A Mutex lock governs that only one action server can control the drone
 *  at a time.  Who knows what would happen if another node starts sending 
 *  out cmd_vel messages?
 *
 * ***********************************************************************/

#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <thread>
#include <limits>       // std::numeric_limits
#include <cmath>        // std::hypot
#include <mutex>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "navigation_interfaces/action/follow_path.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <octomap_msgs/conversions.h>
#include "octomap_msgs/msg/octomap.hpp"

#include "navigation_lite/visibility_control.h"
#include "navigation_lite/holddown_timer.hpp"
#include "navigation_lite/exceptions.hpp"

#include <pluginlib/class_loader.hpp>
#include <navigation_lite/controller.hpp>

const double PI  =3.141592653589793238463;

static const float DEFAULT_WAYPOINT_RADIUS_ERROR = 0.3; // Acceptable XY distance to waypoint deemed as close enough
static const float DEFAULT_YAW_THRESHOLD = 0.025;       // Acceptible YAW to start foreward acceleration
static const int DEFAULT_HOLDDOWN = 1;                  // Time to ensure stability in flight is attained

// In C the modulo operation returns a value with the same sign as the dividend.
// Hence a custom modulo function
inline double modulo(const double a, const double n) {
  return a - floor(a/n) * n;
}

// Returns the difference between two angles x and y as a number 
// between -180 and 180.  c can be PI for radians, or 180 for degrees. 
inline double getDiff2Angles(const double x, const double y, const double c)
{
  double a = x-y;
  return modulo( a+c, 2*c) - c;
}

using namespace std::chrono_literals;

namespace navigation_lite
{
class ControllerActionServer : public rclcpp::Node
{
public:
  using FollowPath = navigation_interfaces::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

  NAVIGATION_LITE_PUBLIC
  explicit ControllerActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("controller_server", options),
    loader_("navigation_lite", "navigation_lite::Controller")
  {            
    one_off_timer_ = this->create_wall_timer(
      1000ms, std::bind(&ControllerActionServer::init, this));
  }
    
private:    
  
  std::mutex server_mutex;   // Only allow one Action Server to address the drone at a time
  pluginlib::ClassLoader<navigation_lite::Controller> loader_;
  std::shared_ptr<navigation_lite::Controller> controller_;
  
  geometry_msgs::msg::Twist last_velocity_;
  
  geometry_msgs::msg::PoseStamped end_pose_;

  float waypoint_radius_error_;
  float yaw_threshold_;
  int holddown_;
  double controller_frequency_;
  
  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::TimerBase::SharedPtr one_off_timer_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string map_frame_;
    
  std::shared_ptr<HolddownTimer> holddown_timer_;

  std::shared_ptr<octomap::OcTree> octomap_;
    
  void init() {
    using namespace std::placeholders;
    
    // Only run this once.  Stop the timer that triggered this.
    this->one_off_timer_->cancel();
    
    // Create a transform listener
    tf_buffer_ =
      std::make_shared<tf2_ros::Buffer>(this->get_clock());      
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Declare and get parameters    
    controller_frequency_ = this->declare_parameter("control_frequency", 10.0);     // Control frequency in Hz.  Must be bigger than 2 Hz
    waypoint_radius_error_ = this->declare_parameter<float>("waypoint_radius_error", DEFAULT_WAYPOINT_RADIUS_ERROR);
    yaw_threshold_ = this->declare_parameter<float>("yaw_threshold", DEFAULT_YAW_THRESHOLD);
    holddown_ = this->declare_parameter<int>("holddown", DEFAULT_HOLDDOWN);
    holddown_timer_ = std::make_shared<HolddownTimer>(holddown_);    
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    
    // Create drone velocity publisher
    publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", 1);

    // ROS2 Subscriptions
    map_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "nav_lite/map", 10, std::bind(&ControllerActionServer::map_callback, this, _1));
      
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "drone/odom", 10, std::bind(&ControllerActionServer::odom_callback, this, _1));
  
    // Create the action server
    this->action_server_ = rclcpp_action::create_server<FollowPath>(
      this,
      "nav_lite/follow_path",
      std::bind(&ControllerActionServer::handle_goal, this, _1, _2),
      std::bind(&ControllerActionServer::handle_cancel, this, _1),
      std::bind(&ControllerActionServer::handle_accepted, this, _1));
      
    RCLCPP_INFO(this->get_logger(), "Controller Action Server [nav_lite/follow_path] started");    
  }   
  
// MAP SUBSCRIPTION ////////////////////////////////////////////////////////////////////////////////////////////////
  void map_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) 
  {
    // Convert ROS message to a OctoMap
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    if (tree) {
      octomap_ = std::shared_ptr<octomap::OcTree>( dynamic_cast<octomap::OcTree *>(tree));
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error creating octree from received message");
    } 
  }
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_subscription_;
  
// ODOM SUBSCRIPTION ////////////////////////////////////////////////////////////////////////////////////////////////
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
  {
    last_velocity_ = msg->twist.twist;
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

// FLIGHT CONTROL ////////////////////////////////////////////////////////////////////////////////////////////////

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
  
// FollowWayPoint Action Server /////////////////////////////////////////////////////////////////
  rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowPath::Goal> goal)
  {
    (void)uuid;
    if(goal->path.poses.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Invalid path, Path is empty.  Rejecting request.");
      return rclcpp_action::GoalResponse::REJECT; 
    }
    RCLCPP_INFO(this->get_logger(), "Controller Server received request to follow %d waypoints", goal->path.poses.size());
    
    if(server_mutex.try_lock()) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Another thread is commanding the drone now.  Rejecting request.");
      return rclcpp_action::GoalResponse::REJECT; 
    }  
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ControllerActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FollowPath::Feedback>();
    auto & distance_to_goal = feedback->distance_to_goal;
    auto & speed = feedback->speed;
    auto result = std::make_shared<FollowPath::Result>();
    
    auto start_time = this->now();
    end_pose_ = goal->path.poses.back();
    if (octomap_ == NULL) {
      RCLCPP_INFO(this->get_logger(), "Waiting for the first map");
    }
    
    // Wait for the fist map to arrive
    rclcpp::Rate r(100);
    while ( (octomap_ == NULL) && !goal_handle->is_canceling() ) {
      r.sleep();
    }
    
    if (goal_handle->is_canceling()) {
      // result->planning_time = this->now() - start_time;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled before the first map has been received!");
      return;
    }
    
    try
    {
      std::string controller_base_name = "controller_plugins::"; 
      controller_ = loader_.createSharedInstance(controller_base_name.append( goal->controller_id ));  //"controller_plugins::PurePursuitController"
      auto node_ptr = shared_from_this(); 
      controller_->configure(node_ptr, goal->controller_id, tf_buffer_, octomap_);
      controller_->setPath( goal->path );
            
      rclcpp::Rate loop_rate( controller_frequency_ );
      while ( rclcpp::ok() ) {
        // Check if there is a cancelling request
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          stop_movement();
          server_mutex.unlock();
          return;
        }
        
        // Compute and publish velocity
        geometry_msgs::msg::PoseStamped pose;    
        if (!read_position(pose)) {
          throw ControllerException("Failed to obtain robot pose.");
        }
        
        // Check if the robot is stuck here (for longer than a timeout)
        
        try {
          geometry_msgs::msg::TwistStamped setpoint;
          setpoint = controller_->computeVelocityCommands( pose, last_velocity_);      
          RCLCPP_INFO(this->get_logger(), "Publishing velocity [%.2f, %.2f, %.2f, %.4f]", 
            setpoint.twist.linear.x, 
            setpoint.twist.linear.y, 
            setpoint.twist.linear.z,
            setpoint.twist.angular.z);
          publisher_->publish( setpoint.twist ); 

          // Publish feedback
          speed = std::hypot( double(last_velocity_.linear.x), double(last_velocity_.linear.y), double(last_velocity_.linear.z) );
          size_t current_idx = find_closest_goal_idx( pose, goal->path); 
          distance_to_goal = calculate_path_length( goal->path, current_idx );
          goal_handle->publish_feedback(feedback);
        } catch (ControllerException & e) {
          RCLCPP_WARN(this->get_logger(), e.what());
        }
        
        if (is_goal_reached()) {
          break;
        }  
                                
        if (!loop_rate.sleep()) {
          RCLCPP_WARN(
            get_logger(), "Control loop missed its desired rate of %.4fHz",
            controller_frequency_);
        }
      }
      
      stop_movement();      
      if (rclcpp::ok() ) { 
        goal_handle->succeed(result);
      }
      stop_movement();

    
    }
    catch(pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "The controller plugin failed to load for some reason: %s", ex.what());
      goal_handle->abort(result);      
      server_mutex.unlock();
      return;
    }
    catch(ControllerException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "The controller threw an exception: %s", ex.what());
      goal_handle->abort(result);
      server_mutex.unlock();
      return;
    }
        
    server_mutex.unlock();
  }

  bool read_position(geometry_msgs::msg::PoseStamped & pose)
  {
    std::string target_frame = "base_link";
    std::string reference_frame = "map";
    
    geometry_msgs::msg::TransformStamped transformStamped;
    
    // Look up for the transformation between map and base_link frames
    // and save the last position in the 'map' frame
    try {
      transformStamped = tf_buffer_->lookupTransform(
        reference_frame, target_frame,
        tf2::TimePointZero);
        pose.header.frame_id = reference_frame;
        pose.header.stamp = transformStamped.header.stamp;
        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = transformStamped.transform.translation.z;
        
        pose.pose.orientation.x = transformStamped.transform.rotation.x;
        pose.pose.orientation.y = transformStamped.transform.rotation.y;
        pose.pose.orientation.z = transformStamped.transform.rotation.z;
        pose.pose.orientation.w = transformStamped.transform.rotation.w;
        
    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        this->get_logger(), "Could not transform %s to %s: %s",
        reference_frame.c_str(), target_frame.c_str(), ex.what());
      return false;  
    }
    
    return true;

  }
    
  bool read_position(double *x, double *y, double *z, double *w)
  {
    std::string target_frame = "base_link";
    std::string reference_frame = "map";
    
    geometry_msgs::msg::TransformStamped transformStamped;
    
    // Look up for the transformation between map and base_link frames
    // and save the last position in the 'map' frame
    try {
      transformStamped = tf_buffer_->lookupTransform(
        reference_frame, target_frame,
        tf2::TimePointZero);
        *x = transformStamped.transform.translation.x;
        *y = transformStamped.transform.translation.y;
        *z = transformStamped.transform.translation.z;
    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        this->get_logger(), "Could not transform %s to %s: %s",
        reference_frame.c_str(), target_frame.c_str(), ex.what());
      return false;  
    }
    *w = get_yaw(transformStamped.transform.rotation);
    
    return true;
  }
  
  double get_yaw(geometry_msgs::msg::Quaternion quaternion)
  {
    // Orientation quaternion
    tf2::Quaternion q(
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw; 
    m.getRPY(roll, pitch, yaw);
    
    return yaw;
  }

  double euclidean_distance(geometry_msgs::msg::PoseStamped start, geometry_msgs::msg::PoseStamped end)
  {
    // for some reason std::hypot(double, double, double) cannot be used, wven with c++17  
    return std::sqrt(
      pow(end.pose.position.x - start.pose.position.x,2) +
      pow(end.pose.position.y - start.pose.position.y,2) +
      pow(end.pose.position.z - start.pose.position.z,2) );
      
  }
  
  bool is_goal_reached() {
    geometry_msgs::msg::PoseStamped pose;
    
    read_position(pose);    
    double position_error = euclidean_distance(pose, end_pose_);      
    double yaw_error = getDiff2Angles( get_yaw(pose.pose.orientation), get_yaw(end_pose_.pose.orientation), PI);  
      
    return holddown_timer_->test( (position_error < waypoint_radius_error_ ) &&
                                  ( yaw_error < yaw_threshold_)  );        
  }
  
  size_t find_closest_goal_idx(geometry_msgs::msg::PoseStamped pose, nav_msgs::msg::Path path)
  {
    size_t closest_pose_idx = 0;
    double curr_min_dist = std::numeric_limits<double>::max();
    
    for (size_t curr_idx = 0; curr_idx < path.poses.size(); ++curr_idx) {
      double curr_dist = euclidean_distance( pose, path.poses[curr_idx]);
      if (curr_dist < curr_min_dist) {
        curr_min_dist = curr_dist;
        closest_pose_idx = curr_idx;
      }
    }
    return closest_pose_idx;
  }  
  
  double calculate_path_length( const nav_msgs::msg::Path path, const size_t current_idx ) 
  {
    double distance = 0.0;
    for(size_t i = current_idx; i < path.poses.size()-1; i++) {
      distance += euclidean_distance( path.poses[i], path.poses[i+1] );
    }
    
    return distance;
  }
  
};  // class ControllerActionServer

}  // namespace navigation_lite

RCLCPP_COMPONENTS_REGISTER_NODE(navigation_lite::ControllerActionServer)
