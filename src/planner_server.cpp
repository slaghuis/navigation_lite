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
 * Action Server responding to navgation_interfaces/action/ComputePathToPose
 *   called only by the Navigation Server
 * Subscribe to map server [navigation_interfaces/msg/ufo_map_stamped] to 
 *   receive a maintained octree global map.
 * Responds to the Action Client with a path derived from the Octree 
 * Motion planning algorithm based on the D* Lite algorithm. 
 * Listens to tf2 map->base_link to find starting position for path planning.
 * All calculations done in the map frame
 * ***********************************************************************/
#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <thread>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "navigation_interfaces/action/compute_path_to_pose.hpp"
#include "navigation_interfaces/msg/ufo_map_stamped.hpp"

#include "drone_interfaces/srv/offboard.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "navigation_lite/visibility_control.h"
#include "navigation_lite/ufomap_ros_msgs_conversions.h"
#include "navigation_lite/d_star_lite.h"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <ufo/map/occupancy_map.h>

using namespace std::placeholders;

namespace navigation_lite
{
  
class PlannerServer : public rclcpp::Node
{
public:
  using ComputePathToPose = navigation_interfaces::action::ComputePathToPose;
  using GoalHandleComputePathToPose = rclcpp_action::ServerGoalHandle<ComputePathToPose>;

  NAVIGATION_LITE_PUBLIC
  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("planner_server", options)
  {
    using namespace std::chrono_literals;
    
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    bypass_planning_ = this->declare_parameter<bool>("bypass_planning", false);
    
    tf_buffer_ = 
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    subscription_ = this->create_subscription<navigation_interfaces::msg::UfoMapStamped>(
      "nav_lite/map", 10, std::bind(&PlannerServer::topic_callback, this, _1));
     
    this->planning_action_server_ = rclcpp_action::create_server<ComputePathToPose>(
      this,
      "nav_lite/compute_path_to_pose",
      std::bind(&PlannerServer::handle_plan_goal, this, _1, _2),
      std::bind(&PlannerServer::handle_plan_cancel, this, _1),
      std::bind(&PlannerServer::handle_plan_accepted, this, _1));
      
    // Build a UFO map
    double resolution;   
    resolution = this->declare_parameter<double>("map_resolution", 0.25);   // use resolution 0.25.  Can then query the map at 0.5 and 1.0
    map_ = std::make_shared<ufo::map::OccupancyMap>(resolution); 

    drone_diameter_ = this->declare_parameter<double>("drone_diameter", 0.80);   // 800 mm for my current craft.
    e_size_ = this->declare_parameter<int>("e_w_size", 500);
    n_size_ = this->declare_parameter<int>("n_s_size", 500);
    u_size_ = this->declare_parameter<int>("u_size", 10);
    
    dsl = new DStarLite(e_size_, n_size_, u_size_);
    
    // Set the lookup function to check the UFO map for occupancy
    auto fn = bind( &PlannerServer::isOccupied, this, _1, _2, _3 );
    dsl->setTestFunction( fn );
    
    RCLCPP_INFO(this->get_logger(), "Action Server [nav_lite/compute_path_to_pose] started");
  }

private:
  rclcpp_action::Server<ComputePathToPose>::SharedPtr planning_action_server_;
  std::shared_ptr<ufo::map::OccupancyMap> map_;
  std::string map_frame_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  double drone_diameter_;
  DStarLite *dsl;
  int e_size_, n_size_, u_size_;
  bool bypass_planning_;
  
  bool read_position(float *x, float *y, float *z)
  {  
    std::string from_frame = "base_link_ned"; 
    std::string to_frame = map_frame_.c_str();
      
    geometry_msgs::msg::TransformStamped transformStamped;
    
    // Look up for the transformation between map and base_link_ned frames
    // and returns the last position in the 'map' frame
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
    return true;
  }
  
  // MAP SUBSCRIPTION ////////////////////////////////////////////////////////////////////////////////////////////////
  void topic_callback(const navigation_interfaces::msg::UfoMapStamped::SharedPtr msg) const
  {
    // Convert ROS message to a UFOmap
    if (ufomap_msgs::msgToUfo(msg->map, map_)) {
      RCLCPP_DEBUG(this->get_logger(), "UFO Map Conversion successfull.");
    } else {
      RCLCPP_WARN(this->get_logger(), "UFO Map Conversion failed.");
    }
  }
  rclcpp::Subscription<navigation_interfaces::msg::UfoMapStamped>::SharedPtr subscription_;
  
  // PLANNER ACTION SERVER ///////////////////////////////////////////////////////////////////////////////////////////

  rclcpp_action::GoalResponse handle_plan_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePathToPose::Goal> goal)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received goal request for path to [%.2f;%.2f;%.2f]", goal->goal.pose.position.x, goal->goal.pose.position.y, goal->goal.pose.position.z);
    (void)uuid;
    
    if ( ((goal->goal.pose.position.x < (-e_size_ / 2)) || (goal->goal.pose.position.x > (e_size_ / 2))) ||
         ((goal->goal.pose.position.y < (-n_size_ / 2)) || (goal->goal.pose.position.y > (n_size_ / 2))) ||
         ((goal->goal.pose.position.z < 0) || (goal->goal.pose.position.z > u_size_)) ) {
      RCLCPP_ERROR(this->get_logger(), "Goal of [%.2f;%.2f;%.2f] is outside map range. [%i;%i;%i]", 
        goal->goal.pose.position.x, goal->goal.pose.position.y, goal->goal.pose.position.z,
        e_size_, n_size_, u_size_);
      return rclcpp_action::GoalResponse::REJECT;
    }     
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_plan_cancel(
    const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_plan_accepted(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&PlannerServer::execute_plan, this, _1), goal_handle}.detach();
  }

  void execute_plan(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    RCLCPP_DEBUG(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ComputePathToPose::Result>();

    auto start_time = this->now();
    if (bypass_planning_) {
      RCLCPP_WARN(this->get_logger(), "Bypassing path planning.");

      // An alternative to avoid path planning. Just return the goal. 
      geometry_msgs::msg::PoseStamped pose;

      // pose.header.stamp = this->now();
      pose.header.frame_id = "map"; 
      pose.pose.position.x = goal->goal.pose.position.x;
      pose.pose.position.y = goal->goal.pose.position.y;
      pose.pose.position.z = goal->goal.pose.position.z;
  
      pose.pose.orientation.x = goal->goal.pose.orientation.x;
      pose.pose.orientation.y = goal->goal.pose.orientation.y; 
      pose.pose.orientation.z = goal->goal.pose.orientation.z; 
      pose.pose.orientation.w = goal->goal.pose.orientation.w;   
    
      result->path.poses.push_back(pose);     
   
    } else {
        
      // # If false, use current robot pose as path start, if true, use start above instead
      if(goal->use_start == true) {
        RCLCPP_DEBUG(this->get_logger(), "Planning a path from %.2f, %.2f, %.2f",
          goal->start.pose.position.x, 
          goal->start.pose.position.y, 
          goal->start.pose.position.z);
        dsl->setStart((int)goal->start.pose.position.x, (int)goal->start.pose.position.y, (int)goal->start.pose.position.z);  
      } else {
        // use the current robot position.
        float x, y, z;
        read_position(&x, &y, &z);  // From tf2      
        dsl->setStart(x, y, z);
        RCLCPP_INFO(this->get_logger(), "Planning a path from %.2f, %.2f, %.2f", x, y, z);
      }
   
      dsl->setGoal((int)goal->goal.pose.position.x, (int)goal->goal.pose.position.y, (int)goal->goal.pose.position.z);    
      dsl->initialize();   
      dsl->computeShortestPath();
    
      dsl->extractPath(result->path.poses); 
      if (result->path.poses.size() > 0) {
        // Overwrite the pose on the goal step
        result->path.poses.back().pose.orientation.x = goal->goal.pose.orientation.x;
        result->path.poses.back().pose.orientation.y = goal->goal.pose.orientation.y;
        result->path.poses.back().pose.orientation.z = goal->goal.pose.orientation.z;
        result->path.poses.back().pose.orientation.w = goal->goal.pose.orientation.w; 
      }; 
          
    }
    
    // Check if goal is done
    if (rclcpp::ok()) {
      result->planning_time = this->now() - start_time;
      goal_handle->succeed(result);
      RCLCPP_DEBUG(this->get_logger(), "Goal succeeded");
    }
  }

  // MAPPING UTILITY FUNCTIONS //////////////////////////////////////////////////////////////////////

  bool isOccupied(const float x, float y, float z)
  {
    // Would it be better to see if the robot can travel between current position (tf_listener)
    // and the target posiiton ina straight line.  Then the granularity of my d*lite algoritm and
    // the UFO map changes a bit.
    
    // The robot's target position
    ufo::math::Vector3 position((double)x, double(y), (double)z);

    // The robot's size (radius)
    double radius = drone_diameter_ / 2;

    // Sphere with center at position and radius radius.
    ufo::geometry::Sphere sphere(position, radius);

    // Check if the robot will be in collision with occupied space 
    // at the finest map resolution.

    // Iterate through all leaf nodes that intersects the bounding volume
    for (auto it = map_->beginLeaves(sphere, true, 
                                 false, false, false, 2),            // Use resolution of 0->0.25m, 1->0.5m 2->1.0m
                                 it_end = map_->endLeaves(); it != it_end; ++it) {
      // Is in collision since a leaf node intersects the bounding volume.
      return true;
    }
    // No leaf node intersects the bounding volume.
    return false;
        
  }
  
};  // class PlannerServer

}  // namespace navigation_lite

RCLCPP_COMPONENTS_REGISTER_NODE(navigation_lite::PlannerServer)
