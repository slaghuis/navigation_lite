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

/* *************************************************************
 * Deals with the interface to the navigation stack, and calls
 * the statet plugin.  REQUIRES PLANNER PLUGINS to be installed
 * *************************************************************/

#include <functional>
#include <memory>
#include <chrono>
#include <thread>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "navigation_interfaces/action/compute_path_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>

#include "navigation_lite/visibility_control.h"

#include <pluginlib/class_loader.hpp>
#include <navigation_lite/regular_planner.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

namespace navigation_lite
{
class PlannerActionServer : public rclcpp::Node
{
public:
  using ComputePathToPose = navigation_interfaces::action::ComputePathToPose;
  using GoalHandleComputePathToPose = rclcpp_action::ServerGoalHandle<ComputePathToPose>;

  NAVIGATION_LITE_PUBLIC
  explicit PlannerActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("planner_server", options), map_frame_("map")
  {
    using namespace std::placeholders;
    
    tf_buffer_ =
      std::make_shared<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);  
   
   // Subscribe to the map from the map server
   octomap_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "nav_lite/map", 10, std::bind(&PlannerActionServer::octomap_topic_callback, this, _1));

    this->action_server_ = rclcpp_action::create_server<ComputePathToPose>(
      this,
      "nav_lite/compute_path_to_pose",
      std::bind(&PlannerActionServer::handle_goal, this, _1, _2),
      std::bind(&PlannerActionServer::handle_cancel, this, _1),
      std::bind(&PlannerActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::shared_ptr<octomap::OcTree> octomap_;
  std::string map_frame_;
  
  //  UTILITY FUNCTIONS /////////////////////////////////////////////////////////////////////////////////////////////////
  
  bool read_position(geometry_msgs::msg::PoseStamped* position)
  {  
    std::string from_frame = "base_link_ned"; 
    std::string to_frame = map_frame_;
      
    geometry_msgs::msg::TransformStamped transformStamped;
    
    // Look up for the transformation between map and base_link_ned frames
    // and returns the last position in the 'map' frame
    try {
      transformStamped = tf_buffer_->lookupTransform(
        to_frame, from_frame, tf2::TimePointZero);
        
      position->header.stamp = transformStamped.header.stamp;
      position->header.frame_id = transformStamped.child_frame_id;
        
      position->pose.position.x = transformStamped.transform.translation.x;
      position->pose.position.y = transformStamped.transform.translation.y;
      position->pose.position.z = transformStamped.transform.translation.z;
        
      position->pose.orientation.x = transformStamped.transform.rotation.x;
      position->pose.orientation.y = transformStamped.transform.rotation.y;
      position->pose.orientation.z = transformStamped.transform.rotation.z;
      position->pose.orientation.w = transformStamped.transform.rotation.w;

    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        this->get_logger(), "Could not transform %s to %s: %s",
        to_frame.c_str(), from_frame.c_str(), ex.what());
      return false;  
    }
    return true;
  }

  // MAP SUBSCRIPTION ////////////////////////////////////////////////////////////////////////////////////////////////
  void octomap_topic_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) 
  {
    // Convert ROS message to a OctoMap
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    if (tree) {
      octomap_ = std::shared_ptr<octomap::OcTree>( dynamic_cast<octomap::OcTree *>(tree));
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error creating octree from received message");
    } 
  }
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription_;
  
  
   // PLANNER ACTION SERVER //////////////////////////////////////////////////////////////////////////////////////////

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePathToPose::Goal> goal)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received goal request for path to [%.2f;%.2f;%.2f]", 
      goal->goal.pose.position.x, 
      goal->goal.pose.position.y, 
      goal->goal.pose.position.z);
      
    (void)uuid;
    // Consider rejecting goals that does not present in the "map" frame
    // Alternatively, include a transform to the map frame.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&PlannerActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    
    RCLCPP_DEBUG(this->get_logger(), "Calculating path to pose");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ComputePathToPose::Result>();

    auto start_time = this->now();
    
    if (octomap_ == NULL) {
      RCLCPP_INFO(this->get_logger(), "Waiting for the first map");
    }
    
    // Wait for the fist map to arrive
    rclcpp::Rate r(100);
    while ( (octomap_ == NULL) && !goal_handle->is_canceling() ) {
      r.sleep();
    }
    
    if (goal_handle->is_canceling()) {
      result->planning_time = this->now() - start_time;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled before the first map has been received!");
      return;
    }
    pluginlib::ClassLoader<navigation_lite::RegularPlanner> planner_loader("navigation_lite", "navigation_lite::RegularPlanner");
    
    try
    {
      std::string planner_base_name = "planner_plugins::";     
      std::shared_ptr<navigation_lite::RegularPlanner> path_planner = planner_loader.createSharedInstance(planner_base_name.append( goal->planner_id ) );

      auto node_ptr = shared_from_this(); 
      path_planner->configure( node_ptr, "Planner", tf_buffer_, octomap_);

      // If false, use current robot pose as path start, if true, use start above instead
      if(goal->use_start == true) {
        RCLCPP_DEBUG(this->get_logger(), "Planning a path from %.2f, %.2f, %.2f",
          goal->start.pose.position.x, 
          goal->start.pose.position.y, 
          goal->start.pose.position.z);
        result->path = path_planner->createPlan( goal->start, goal->goal);
      } else {
        // use the current robot position.
        geometry_msgs::msg::PoseStamped current_pos;
        if( !read_position(&current_pos) ) {   // From tf2
          RCLCPP_ERROR(this->get_logger(), "Failed to read current position.  Aborting the action.");
          goal_handle->abort(result);                // This crashes the server.  Why?  What alternatice is there?
        }
        RCLCPP_DEBUG(this->get_logger(), "Planning a path from %.2f, %.2f, %.2f",
          current_pos.pose.position.x, 
          current_pos.pose.position.y, 
          current_pos.pose.position.z);
          
        result->path = path_planner->createPlan( current_pos, goal->goal);
      }
      if( !validate_path( goal->goal, result->path, goal->planner_id)) {
        // No valid goal was calculated
        goal_handle->abort(result);
      }
    }
    
    catch(pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s", ex.what());
    }
        
    // Check if goal is done
    if (rclcpp::ok()) {
      result->planning_time = this->now() - start_time;
      goal_handle->succeed(result);
      RCLCPP_DEBUG(this->get_logger(), "Successfully calculated a path to the target pose");
    }
  } 
  
  bool validate_path(
    const geometry_msgs::msg::PoseStamped & goal,
    const nav_msgs::msg::Path & path,
    const std::string & planner_id)
  {
    if (path.poses.size() == 0) {
      RCLCPP_WARN( this->get_logger(), "Planning algoritm %s failed to generate a valid"
        " path to (%.2f, %.2f, %.2f)", planner_id.c_str(),
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
      // Navigation 2 calls a Termnate here.  What function can I use?
      
        return false;
    }
    
    RCLCPP_DEBUG(
      this->get_logger(),
      "Found a valid path of size %lu to (%.2f, %.2f, %.2f)",
      path.poses.size(),
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
      
    return true;  
    
  }
};  // class PlannerActionServer

}  // namespace navigation_lite

RCLCPP_COMPONENTS_REGISTER_NODE(navigation_lite::PlannerActionServer)

/*


int main(int argc, char** argv)
{
  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<navigation_lite::RegularPlanner> planner_loader("navigation_lite", "navigation_lite::RegularPlanner");

  try
  {
    std::shared_ptr<navigation_lite::RegularPlanner> triangle = planner_loader.createSharedInstance("planner_plugins::NoPlan");
    triangle->initialize(10.0);

    std::shared_ptr<navigation_lite::RegularPlanner> square = planner_loader.createSharedInstance("planner_plugins::ThetaStar");
    square->initialize(10.0);

    printf("Triangle area: %.2f\n", triangle->area());
    printf("Square area: %.2f\n", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}

*/
