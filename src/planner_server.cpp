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
 * Subscribe to map server topic navlite/map [octomap_msgs::msg::Octomap] to 
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

#include "drone_interfaces/srv/offboard.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "navigation_lite/visibility_control.h"
#include "navigation_lite/d_star_lite.h"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <octomap_msgs/conversions.h>
#include "octomap_msgs/msg/octomap.hpp"

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
    
    subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "nav_lite/map", 10, std::bind(&PlannerServer::topic_callback, this, _1));
     
    this->planning_action_server_ = rclcpp_action::create_server<ComputePathToPose>(
      this,
      "nav_lite/compute_path_to_pose",
      std::bind(&PlannerServer::handle_plan_goal, this, _1, _2),
      std::bind(&PlannerServer::handle_plan_cancel, this, _1),
      std::bind(&PlannerServer::handle_plan_accepted, this, _1));
      
    robot_diameter_ = this->declare_parameter<double>("drone_diameter", 0.80);   // 800 mm for my current craft.
    robot_height_   = this->declare_parameter<double>("drone_height", 0.50);
    
    // This is constrain the cost map.  I cannot fly further than this.  
    // Guess this has to be re-thought a bit.
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
  std::unique_ptr<octomap::OcTree> octomap_;
  std::string map_frame_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  float robot_diameter_;
  float robot_height_;
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
  void topic_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) 
  {
    // Convert ROS message to a OctoMap
    std::unique_ptr<octomap::AbstractOcTree> tree{octomap_msgs::msgToMap(*msg)};
    if (tree) {
      octomap_ = std::unique_ptr<octomap::OcTree>( dynamic_cast<octomap::OcTree *>(tree.release()));
    }
        
    if (octomap_){ // can be NULL
      RCLCPP_DEBUG(this->get_logger(), "Octree Conversion successfull.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Octree Conversion failed.");
    }
  }
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription_;
  
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
      rclcpp::Time now = this->get_clock()->now();

      // An alternative to avoid path planning. Just return the goal. 
      geometry_msgs::msg::PoseStamped pose;

      pose.header.stamp = now;
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
    RCLCPP_DEBUG(this->get_logger(), "Testing [%.2f, %.2f, %.2f]", x, y, z);
    if (octomap_) {
      // Set a boundig box around the passed x, y, z coordinates to see if the drone can fit here    
      // This box is axis aligned, so make sure you build a bit of a safey margin into the parameter
      float robot_half_diameter = robot_diameter_ / 2;
      octomap::point3d bbxMin(x-robot_half_diameter, y-robot_half_diameter, z);
      octomap::point3d bbxMax(x+robot_half_diameter, y+robot_half_diameter, z + robot_height_);
      for(octomap::OcTree::leaf_bbx_iterator it = octomap_->begin_leafs_bbx(bbxMin,bbxMax),
          end=octomap_->end_leafs_bbx(); it!= end; ++it) {

        octomap::OcTreeNode* node = octomap_->search( it.getKey() );
        if(node!=NULL) {   // NULL = Undefined
          if (octomap_->isNodeOccupied(node)) {
            return true;
          }        
        }
      }
    }
    
    return false;        
  }
  
};  // class PlannerServer

}  // namespace navigation_lite

RCLCPP_COMPONENTS_REGISTER_NODE(navigation_lite::PlannerServer)
