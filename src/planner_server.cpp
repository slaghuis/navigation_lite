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
 * Subscribe to sensor_msgs/msg/Range (all of them) to maintain an octree
 *   global map.  (Or is this a seperate server. Still unsure)
 * Responds to the Action Client with a path derived from the Octree
 * ***********************************************************************/

#include <functional>
#include <memory>
#include <thread>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "navigation_interfaces/action/compute_path_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "navigation_lite/visibility_control.h"

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
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<ComputePathToPose>(
      this,
      "nav_lite/compute_path_to_pose",
      std::bind(&PlannerServer::handle_goal, this, _1, _2),
      std::bind(&PlannerServer::handle_cancel, this, _1),
      std::bind(&PlannerServer::handle_accepted, this, _1));
    RCLCPP_INFO(this->get_logger(), "Action Serever [nav_lite/compute_path_to_pose] started");
  }

private:
  rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePathToPose::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request for path to [%.2f;%.2f;%.2f]", goal->goal.pose.position.x, goal->goal.pose.position.y, goal->goal.pose.position.z);
    (void)uuid;
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
    std::thread{std::bind(&PlannerServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ComputePathToPose::Result>();
        
    auto start_time = this->now();
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "map";       // ACTION CONFIRM THE FRAME MAP or odom or base_link
    pose.pose.position.x = goal->goal.pose.position.x;
    pose.pose.position.y = goal->goal.pose.position.y;
    pose.pose.position.z = goal->goal.pose.position.z;
  
    //tf2::Quaternion q;
    //q.setRPY( 0, 0, msg.value()[i].theta );  // Create this quaternion from roll/pitch/yaw (in radians)
    //q.normalize();
  
    pose.pose.orientation.x = goal->goal.pose.orientation.x; //q[0];
    pose.pose.orientation.y = goal->goal.pose.orientation.y; //q[1];
    pose.pose.orientation.z = goal->goal.pose.orientation.z; //q[2];
    pose.pose.orientation.w = goal->goal.pose.orientation.w; //q[3];
    
    result->path.poses.push_back(pose);
    //for (int i = 1; (i < 20) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
    //  if (goal_handle->is_canceling()) {
    //    result->planning_time = this->now() - start_time;
    //    goal_handle->canceled(result);
    //    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    //    return;
    //  }
      // Update path
    //  result->path.push_back();

//      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      //goal_handle->publish_feedback(feedback);
      //RCLCPP_INFO(this->get_logger(), "Publish feedback");

    //  loop_rate.sleep();
    //}

    // Check if goal is done
    if (rclcpp::ok()) {
//      result->sequence = sequence;
      result->planning_time = this->now() - start_time;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class PlannerServer

}  // namespace navigation_lite

RCLCPP_COMPONENTS_REGISTER_NODE(navigation_lite::PlannerServer)
