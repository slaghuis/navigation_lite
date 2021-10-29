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

#include "navigation_lite/action_wait.h"

namespace NavigationNodes
{
 
BT::NodeStatus NavLiteWaitAction::tick()
{
  using namespace std::placeholders;
  
  action_status = ActionStatus::VIRGIN;
   
  BT::Optional<int> msg = getInput<int>("seconds");
  // Check if optional is valid. If not, throw its error
  if (!msg)
  {
      throw BT::RuntimeError("missing required input [seconds]: ", 
                             msg.error() );
  }

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    throw BT::RuntimeError("Action server not available. Cannot Wait the drone.  Failing.");
  }
  
  // Call the action server
  auto goal_msg = Wait::Goal();
  goal_msg.time.sec = msg.value();
  goal_msg.time.nanosec = 0;
    
  auto send_goal_options = rclcpp_action::Client<Wait>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&NavLiteWaitAction::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&NavLiteWaitAction::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&NavLiteWaitAction::result_callback, this, _1);

  future_goal_handle_ = std::make_shared<
      std::shared_future<GoalHandleWait::SharedPtr>>(
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options));
  
  _halt_requested.store(false);
  while (!_halt_requested && ((action_status == ActionStatus::VIRGIN) || (action_status == ActionStatus::PROCESSING)))
  {   
    std::this_thread::sleep_for( std::chrono::milliseconds(10) );
  }    
  
  cleanup();
  node_->increment_recovery_count();
  return (action_status == ActionStatus::SUCCEEDED) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
    
void NavLiteWaitAction::cleanup() 
{

  if( _halt_requested )
  {
    RCLCPP_INFO(node_->get_logger(), "[%s] - Cleaning up after a halt()", name().c_str());
    try {
      goal_handle_ = future_goal_handle_->get();
      this->client_ptr_->async_cancel_goal(goal_handle_); // Request a cancellation.
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "[%s] - Exception caught");
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "[%s] - Cleaning up after SUCCESS", name().c_str());
    // The Action Server Request completed as per normal.  Nothng to do.
  }
}
  
void NavLiteWaitAction::halt() 
{
  _halt_requested.store(true);  
  action_status = ActionStatus::CANCELED;
}
  
////  ROS2 Action Client Functions ////////////////////////////////////////////
void NavLiteWaitAction::goal_response_callback(std::shared_future<GoalHandleWait::SharedPtr> future)
  {
  
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      action_status = ActionStatus::REJECTED;
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
      action_status = ActionStatus::PROCESSING;
    }
  }

  void NavLiteWaitAction::feedback_callback(
    GoalHandleWait::SharedPtr,
    const std::shared_ptr<const Wait::Feedback> feedback)
  {
    RCLCPP_INFO(node_->get_logger(), "Time Left: %ds", feedback->time_left.sec);
  }

  void NavLiteWaitAction::result_callback(const GoalHandleWait::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        action_status = ActionStatus::SUCCEEDED;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(node_->get_logger(), "Goal was aborted");
        action_status = ActionStatus::ABORTED;
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(node_->get_logger(), "Goal was canceled");
        action_status = ActionStatus::CANCELED;
        return;
      default:
        RCLCPP_WARN(node_->get_logger(), "Unknown result code");
        action_status = ActionStatus::UNKNOWN;
        return;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Waiting completed in %d seconds", result.result->total_elapsed_time.sec);
  }  
  
}  // namespace