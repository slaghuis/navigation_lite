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

#include "navigation_lite/action_spin.h"

namespace NavigationNodes
{
 
BT::NodeStatus NavLiteSpinAction::tick()
{
  using namespace std::placeholders;
  
  action_status = ActionStatus::VIRGIN;
   
  BT::Optional<float> msg = getInput<float>("radians");
  // Check if optional is valid. If not, throw its error
  if (!msg)
  {
      throw BT::RuntimeError("missing required input [radians]: ", 
                             msg.error() );
  }

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    throw BT::RuntimeError("Action server not available. Cannot Spin the drone.  Failing.");
  }
  
  // Call the action server
  auto goal_msg = Spin::Goal();
  goal_msg.target_yaw = msg.value();
    
  auto send_goal_options = rclcpp_action::Client<Spin>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&NavLiteSpinAction::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&NavLiteSpinAction::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&NavLiteSpinAction::result_callback, this, _1);

  future_goal_handle_ = std::make_shared<
      std::shared_future<GoalHandleSpin::SharedPtr>>(
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options));
  
  _halt_requested.store(false);
  while (!_halt_requested && ((action_status == ActionStatus::VIRGIN) || (action_status == ActionStatus::PROCESSING)))
  {   
    std::this_thread::sleep_for( std::chrono::milliseconds(10) );
  }  
  
  cleanup();
  return (action_status == ActionStatus::SUCCEEDED) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
    
void NavLiteSpinAction::cleanup() 
{

  if( _halt_requested )
  {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] - Cleaning up after a halt()", name().c_str());
    try {
      goal_handle_ = future_goal_handle_->get();
      this->client_ptr_->async_cancel_goal(goal_handle_); // Request a cancellation.
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "[%s] - Exception caught");
    }
  } else {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] - Cleaning up after SUCCESS", name().c_str());
    // The Action Server Request completed as per normal.  Nothng to do.
  }
}
  
void NavLiteSpinAction::halt() 
{
  _halt_requested.store(true); 
  action_status = ActionStatus::CANCELED;
}
  
////  ROS2 Action Client Functions ////////////////////////////////////////////
void NavLiteSpinAction::goal_response_callback(std::shared_future<GoalHandleSpin::SharedPtr> future)
  {
    //future_goal_handle_ = future;
  
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      action_status = ActionStatus::REJECTED;
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, Waiting for result");
      action_status = ActionStatus::PROCESSING;
    }
  }

  void NavLiteSpinAction::feedback_callback(
    GoalHandleSpin::SharedPtr,
    const std::shared_ptr<const Spin::Feedback> feedback)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Angular distance travelled: %.2f radians.", feedback->angular_distance_traveled);
  }

  void NavLiteSpinAction::result_callback(const GoalHandleSpin::WrappedResult & result)
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
    
    RCLCPP_DEBUG(node_->get_logger(), "Spinning completed in %d seconds", result.result->total_elapsed_time.sec);
  }  
  
}  // namespace