#ifndef ACTION_WAIT_H
#define ACTION_WAIT_H

#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "navigation_interfaces/action/wait.hpp"

#include "navigation_lite/navigation_server.h"

#include "rclcpp/rclcpp.hpp"

namespace NavigationNodes
{

class NavLiteWaitAction : public BT::AsyncActionNode
{
  public:
    using Wait = navigation_interfaces::action::Wait;
    using GoalHandleWait = rclcpp_action::ClientGoalHandle<Wait>;

    NavLiteWaitAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
      
       this->client_ptr_ = rclcpp_action::create_client<Wait>(
       node_,
      "nav_lite/wait");
    }
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<int>("seconds") };
    }
    
    BT::NodeStatus tick() override;
    void halt() override;
    void cleanup();

  private:
    std::atomic_bool _halt_requested;
    NavigationNodes::ActionStatus action_status;
    
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    
    rclcpp_action::Client<Wait>::SharedPtr client_ptr_;

    std::shared_ptr<std::shared_future<GoalHandleWait::SharedPtr>> future_goal_handle_;
    GoalHandleWait::SharedPtr goal_handle_;
    
    void goal_response_callback(std::shared_future<GoalHandleWait::SharedPtr> future);
    void feedback_callback( 
      GoalHandleWait::SharedPtr,
      const std::shared_ptr<const Wait::Feedback> feedback);
    void result_callback(const GoalHandleWait::WrappedResult & result);

};
     
} // Namespace

#endif // ACTION_WAIT_H

