#ifndef ACTION_SPIN_H
#define ACTION_SPIN_H

#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "navigation_interfaces/action/spin.hpp"

#include "navigation_lite/navigation_server.h"

#include "rclcpp/rclcpp.hpp"

namespace NavigationNodes
{

//class NavLiteSpinAction : public BT::CoroActionNode
class NavLiteSpinAction : public BT::AsyncActionNode
{
  public:
    using Spin = navigation_interfaces::action::Spin;
    using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

    NavLiteSpinAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
      
       this->client_ptr_ = rclcpp_action::create_client<Spin>(
       node_,
      "nav_lite/spin");
    }
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<int>("radians") };
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
    
    rclcpp_action::Client<Spin>::SharedPtr client_ptr_;

    std::shared_ptr<std::shared_future<GoalHandleSpin::SharedPtr>> future_goal_handle_;
    GoalHandleSpin::SharedPtr goal_handle_;
    
    void goal_response_callback(std::shared_future<GoalHandleSpin::SharedPtr> future);
    void feedback_callback( 
      GoalHandleSpin::SharedPtr,
      const std::shared_ptr<const Spin::Feedback> feedback);
    void result_callback(const GoalHandleSpin::WrappedResult & result);

};
     
} // Namespace

#endif // ACTION_SPIN_H

