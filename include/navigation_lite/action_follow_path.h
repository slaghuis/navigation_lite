#ifndef ACTION_FollowPath_H
#define ACTION_FollowPath_H

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>  //Dealing with a vector of poses to define waypoints

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "navigation_interfaces/action/follow_path.hpp"

#include "navigation_lite/navigation_server.h"
#include "navigation_lite/pose_3D.h"

#include "rclcpp/rclcpp.hpp"

namespace NavigationNodes
{

class NavLiteFollowPathAction : public BT::AsyncActionNode
{
  public:
    using FollowPath = navigation_interfaces::action::FollowPath;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    NavLiteFollowPathAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
      
       this->client_ptr_ = rclcpp_action::create_client<FollowPath>(
       node_,
      "nav_lite/follow_path");
    }
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::vector<Pose3D>>("path"),
                BT::InputPort<std::string>("controller") };
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
    
    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;

    std::shared_ptr<std::shared_future<GoalHandleFollowPath::SharedPtr>> future_goal_handle_;
    GoalHandleFollowPath::SharedPtr goal_handle_;
    
    void goal_response_callback(std::shared_future<GoalHandleFollowPath::SharedPtr> future);
    void feedback_callback( 
      GoalHandleFollowPath::SharedPtr,
      const std::shared_ptr<const FollowPath::Feedback> feedback);
    void result_callback(const GoalHandleFollowPath::WrappedResult & result);


};
     
} // Namespace

#endif // ACTION_FollowPath_H

