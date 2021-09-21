#ifndef ACTION_FollowWaypoints_H
#define ACTION_FollowWaypoints_H

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

#include "navigation_interfaces/action/follow_waypoints.hpp"

#include "navigation_lite/navigation_server.h"
#include "navigation_lite/pose_3D.h"

#include "rclcpp/rclcpp.hpp"

namespace NavigationNodes
{

class NavLiteFollowWaypointsAction : public BT::AsyncActionNode
{
  public:
    using FollowWaypoints = navigation_interfaces::action::FollowWaypoints;
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    NavLiteFollowWaypointsAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    { }

    void init(rclcpp::Node::SharedPtr node) {
      node_ = node;
      
       this->client_ptr_ = rclcpp_action::create_client<FollowWaypoints>(
       node_,
      "nav_lite/follow_waypoints");
    }
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::vector<Pose3D>>("path") };
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
    
    rclcpp_action::Client<FollowWaypoints>::SharedPtr client_ptr_;

    std::shared_ptr<std::shared_future<GoalHandleFollowWaypoints::SharedPtr>> future_goal_handle_;
    GoalHandleFollowWaypoints::SharedPtr goal_handle_;
    
    void goal_response_callback(std::shared_future<GoalHandleFollowWaypoints::SharedPtr> future);
    void feedback_callback( 
      GoalHandleFollowWaypoints::SharedPtr,
      const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
    void result_callback(const GoalHandleFollowWaypoints::WrappedResult & result);


};
     
} // Namespace

#endif // ACTION_FollowWaypoints_H

