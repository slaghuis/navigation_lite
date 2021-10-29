#ifndef READ_GOAL_H
#define READ_GOAL_H

#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "navigation_lite/navigation_server.h"
#include "navigation_lite/pose_3D.h"

#include "rclcpp/rclcpp.hpp"

namespace NavigationNodes
{

class NavLiteReadGoalAction : public BT::SyncActionNode
{
  public:
    NavLiteReadGoalAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // A node having ports MUST implement this STATIC method
    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<Pose3D>("pose") };
    }
    
    BT::NodeStatus tick() override;
   
    void init(rclcpp::Node::SharedPtr node, Pose3D goal) {
      node_ = node;
      goal_ = goal;
    }
    
  private:
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    Pose3D goal_;
};
       
} // Namespace

#endif // READ_GOAL_H

