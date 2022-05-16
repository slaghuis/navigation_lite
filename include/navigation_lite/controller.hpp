#ifndef NAVIGATION_LITE_CONTROLLER_HPP
#define NAVIGATION_LITE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace navigation_lite
{
  class Controller
  {
    public:
      virtual void configure(const rclcpp::Node::SharedPtr parent, 
                             std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                             std::shared_ptr<octomap::OcTree> costmap ) = 0;
    
      virtual void setPath(const nav_msgs::msg::Path & path) = 0;
    
      virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & speed) = 0;
//        nav2_core::GoalChecker * goal_checker) = 0;
    
      virtual ~Controller() {}

    protected:
      Controller(){}
  };
} // namespace navigation_lite

#endif // NAVIGATION_LITE_CONTROLLER_HPP