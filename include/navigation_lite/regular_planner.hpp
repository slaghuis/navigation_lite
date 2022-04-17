#ifndef NAVIGATION_LITE_REGULAR_PLANNER_HPP
#define NAVIGATION_LITE_REGULAR_PLANNER_HPP

#include <string>              // std::string
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace navigation_lite
{
  class RegularPlanner
  {
    public:
      virtual void configure(const rclcpp::Node::SharedPtr parent, 
                             std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                             std::shared_ptr<octomap::OcTree> costmap ) = 0;
    
      virtual nav_msgs::msg::Path createPlan( const geometry_msgs::msg::PoseStamped & start,
                                              const geometry_msgs::msg::PoseStamped & goal) = 0;
    
      virtual void initialize(double side_length) = 0;
      virtual double area() = 0;
      virtual ~RegularPlanner(){}

    protected:
      RegularPlanner(){}
  };
}  // namespace navigation_lite

#endif  // NAVIGATION_LITE_REGULAR_PLANNER_HPP