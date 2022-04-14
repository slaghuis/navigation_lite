#ifndef REGULAR_PLANNNER_HPP
#define REGULAR_PLANNNER_HPP

#include <string>              // std::string
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace planner_base
{
  class RegularPlanner
  {
    public:
      virtual void configure(const rclcpp::Node::SharedPtr parent, 
                             std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                             std::shared_ptr<octomap::OcTree> costmap ) = 0;
      virtual void activate() = 0;
      virtual void deactivate() = 0;
      virtual void cleanup() = 0;
      virtual nav_msgs::msg::Path createPlan( const geometry_msgs::msg::PoseStamped & start,
                                              const geometry_msgs::msg::PoseStamped & goal) = 0;
      virtual ~RegularPlanner(){}

    protected:
      RegularPlanner(){}
  };
}  // namespace planner_base

#endif  // REGULAR_PLANNNER_HPP
