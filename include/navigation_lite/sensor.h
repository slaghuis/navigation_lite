#ifndef SENSOR_H
#define SENSOR_H

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


using std::placeholders::_1;

class Sensor {
public:
  // Pointer to the ROS node
  rclcpp::Node::SharedPtr node_;         
  std::string topic_;
  std::string frame_;
  bool transform_;
    
  Sensor(rclcpp::Node::SharedPtr node, std::string topic, std::string frame);
  //~Sensor() {}

  void set_transform(const geometry_msgs::msg::TransformStamped transformS);
  std::shared_ptr<geometry_msgs::msg::TransformStamped> get_transform();
  float get_range();

private:

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_;
  std::shared_ptr<sensor_msgs::msg::Range> range_msg;
  std::shared_ptr<geometry_msgs::msg::TransformStamped> transformS;
    
  void range_callback(const sensor_msgs::msg::Range::SharedPtr msg);
};

#endif
