#ifndef SONAR_PRECIPITATOR_H
#define SONAR_PRECIPITATOR_H

#include <chrono>
#include <memory>
#include <thread>

#include "navigation_lite/sensor.h"
#include "navigation_lite/conversions.h"

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "navigation_interfaces/srv/save_map.hpp"
#include "navigation_interfaces/srv/load_map.hpp"
#include "navigation_interfaces/srv/reset.hpp"

#include "navigation_interfaces/msg/ufo_map_stamped.hpp"

#include <ufo/map/occupancy_map.h>

class SensorPrecipitator {
public:
  SensorPrecipitator(rclcpp::Node::SharedPtr node, const std::string map_topc, const std::string map_frame);
  ~SensorPrecipitator() {}
  std::shared_ptr<Sensor> add_sensor(const std::string sensor_topic, const std::string sensor_frame);
  
  bool load_from_file(std::string filename);
  bool write_to_file(std::string filename, bool compressed, int depth);
  bool reset(double resolution, int depth_levels); 

private:
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    std::string frame_;
        
    void execute(double rate);

    rclcpp::Publisher<navigation_interfaces::msg::UfoMapStamped>::SharedPtr map_publisher_;
    
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;

    std::vector<std::shared_ptr<Sensor> > sensors;
  
    std::shared_ptr<ufo::map::OccupancyMap> map;
   
};

#endif
