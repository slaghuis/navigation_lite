// Copyright (c) 2021 Xeni Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* **********************************************************************
 * This is the love child between the sensor_pointcloud package 
 * [https://github.com/eliotlim/sensor_pointcloud] by Eliot Lim (github: @eliotlim)
 * and ufo_ros https://github.com/UnknownFreeOccupied/ufomap_ros by 
 * D. Duberg, KTH Royal Institute of Technology.  All elements taken were 
 * intergrated and rewritten to work in ROS Foxy by Eric Slaghuis (github @slaghuis).
 * Lim's work (under BSD license) to read sensor transformation from a 
 * parameter file, was used to populate a UFO Map pointcloud and pupulate an 
 * UFO map. The UFO map is then published as as a custom message type to the 
 * navigation lite stack, typically for the planner server.
 *
 * Subscribes to a dynamic list of sensor topics [sensor_msgs::msg::Range]
 * Publishes a UnknownFreeOccupied map [navigation_interfaces::msg::map]
 * exposes three simple services to Load and save the map, and to clear the 
 * map.
 * ***********************************************************************/

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "navigation_lite/sensor_precipitator.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace navigation_lite
{
class MapServer : public rclcpp::Node
{
public:

  explicit MapServer(const rclcpp::NodeOptions & options)
  : Node("map_server", options)
  {
    using namespace std::placeholders;
    
    // Create simple services
    load_service = this->create_service<navigation_interfaces::srv::LoadMap>("nav_lite/load_map", std::bind(&MapServer::load_map, this, _1, _2));
    save_service = this->create_service<navigation_interfaces::srv::SaveMap>("nav_lite/save_map", std::bind(&MapServer::save_map, this, _1, _2));
    reset_service = this->create_service<navigation_interfaces::srv::Reset>("nav_lite/reset_map", std::bind(&MapServer::reset_map, this, _1, _2));
   
    // Declare some parameters
    this->declare_parameter<std::string>("map_frame", "error");
    this->declare_parameter<std::string>("map_topic", "error");
    this->declare_parameter<std::string>("base_link_frame", "base_link");
    this->declare_parameter("sensors");
    
    // Kick off a init routine
    this->init_timer_ = this->create_wall_timer( 
      std::chrono::milliseconds(500), 
      std::bind(&MapServer::init, this) );
  }

  void init()
  {
    // run this timer only once
    init_timer_->cancel();
  
    // read parameters
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("map_topic", map_topic_);
    this->get_parameter("base_link_frame", base_link_frame_);
    RCLCPP_DEBUG(this->get_logger(), "Map Frame: %s", map_frame_.c_str());
  
    rclcpp::Parameter sensor_param("sensors", std::vector<std::string>({}));
    this->get_parameter("sensors", sensor_param);
  
    // Create SensorPrecipitator Object
    auto node_ptr = shared_from_this();
    precipitator = std::make_shared<SensorPrecipitator>(node_ptr, map_topic_, map_frame_);
  
    sensors = sensor_param.as_string_array(); 
    if (sensors.size() == 0) RCLCPP_WARN(this->get_logger(), "No Sensors Configured");
  
    for (std::vector<std::string>::iterator sensorNameIt = sensors.begin(); sensorNameIt != sensors.end(); ++sensorNameIt) {
      std::string sensor_topic, sensor_frame;
    
      this->declare_parameter<std::string>(*sensorNameIt + ".topic", "error");
      this->get_parameter(*sensorNameIt + ".topic", sensor_topic);

      this->declare_parameter<std::string>(*sensorNameIt + ".transform.frame", "error");
      this->get_parameter(*sensorNameIt + ".transform.frame", sensor_frame);
        
      RCLCPP_INFO(this->get_logger(), "Sensor Parameters Loaded - Topic: %s Frame: %s ", sensor_topic.c_str(), sensor_frame.c_str());
    
      std::shared_ptr<Sensor> s = precipitator->add_sensor(sensor_topic, sensor_frame);
    
      bool loadTransform = false;
      double translation[3] = {0, 0, 0};
    
      // load x,y,z parameters
      for(char c = 'X'; c<='Z'; c++)
      {
        std::string paramStr = *sensorNameIt + ".transform.pos" + c;
        this->declare_parameter<float>(*sensorNameIt + ".transform.pos" + c, 0.0);
        if (this->get_parameter(paramStr, translation[c-'X'])) {
          loadTransform = true;
          RCLCPP_DEBUG(this->get_logger(), "Loading transform for %s: %f", paramStr.c_str(), translation[c-'X']);
        }
      }
    
      if (loadTransform) {
        float roll, pitch, yaw;
        this->declare_parameter<float>(*sensorNameIt + ".transform/roll", 0.0);
        this->get_parameter(*sensorNameIt + ".transform/roll",  roll);
        this->declare_parameter<float>(*sensorNameIt + ".transform/pitch", 0.0);
        this->get_parameter(*sensorNameIt + ".transform/pitch", pitch);
        this->declare_parameter<float>(*sensorNameIt + ".transform/yaw", 0.0);
        this->get_parameter(*sensorNameIt + ".transform/yaw",   yaw);
      
        // Load role, pitch and yaw from parameters
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
      
        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = now;
        t.header.frame_id = base_link_frame_;  // Changed from map frame.
        //t.child_frame_id = <<<<<<will be set before message sent

        t.transform.translation.x = translation[0];
        t.transform.translation.y = translation[1];
        t.transform.translation.z = translation[2];
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
      
        s->set_transform(t);
           
      }
    }   
  }

private:
  // Parameters
  std::string map_frame_;
  std::string map_topic_;    
  std::string base_link_frame_;
  std::vector<std::string> sensors = {};
    
  std::shared_ptr<SensorPrecipitator> precipitator;
    
  rclcpp::TimerBase::SharedPtr init_timer_;
    
  rclcpp::Service<navigation_interfaces::srv::LoadMap>::SharedPtr load_service;
  rclcpp::Service<navigation_interfaces::srv::SaveMap>::SharedPtr save_service;
  rclcpp::Service<navigation_interfaces::srv::Reset>::SharedPtr   reset_service;
    

  void save_map(const std::shared_ptr<navigation_interfaces::srv::SaveMap::Request> request,
          std::shared_ptr<navigation_interfaces::srv::SaveMap::Response> response)
  {
    response->success = precipitator->write_to_file(request->filename, request->depth, request->depth);
  }

  void load_map(const std::shared_ptr<navigation_interfaces::srv::LoadMap::Request> request,
          std::shared_ptr<navigation_interfaces::srv::LoadMap::Response> response)
  {
    response->success = precipitator->load_from_file(request->filename);
  }

  void reset_map(const std::shared_ptr<navigation_interfaces::srv::Reset::Request> request,
          std::shared_ptr<navigation_interfaces::srv::Reset::Response> response)
  {
    response->success = precipitator->reset(request->new_resolution, request->new_depth_levels); 
  }

};  // class MapServer

}  // namespace navigation_lite

RCLCPP_COMPONENTS_REGISTER_NODE(navigation_lite::MapServer)
 