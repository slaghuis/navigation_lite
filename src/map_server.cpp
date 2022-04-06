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
 * Reads SensorMsgs::msg::PointCloud2 and maintains a UFO Map.  See
 * https://github.com/UnknownFreeOccupied/ufomap by D. Duberg, KTH Royal 
 * Institute of Technology. 
 * Publishes this map in a custom message type.
 * ***********************************************************************/

#include <functional>
#include <future>
#include <numeric>
#include <memory>
#include <string>
#include <sstream>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <ufo/map/occupancy_map.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "navigation_lite/ufomap_ros_msgs_conversions.h"
#include "navigation_lite/ufomap_ros_conversions.h"

#include "navigation_interfaces/srv/save_map.hpp"
#include "navigation_interfaces/srv/load_map.hpp"
#include "navigation_interfaces/srv/reset.hpp"
#include "navigation_interfaces/msg/ufo_map_stamped.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace navigation_lite
{
class MapServer : public rclcpp::Node
{
public:

  explicit MapServer(const rclcpp::NodeOptions & options)
  : Node("map_server", options)
  {
    // Declare and read some parameters
    map_frame_id_ = this->declare_parameter<std::string>("map_frame", "error");
    map_topic_ = this->declare_parameter<std::string>("map_topic", "error");
    pointcloud_topic_ = this->declare_parameter<std::string>("pointcloud_topic", "pointcloud");

    this->declare_parameter<double>("map_resolution", 0.1);      // Spatial extent of tree is 6553,6 meters in either direction
    
    max_range_ = this->declare_parameter<double>("max_range", -1.0);      // Max range (m) when inserting data into map
    insert_depth_ = this->declare_parameter<int>("insert_depth", 0);      // Integration depth
    simple_ray_casting_ = this->declare_parameter<bool>("simple_ray_casting", false);  
    early_stopping_ = this->declare_parameter<int>("early_stopping", 0);  //Early Stopping
    async_ = this->declare_parameter<bool>("async", false);               // Async integration
    
    clear_robot_     = this->declare_parameter<bool>("clear_robot", false);
    robot_frame_id_  = this->declare_parameter<std::string>("robot_frame_id", "base_link");
    robot_height_    = this->declare_parameter<double>("robot_height", 0.4);    // Robot height(m)
    robot_radius_    = this->declare_parameter<double>("robot_radius", 0.5);    // Robot radius(m)
    clearing_depth_  = this->declare_parameter<int>("clearing_depth", 0);       // Clearing depth
    
    // Kick off a init routine
    this->init_timer_ = this->create_wall_timer( 
      std::chrono::milliseconds(500), 
      std::bind(&MapServer::init, this) );
  }

private:
  // Parameters
  std::string map_frame_id_;
  std::string map_topic_;    
  std::string pointcloud_topic_;

  // Parameters for integration
  double max_range_;
  ufo::map::DepthType insert_depth_;
  bool simple_ray_casting_;
  unsigned int early_stopping_;
  bool async_;
  
  // Parameters for Clear Robot
  bool clear_robot_;
  std::string robot_frame_id_;
  double robot_height_, robot_radius_;
  int clearing_depth_;
    
  // Parameters for Publishing
  bool compress_;
	bool update_part_of_map_;
	ufo::map::DepthType publish_depth_;
	std::future<void> update_async_handler_;
  
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
  rclcpp::Service<navigation_interfaces::srv::LoadMap>::SharedPtr load_service;
  rclcpp::Service<navigation_interfaces::srv::SaveMap>::SharedPtr save_service;
  rclcpp::Service<navigation_interfaces::srv::Reset>::SharedPtr   reset_service;
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<navigation_interfaces::msg::UfoMapStamped>::SharedPtr map_publisher_;
    
  std::shared_ptr<ufo::map::OccupancyMap> map_;  
  
  void init()
  {
    // run this timer only once
    init_timer_->cancel();
  
    
    // Initiate the publisher
    map_publisher_ = this->create_publisher<navigation_interfaces::msg::UfoMapStamped>(map_topic_, 3);
    
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Setup the UFO map
    double resolution;   
    this->get_parameter("map_resolution", resolution);
    
    map_ = std::make_shared<ufo::map::OccupancyMap>(resolution); 
        
    // Start listening for pointcloud messages
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, 10, std::bind(&MapServer::topic_callback, this, std::placeholders::_1));  
      
    pub_timer_ = this->create_wall_timer(
      1000ms, std::bind(&MapServer::publish_map, this));  
    //publish_map();  // Send the first map, and then only when it has been updated.
  
    // Create simple services
    load_service = this->create_service<navigation_interfaces::srv::LoadMap>("nav_lite/load_map", std::bind(&MapServer::load_map, this, _1, _2));
    save_service = this->create_service<navigation_interfaces::srv::SaveMap>("nav_lite/save_map", std::bind(&MapServer::save_map, this, _1, _2));
    reset_service = this->create_service<navigation_interfaces::srv::Reset>("nav_lite/reset_map", std::bind(&MapServer::reset_map, this, _1, _2));

  }

  void publish_map()
  {
    // Compose the map message and publish
    rclcpp::Time now = this->get_clock()->now();
    
    // If the UFOMap should be compressed using LZ4.
    // Good if you are sending the UFOMap between computers.
    bool compress = false;
    
    // Lowest depth to publish.
    // Higher value means less data to transfer, good in
    // situation where the data rate is low.
    // Many nodes do not require detailed maps as well.    
    ufo::map::DepthType pub_depth = 0;
    
    auto message = std::make_shared<navigation_interfaces::msg::UfoMapStamped>();
    //navigation_interfaces::msg::UfoMapStamped::Ptr message(new navigation_interfaces::msg::UfoMapStamped);
    //Convert UFOMap to ROS Message
    if (ufomap_msgs::ufoToMsg(*map_, message->map, ufo::geometry::BoundingVolume(), compress, pub_depth, 1, 0)) {
      message->header.stamp = now;
      message->header.frame_id = map_frame_id_;    // Should be "map"
      map_publisher_->publish(*message);
      RCLCPP_DEBUG(this->get_logger(), "Map published");
    }  
  }
  /*
  ufo::math::Pose6 rosToUfo(geometry_msgs::msg::Transform const& transform)
  {
	  return ufo::math::Pose6(transform.translation.x, transform.translation.y,
	                          transform.translation.z, transform.rotation.w,
	                          transform.rotation.x, transform.rotation.y,
	                          transform.rotation.z);
  }

 
    
  void rosToUfo(sensor_msgs::msg::PointCloud2 const& cloud_in,
              ufo::map::PointCloudColor& cloud_out)
  {
	  cloud_out.reserve(cloud_in.data.size() / cloud_in.point_step);

	  bool has_x, has_y, has_z, has_rgb;
	  getFields(cloud_in, has_x, has_y, has_z, has_rgb);

	  if (!has_x || !has_y || !has_z) {
		  throw std::runtime_error("cloud_in missing one or more of the xyz fields");
	  }

	  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x");
	  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_in, "y");
	  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_in, "z");

	  if (has_rgb) {
		  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(cloud_in, "r");
		  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(cloud_in, "g");
		  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(cloud_in, "b");

		  for (; iter_x != iter_x.end();
		     ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
			  if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z) &&
			      !std::isnan(*iter_r) && !std::isnan(*iter_g) && !std::isnan(*iter_b)) {
				  cloud_out.push_back(
				      ufo::map::Point3Color(*iter_x, *iter_y, *iter_z, *iter_r, *iter_g, *iter_b));
			  }
		  }

	  } else {
		  // TODO: Should this throw?
		  // throw std::runtime_error("cloud_in missing one or more of the rgb fields");

		  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
			  if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
				  cloud_out.push_back(ufo::map::Point3Color(*iter_x, *iter_y, *iter_z));
			  }
		  }
    }
  }
  */  
  void topic_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    // Get transform
    ufo::math::Pose6 transform;
    
    try {
      // Lookup transform
      geometry_msgs::msg::TransformStamped tf_trans = tf_buffer_->lookupTransform(map_frame_id_, 
                                                                              msg->header.frame_id, 
                                                                              tf2::TimePointZero);
      // Convert ROS transform to UFO transform
      transform = ufomap_ros::rosToUfo(tf_trans.transform);
      
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(
            this->get_logger(), "Could not transform %s to %s: %s",
            "map", msg->header.frame_id, ex.what());
      return;
    }

    ufo::map::PointCloudColor cloud;
    // Convert ROS point cloud to UFO point cloud
    ufomap_ros::rosToUfo(*msg, cloud);
    // Transform point cloud to correct frame, do it in parallel (second param true)
    cloud.transform(transform, true);

    // Integrate point cloud into UFOMap, no max range (third param -1), 
    // free space at depth level 1 (fourth param 1)
    //map_->insertPointCloudDiscrete(transform.translation(), cloud, -1, 1);    
    map_->insertPointCloudDiscrete(transform.translation(), cloud, max_range_, insert_depth_, simple_ray_casting_, early_stopping_, async_);    
    
    if (clear_robot_) {
      try {
        // Lookup transform
        geometry_msgs::msg::TransformStamped tf_trans = tf_buffer_->lookupTransform(map_frame_id_, 
                                                                                    robot_frame_id_, 
                                                                                    tf2::TimePointZero);
        // Convert ROS transform to UFO transform
        transform = ufomap_ros::rosToUfo(tf_trans.transform);
      
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(
              this->get_logger(), "Could not transform %s to %s: %s",
              "map", msg->header.frame_id, ex.what());
        return;
      }
      
      ufo::map::Point3 r(robot_radius_, robot_radius_, robot_height_ / 2.0);
			ufo::geometry::AABB aabb(transform.translation() - r,
				                             transform.translation() + r);
			//map_->setValueVolume(aabb, map_->getClampingThresMin(), clearing_depth_);  // /usr/local/include/ufo/map/occupancy_map_base.h:508:4: error: call of overloaded ‘setOccupancy(float&, double)’ is ambiguous
      
    }
    
    
    // Map has changed, publish
    // publish_map();
      
  }
  

  bool save_map(const std::shared_ptr<navigation_interfaces::srv::SaveMap::Request> request,
          std::shared_ptr<navigation_interfaces::srv::SaveMap::Response> response)
  {
//    response->success = map_->write(request->filename, false, request->depth);
        
		ufo::geometry::BoundingVolume bv =
			        ufomap_msgs::msgToUfo(request->bounding_volume);
			    response->success = map_->write(request->filename, bv, request->compress,
			                                 request->depth, 1, request->compression_level);
	  return true;
  }

  void load_map(const std::shared_ptr<navigation_interfaces::srv::LoadMap::Request> request,
          std::shared_ptr<navigation_interfaces::srv::LoadMap::Response> response)
  {
    response->success = map_->read(request->filename);
  }

  void reset_map(const std::shared_ptr<navigation_interfaces::srv::Reset::Request> request,
          std::shared_ptr<navigation_interfaces::srv::Reset::Response> response)
  {    
    map_->clear(request->new_resolution, request->new_depth_levels);
    response->success = true;
  }

};  // class MapServer

}  // namespace navigation_lite

RCLCPP_COMPONENTS_REGISTER_NODE(navigation_lite::MapServer)
 