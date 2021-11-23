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

/**
  SensorPrecipitatir Class
  Purpose: Instantiates UFO Map and tfListener using a tfBuffer
  Adapted from the work of 
  @author Eliot Lim (github: @eliotlim)
  @version 1.0 (16/5/17)
*/

#include <navigation_lite/sensor_precipitator.h>

SensorPrecipitator::SensorPrecipitator(rclcpp::Node::SharedPtr node, std::string map_topic, std::string map_frame) :
                                     node_(node), frame_(map_frame) {
                                       
  // ROS Setup
  map_publisher_ = node_->create_publisher<navigation_interfaces::msg::UfoMapStamped>(map_topic, 3);
                                       
  // Transform listener
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  transform_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
 
  // Transform publisher
  tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
                                                                            
  // Spin up a new thread
  float rate = 20.0;
  std::thread{std::bind(&SensorPrecipitator::execute, this, _1), rate}.detach();                                     
}

std::shared_ptr<Sensor> SensorPrecipitator::add_sensor(std::string topic, std::string frame) {
  auto sensor_ptr = std::shared_ptr<Sensor>(new Sensor(node_, topic, frame));
  sensors.push_back(sensor_ptr);
  RCLCPP_INFO(node_->get_logger(), "Inserted Sensor - Topic: %s, Frame: %s", topic.c_str(), frame.c_str());
  return sensor_ptr;
}

void SensorPrecipitator::execute(double rate) {
  rclcpp::Rate loop_rate(rate);
  
  // Create a UFOMap
  
  // Build a UFO map
  // 25 cm voxel size            
  double resolution = 0.25;   
  node_->declare_parameter<double>("map_resolution", 0.25);   // use resolution 0.25.  Can then query the map at 0.5 and 1.0
  node_->get_parameter("map_resolution", resolution);
                                       
  // Maximum range to integrate, in meters.
  // Set to negative value to ignore maximum range.
  double max_range = 7.0;

  // The depth at which free space should be cleared.
  // A higher value significantly increases the integration speed
  // for smaller voxel sizes.
  ufo::map::DepthType integration_depth = 1;
  // Will free space at resolution * 2^(integration_depth) voxel size.

  // Some translation [x, y, z]
  ufo::math::Vector3 translation(0.0, 0.0, 0.0); 

  // Some rotation (w, x, y, z)
  ufo::math::Quaternion rotation(1.0, 0.0, 0.0, 0.0);

  ufo::math::Pose6 frame_origin(translation, rotation);

  ufo::math::Vector3 sensor_origin(translation);    
                                       
  map = std::make_shared<ufo::map::OccupancyMap>(resolution);                                     

  // Point cloud
  ufo::map::PointCloud cloud;

  // Fill point cloud
  
  while (rclcpp::ok()) {
    //pcl::PointCloud<pcl::PointXYZ>::SharedPtr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    //cloud->header.frame_id = frame_;
    //cloud->height = 1;
    cloud.clear();

    // Convert all Sensor readings to Points
    for (std::vector<std::shared_ptr<Sensor>>::iterator sensorIt = sensors.begin(); sensorIt != sensors.end(); ++sensorIt) { 
        std::shared_ptr<Sensor> sensor = *sensorIt;
        
      // Publish sensor transform if available
      if (sensor->transform_) {
        sensor->get_transform()->header.stamp = node_->get_clock()->now();
        tf_publisher_->sendTransform( * sensor->get_transform());
      }

      // Check Sensor Range Validity
      if (sensor->get_range() < 0) { continue; }

      // Get StampedTransform for Sensor
      geometry_msgs::msg::TransformStamped transform;
      try {
        // Calling lookupTransform with tf2::TimePointZero
        // results in the latest available transform
        // Swapped point_cloud->header.frame_id with "map"
        // This is dependent on a map->odom->base_link->"sensor" transform chain being in place!!!!! 
        transform = tf_buffer_->lookupTransform(frame_,
                                                   sensor->frame_,
                                                   tf2::TimePointZero);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s : %s", 
                    sensor->frame_.c_str(), frame_.c_str(), ex.what());
        continue; // skip this reading
      }

      // Transform the range reading into a point
      geometry_msgs::msg::PointStamped pt;
      pt.point.x = sensor->get_range();
      geometry_msgs::msg::PointStamped point_out;

      tf2::doTransform(pt, point_out, transform);

      // Store the point in map
      std::shared_ptr<ufo::map::Point3> ufo_point = std::make_shared<ufo::map::Point3>();
      ufo_point->x() = point_out.point.x;
      ufo_point->y() = point_out.point.y;
      ufo_point->z() = point_out.point.z;
      
      cloud.push_back(*ufo_point);

    }
    
    // Specify if the point cloud should be transformed in parallel or not.
    bool parallel = true;
    // Transform point cloud to correct frame
    cloud.transform(frame_origin, parallel);

    // Integrate point cloud into UFOMap
    map->insertPointCloudDiscrete(sensor_origin, cloud, max_range, integration_depth);

    // Compose the map message and publish
    rclcpp::Time now = node_->get_clock()->now();
    bool compress = false;
    
    ufo::map::DepthType pub_depth = 0;
    
    
    auto message = std::make_shared<navigation_interfaces::msg::UfoMapStamped>();
    //navigation_interfaces::msg::UfoMapStamped::Ptr message(new navigation_interfaces::msg::UfoMapStamped);
    //Convert UFOMap to ROS Message
    if (navigation_interfaces::ufoToMsg(*map, message->map, ufo::geometry::BoundingVolume(), compress, pub_depth, 1, 0)) {
      message->header.stamp = now;
      message->header.frame_id = frame_;    // Should be "map"
      map_publisher_->publish(*message);
      RCLCPP_DEBUG(node_->get_logger(), "Map published");
    }  
    loop_rate.sleep();
  }
}


bool SensorPrecipitator::load_from_file(std::string filename)
{
  return map->read(filename);
}

bool SensorPrecipitator::write_to_file(std::string filename, bool compressed, int depth)
{
  return map->write(filename, compressed, depth);

}

bool SensorPrecipitator::reset(double resolution, int depth_levels)
{
  map->clear(resolution, depth_levels);
  return true;
}

