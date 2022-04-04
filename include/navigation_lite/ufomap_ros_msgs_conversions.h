/**
 * UfoMap ROS message conversions
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufo_ros
 * License: BSD 3
 *
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFOMAP_ROS_MSGS_CONVERSIONS_H
#define UFOPAM_ROS_MSGS_CONVERSIONS_H

// UFO
#include <ufo/geometry/aabb.h>
#include <ufo/geometry/bounding_volume.h>
#include <ufo/geometry/frustum.h>
#include <ufo/geometry/line_segment.h>
#include <ufo/geometry/obb.h>
#include <ufo/geometry/plane.h>
#include <ufo/geometry/point.h>
#include <ufo/geometry/ray.h>
#include <ufo/geometry/sphere.h>

#include <ufo/map/occupancy_map.h>

// Navigation Messages for UFO 
#include <navigation_interfaces/msg/aabb.hpp>
#include <navigation_interfaces/msg/bounding_volume.hpp>
#include <navigation_interfaces/msg/frustum.hpp>
#include <navigation_interfaces/msg/line_segment.hpp>
#include <navigation_interfaces/msg/obb.hpp>
#include <navigation_interfaces/msg/plane.hpp>
#include <navigation_interfaces/msg/point.hpp>
#include <navigation_interfaces/msg/ray.hpp>
#include <navigation_interfaces/msg/sphere.hpp>
#include <navigation_interfaces/msg/ufo_map.hpp>

// STD
#include <type_traits>
#include <ios>
#include <iostream>
#include <sstream>
#include <string>

namespace ufomap_msgs
{
//
// ROS message type to UFO type
//

ufo::geometry::Point msgToUfo(navigation_interfaces::msg::Point const& point);

ufo::geometry::AABB msgToUfo(navigation_interfaces::msg::Aabb const& Aabb);

ufo::geometry::Plane msgToUfo(navigation_interfaces::msg::Plane const& plane);

ufo::geometry::Frustum msgToUfo(navigation_interfaces::msg::Frustum const& frustum);

ufo::geometry::LineSegment msgToUfo(navigation_interfaces::msg::LineSegment const& line_segment);

ufo::geometry::OBB msgToUfo(navigation_interfaces::msg::Obb const& Obb);

ufo::geometry::Ray msgToUfo(navigation_interfaces::msg::Ray const& ray);

ufo::geometry::Sphere msgToUfo(navigation_interfaces::msg::Sphere const& sphere);

ufo::geometry::BoundingVolume msgToUfo(navigation_interfaces::msg::BoundingVolume const& msg);

//
// UFO type to ROS message type
//

navigation_interfaces::msg::Point ufoToMsg(ufo::geometry::Point const& point);

navigation_interfaces::msg::Aabb ufoToMsg(ufo::geometry::AABB const& Aabb);

navigation_interfaces::msg::Plane ufoToMsg(ufo::geometry::Plane const& plane);

navigation_interfaces::msg::Frustum ufoToMsg(ufo::geometry::Frustum const& frustum);

navigation_interfaces::msg::LineSegment ufoToMsg(ufo::geometry::LineSegment const& line_segment);

navigation_interfaces::msg::Obb ufoToMsg(ufo::geometry::OBB const& Obb);

navigation_interfaces::msg::Ray ufoToMsg(ufo::geometry::Ray const& ray);

navigation_interfaces::msg::Sphere ufoToMsg(ufo::geometry::Sphere const& sphere);

navigation_interfaces::msg::BoundingVolume ufoToMsg(
    ufo::geometry::BoundingVolume const& bounding_volume);

//
// ROS message type to UFO type
//

template <typename TreeType>
bool msgToUfo(navigation_interfaces::msg::UfoMap const& msg, TreeType& tree)
{
  std::stringstream data_stream(std::ios_base::in | std::ios_base::out |
	                              std::ios_base::binary);
  if (!msg.data.empty()) {
    data_stream.write((char const*)&msg.data[0], msg.data.size());
    return tree->readData(data_stream, msgToUfo(msg.info.bounding_volume),
                     msg.info.resolution, msg.info.depth_levels,
                     msg.info.uncompressed_data_size, msg.info.compressed);
  }
  return false;
}

//
// UFO type to ROS message type
//

template <typename TreeType>
bool ufoToMsg(TreeType const& tree, navigation_interfaces::msg::UfoMap& msg, bool compress = false,
              unsigned int depth = 0, int compression_acceleration_level = 1,
              int compression_level = 0)
{
  return ufoToMsg(tree, msg, ufo::geometry::BoundingVolume(), compress, depth,
                compression_acceleration_level, compression_level);
}

template <typename TreeType, typename BoundingType>
bool ufoToMsg(TreeType const& tree, navigation_interfaces::msg::UfoMap& msg,
              BoundingType const& bounding_volume, bool compress = false,
              unsigned int depth = 0, int compression_acceleration_level = 1,
              int compression_level = 0)
{
  ufo::geometry::BoundingVolume bv;
  bv.add(bounding_volume);
  return ufoToMsg(tree, msg, bv, compress, depth, compression_acceleration_level,
	                compression_level);
}

template <typename TreeType>
bool ufoToMsg(TreeType const& tree, navigation_interfaces::msg::UfoMap& msg,
              ufo::geometry::BoundingVolume const& bounding_volume, bool compress = false,
              unsigned int depth = 0, int compression_acceleration_level = 1,
              int compression_level = 0)
{
  msg.info.version = tree.getFileVersion();
  msg.info.id = tree.getTreeType();
  msg.info.resolution = tree.getResolution();
  msg.info.depth_levels = tree.getTreeDepthLevels();
  msg.info.compressed = compress;
  msg.info.bounding_volume = ufoToMsg(bounding_volume);

  std::stringstream data_stream(std::ios_base::in | std::ios_base::out |
	                              std::ios_base::binary);
  msg.info.uncompressed_data_size =
     tree.writeData(data_stream, bounding_volume, compress, depth,
     compression_acceleration_level, compression_level);
  if (0 > msg.info.uncompressed_data_size) {
    return false;
  }

  std::string const& data_string = data_stream.str();
  msg.data = std::vector<int8_t>(data_string.begin(), data_string.end());
  return true;
}  

}  // namespace ufomap_msgs

#endif  // UFOMAP_ROS_MSGS_CONVERSIONS_H
