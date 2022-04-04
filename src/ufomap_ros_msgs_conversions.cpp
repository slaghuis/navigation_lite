#include "navigation_lite/ufomap_ros_msgs_conversions.h"

namespace ufomap_msgs
{
ufo::geometry::Point msgToUfo(navigation_interfaces::msg::Point const& point)
{
	return ufo::geometry::Point(point.x, point.y, point.z);
}

ufo::geometry::AABB msgToUfo(navigation_interfaces::msg::Aabb const& aabb)
{
	ufo::geometry::AABB a;
	a.center = msgToUfo(aabb.center);
	a.half_size = msgToUfo(aabb.half_size);
	return a;
}

ufo::geometry::Plane msgToUfo(navigation_interfaces::msg::Plane const& plane)
{
	return ufo::geometry::Plane(msgToUfo(plane.normal), plane.distance);
}

ufo::geometry::Frustum msgToUfo(navigation_interfaces::msg::Frustum const& frustum)
{
	ufo::geometry::Frustum f;
	for (size_t i = 0; i < frustum.planes.size(); ++i) {
		f.planes[i] = msgToUfo(frustum.planes[i]);
	}
	return f;
}

ufo::geometry::LineSegment msgToUfo(navigation_interfaces::msg::LineSegment const& line_segment)
{
	return ufo::geometry::LineSegment(msgToUfo(line_segment.start),
	                                  msgToUfo(line_segment.end));
}

ufo::geometry::OBB msgToUfo(navigation_interfaces::msg::Obb const& obb)
{
	return ufo::geometry::OBB(msgToUfo(obb.center), msgToUfo(obb.half_size),
	                          msgToUfo(obb.rotation));
}

ufo::geometry::Ray msgToUfo(navigation_interfaces::msg::Ray const& ray)
{
	return ufo::geometry::Ray(msgToUfo(ray.origin), msgToUfo(ray.direction));
}

ufo::geometry::Sphere msgToUfo(navigation_interfaces::msg::Sphere const& sphere)
{
	return ufo::geometry::Sphere(msgToUfo(sphere.center), sphere.radius);
}

ufo::geometry::BoundingVolume msgToUfo(navigation_interfaces::msg::BoundingVolume const& msg)
{
	ufo::geometry::BoundingVolume bv;
	for (navigation_interfaces::msg::Aabb const& aabb : msg.aabbs) {
		bv.add(msgToUfo(aabb));
	}
	for (navigation_interfaces::msg::Frustum const& frustum : msg.frustums) {
		bv.add(msgToUfo(frustum));
	}
	for (navigation_interfaces::msg::LineSegment const& line_segment : msg.line_segments) {
		bv.add(msgToUfo(line_segment));
	}
	for (navigation_interfaces::msg::Obb const& obb : msg.obbs) {
		bv.add(msgToUfo(obb));
	}
	for (navigation_interfaces::msg::Plane const& plane : msg.planes) {
		bv.add(msgToUfo(plane));
	}
	for (navigation_interfaces::msg::Point const& point : msg.points) {
		bv.add(msgToUfo(point));
	}
	for (navigation_interfaces::msg::Ray const& ray : msg.rays) {
		bv.add(msgToUfo(ray));
	}
	for (navigation_interfaces::msg::Sphere const& sphere : msg.spheres) {
		bv.add(msgToUfo(sphere));
	}
	return bv;
}

//
// UFOMap type to ROS message type
//

navigation_interfaces::msg::Point ufoToMsg(ufo::geometry::Point const& point)
{
	navigation_interfaces::msg::Point msg;
	msg.x = point.x();
	msg.y = point.y();
	msg.z = point.z();
	return msg;
}

navigation_interfaces::msg::Aabb ufoToMsg(ufo::geometry::AABB const& aabb)
{
	navigation_interfaces::msg::Aabb msg;
	msg.center = ufoToMsg(aabb.center);
	msg.half_size = ufoToMsg(aabb.half_size);
	return msg;
}

navigation_interfaces::msg::Plane ufoToMsg(ufo::geometry::Plane const& plane)
{
	navigation_interfaces::msg::Plane msg;
	msg.normal = ufoToMsg(plane.normal);
	msg.distance = plane.distance;
	return msg;
}

navigation_interfaces::msg::Frustum ufoToMsg(ufo::geometry::Frustum const& frustum)
{
	navigation_interfaces::msg::Frustum msg;
	for (size_t i = 0; i < msg.planes.size(); ++i) {
		msg.planes[i] = ufoToMsg(frustum.planes[i]);
	}
	return msg;
}

navigation_interfaces::msg::LineSegment ufoToMsg(ufo::geometry::LineSegment const& line_segment)
{
	navigation_interfaces::msg::LineSegment msg;
	msg.start = ufoToMsg(line_segment.start);
	msg.end = ufoToMsg(line_segment.end);
	return msg;
}

navigation_interfaces::msg::Obb ufoToMsg(ufo::geometry::OBB const& obb)
{
	navigation_interfaces::msg::Obb msg;
	msg.center = ufoToMsg(obb.center);
	msg.half_size = ufoToMsg(obb.half_size);
	// TODO: Fix
	// msg.rotation = ufoToMsg(obb.rotation);
	return msg;
}

navigation_interfaces::msg::Ray ufoToMsg(ufo::geometry::Ray const& ray)
{
	navigation_interfaces::msg::Ray msg;
	msg.origin = ufoToMsg(ray.origin);
	msg.direction = ufoToMsg(ray.direction);
	return msg;
}

navigation_interfaces::msg::Sphere ufoToMsg(ufo::geometry::Sphere const& sphere)
{
	navigation_interfaces::msg::Sphere msg;
	msg.center = ufoToMsg(sphere.center);
	msg.radius = sphere.radius;
	return msg;
}

navigation_interfaces::msg::BoundingVolume ufoToMsg(ufo::geometry::BoundingVolume const& bounding_volume)
{
	navigation_interfaces::msg::BoundingVolume msg;
	for (ufo::geometry::BoundingVar const& bv : bounding_volume) {
		std::visit(
		    [&msg](auto&& arg) -> void {
			    using T = std::decay_t<decltype(arg)>;
			    if constexpr (std::is_same_v<T, ufo::geometry::AABB>) {
				    msg.aabbs.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::Frustum>) {
				    msg.frustums.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::LineSegment>) {
				    msg.line_segments.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::OBB>) {
				    msg.obbs.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::Plane>) {
				    msg.planes.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::math::Vector3>) {
				    msg.points.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::Ray>) {
				    msg.rays.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::Sphere>) {
				    msg.spheres.push_back(ufoToMsg(arg));
			    }
		    },
		    bv);
	}
	return msg;
}
}  // namespace ufomap_msgs