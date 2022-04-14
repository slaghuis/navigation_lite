// Copyright (c) 2022 Xeni Robotics
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

/****************************************************************************
 * A ROS2 Action Server that takes as an input a number of x, y coordinates
 * and based on parameters given, calculates a fligt path to cover this 
 * area.  Applicable in photogrammetry, crop spraying and search use cases.
 *
 * **************************** * WARNING NOTE * ****************************
 * This node does not use a map.  It simply calculates poses over a provided 
 * polygon (field).  The next step should be to call some path planner that
 * uses a ComputePathThroughPoses action to do path planning.  This path can 
 * then be passed to the contoller server to execute.  This node is just step 
 * one of three.  NOTE: A pose might be in a known obstucted space! 
 * 
 * According to the NAV2 definition, this is a Waypoint Following server
 ***************************************************************************/

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "navigation_lite/edge.hpp"
#include "navigation_interfaces/action/compute_coverage_poses.hpp"
#include <tf2/LinearMath/Quaternion.h>

class CoveragePosesPlannerServer : public rclcpp::Node
{
public:
  using CoveragePoses = navigation_interfaces::action::ComputeCoveragePoses;
  using GoalHandleCoveragePoses = rclcpp_action::ServerGoalHandle<CoveragePoses>;

  explicit CoveragePosesPlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("coverage_path_planning_server", options)
  {
    using namespace std::placeholders;
    
    // Declare some parameters
    this->declare_parameter<float>("overlap",0.1);  // Ten percent overlap
    this->declare_parameter<float>("minimum_height", 5.0);  // Minimum Flight Height
    this->declare_parameter<float>("maximum_height", 30.0);  // Maximum Flight Height
    
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
            
    this->action_server_ = rclcpp_action::create_server<CoveragePoses>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "nav_lite/compute_coverage_path",
      std::bind(&CoveragePosesPlannerServer::handle_goal, this, _1, _2),
      std::bind(&CoveragePosesPlannerServer::handle_cancel, this, _1),
      std::bind(&CoveragePosesPlannerServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<CoveragePoses>::SharedPtr action_server_;
  std::vector<Edge> edges;
  std::vector<Vertex> verticies;
  std::string map_frame_;
  
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CoveragePoses::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received polygon with %d verticies." , goal->area.points.size() );
    (void)uuid;
    // Let's reject inputs that are not polygons.
    if (goal->area.points.size() < 3) {
      RCLCPP_ERROR(this->get_logger(), "At least three verticies are required to define a polygon." );
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCoveragePoses> goal_handle)
  {
    RCLCPP_WARN(this->get_logger(), "Received request to cancel coverage planning goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute(const std::shared_ptr<GoalHandleCoveragePoses> goal_handle)
  {
    RCLCPP_DEBUG(this->get_logger(), "Planning a path over the provided polygon.");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<CoveragePoses::Result>();
    double projected_area_width, projected_area_height;
    
    auto start_time = this->now();
    projected_area_width  = goal->projected_area_w;
    projected_area_height = goal->projected_area_h;
    double h = goal->height;
    
    // Test the height safety parameters
    double test_h;    
    this->get_parameter("minimum_height", test_h);
    if(h < test_h) {
      h = test_h;
      RCLCPP_WARN(this->get_logger(), "Provided flight height below the safety margin.  Increasing flight hight to %5.2fm", test_h);
    } else {
      this->get_parameter("maximum_height", test_h);   
      if(h > test_h) {
        h = test_h;
        RCLCPP_WARN(this->get_logger(), "Provided flight height above the safety margin.  Decreasing flight hight to %5.2fm", test_h);
      }
    }
    
    double overlap;
    this->get_parameter("overlap", overlap);
    
    double ov_x = goal->projected_area_w * overlap;
    double ov_y = goal->projected_area_h * overlap;    
    
    for(std::size_t i = 0; i< goal->area.points.size(); i++) {
      Vertex v_point;
      
      v_point.x = goal->area.points[i].x;
      v_point.y = goal->area.points[i].y;
      
      verticies.push_back(v_point);         // Not effective duplicatin these, but it is what it is.
    }
             
    // Load the edges from the input provided
    //edges.clear();  // Clear the current set of edges, and recalculate. 
    
    for(std::size_t i = 0; i< verticies.size(); i++) {
      if (i == verticies.size()-1) {
        edges.push_back({&verticies[i], &verticies[0]});  // Close the area by linking the first and last verticies 
      } else {
        edges.push_back({&verticies[i], &verticies[i+1]});  // Link successive vericies
      }
    }         
             
    // Calculate the waypoints to fly to cover the area with the camera
    // Parameters:
    //   ox_x   Horisontal overlap (m) 
    //   ov_y   Verical Overlap (m)
    //   h      Height of flight (m)

    // * Phase 1:
    // ** 1. Find the first Vertex Vscan of the longest edge
  
    // Find the longest edge
    Edge *e_scan = &edges[0];  // Longest Edge
    for (auto& it : edges) {
      if(it.length() > e_scan->length()) {
        e_scan = &it;
      }
    }

    // Find Vfar, the Vertex with the longest distance from e_scan
    Vertex *v_far;
    double v_far_distance = 0.0;
    for (auto& it : verticies) {
      double it_dist = e_scan->distance_to(&it);
      if( it_dist > v_far_distance) {
        v_far = &it;
        v_far_distance = it_dist;
      }  
    }
    RCLCPP_DEBUG(this->get_logger(), "Found Furthest Vertex: %5.2f; %5.2f", v_far->x, v_far->y);

    // ** 2. Compute the scan direction parallel to the longest edge;
    std::pair<double, double> e_scan_formula = e_scan->line();
  
    // Find a line perpendicular to e_scan that passes through v_far 
    double m_perp = -1 / e_scan_formula.first;  // slope of perpenducular line is the negative recoplrocal 
    double c_perp = (v_far->y - m_perp * v_far->x);  // y - y_far = m(x - x_far) => c = (y-mx)
  
    // Find where the perpendicular line crosses e_scan
    Vertex intersect;  
    intersect.x = (c_perp - e_scan_formula.second) / (e_scan_formula.first - m_perp);
    intersect.y = m_perp*intersect.x + c_perp;
  
    // Define an edge starting at this intersction point, ending at v_far.  
    Edge perpendicular(&intersect, v_far);  
  
    double cumilative_perp_dist = projected_area_width/2;
    bool flip = false;
    
    do {

      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }

      // Define the first scan line, y=mx+c, parralel to e_scan, half the camera width Lx/2 from e_scan
      // First find a point on the line.
      std::pair<double, double> flight_line_point = perpendicular.point_from_start( cumilative_perp_dist );
  
      // Substitute y = mx + c, knowing x and y from flight_line_point and m from e_scan to create a parralel line.
      double m_flight_line = e_scan_formula.first;
      double c_flight_line = flight_line_point.second - m_flight_line*flight_line_point.first;
  
      // Iterate through the edges, finding intersections with the flight line
      // There can only be two edges that have intersections, otherwise the area is not valid.
      Vertex i_one;
      Vertex i_two;
      Vertex *point, *other_point;

      point = &i_one;
      for (auto& it : edges) {
        if( it.intersect(m_flight_line, c_flight_line, point) ) {
          point = &i_two;
        }    
      }  

      // Flip the direction of the line around to improve the sequence of the waypoints
      // for flight efficiencly
      if (flip) {
        point = &i_two;
        other_point = &i_one;
      } else {
        point = &i_one;
        other_point = &i_two;
      }
      flip = !flip;
      Edge scan_line(point, other_point);
    
      // ** 3.(a) Compute the distance d from Vertex v_i to v_next(i)  
      double d = scan_line.length(); // distance from Vertex v_i to v_next_i
      //std::cout << "length: " << d <<  " meters" << std::endl;
    
      // ** 3. (b) Compute n_w by Equation (19) - Number of waypoints
      int n_w = floor( (d - ov_y) / (projected_area_height - ov_y) );
      //std::cout << "Number of waypoints: " << n_w << std::endl;

      // ** 3. (c) Compute Equation (20) - c_ov_y Corrected Overlap
      // double c_ov_y = (n_w * projected_area_height - d) / (n_w - 1);
  
      // ** 3. (c) Compute Equation (21) - dw Corrected distance between waypoints
      double c_d_w = (d - projected_area_height) / (n_w - 1);
  
      // ** 3. (d) Place the first waypoint at a distance Ly/2 from the start of the line
      double cumilative_dist = projected_area_height / 2;
      std::pair<double, double> waypoint; // = scan_line.point_from_start( cumilative_dist );

      // Turn the slope of the scan line into a quaternion
      tf2::Quaternion q;
      q.setRPY( 0, 0, scan_line.inclination());
      q.normalize();
      
      do {
        waypoint = scan_line.point_from_start(cumilative_dist);
        
        geometry_msgs::msg::PoseStamped waypoint_pose;
        waypoint_pose.header.stamp = this->get_clock()->now();
        waypoint_pose.header.frame_id = map_frame_;
        
        waypoint_pose.pose.position.x = waypoint.first;
        waypoint_pose.pose.position.y = waypoint.second;
        waypoint_pose.pose.position.z = h;
                
        waypoint_pose.pose.orientation.x = (double) q.x();
        waypoint_pose.pose.orientation.y = (double) q.y();
        waypoint_pose.pose.orientation.z = (double) q.z();
        waypoint_pose.pose.orientation.w = (double) q.w();
        
        result->poses.push_back(waypoint_pose);
        
        cumilative_dist += c_d_w;
      } while (cumilative_dist <= d);
      
      cumilative_perp_dist += projected_area_width - ov_x;

      loop_rate.sleep();
      

    } while (cumilative_perp_dist < perpendicular.length());  
  
    // Check if goal is done
    if (rclcpp::ok()) {
      result->planning_time = this->now() - start_time;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCoveragePoses> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&CoveragePosesPlannerServer::execute, this, _1), goal_handle}.detach();
  }
};  // class CoveragePosesPlannerServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<CoveragePosesPlannerServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
