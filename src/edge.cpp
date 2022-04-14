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

#include "coverage_planner/edge.h"

Edge::Edge(Vertex *a, Vertex *b) {
  v_start = a;
  v_end = b;
  
  // Calculate the distance between two points
  v_length = sqrt(pow(v_start->x - v_end->x, 2) +
                  pow(v_start->y - v_end->y, 2));

  // Calculate the slope and intercept of the line intersecting two verticies
  // to make up the equation y=mx+c
  v_slope = (v_end->y - v_start->y) / (v_end->x - v_start->x);  // m = rise / run
  v_intercept = v_start->y - v_slope * v_start->x;
}

double Edge::length() {
  return v_length;
}  

// returns the inclination (radians) of the edge
double Edge::inclination() {
  
  double err_x = v_end->x - v_start->x;
  double err_y = v_end->y - v_start->y;
  
  return atan2(err_y, err_x);
}  

// returns the slope and intercept of the line intersecting two verticies
// to make up the equation y=mx+c
std::pair<double, double> Edge::line() {  
  return std::make_pair(v_slope, v_intercept);
}

// returns the shorest distance from the line through the two verticies
// defining this edge to a point.
double Edge::distance_to(Vertex *a) {
   return abs(v_slope*a->x - 1*a->y + v_intercept) / sqrt(pow(v_slope,2) + pow(-1,2));
}

// returns a point on the edge distance d from the start, towards the end
// works in the principle of calculating where the line crosses a circle
// with origin at the start, and a radius of distcane.  There will be two points
// thus we need to test which one is closest to the end.
std::pair<double, double> Edge::point_from_start(double d) {
  double x, y;
  
  // First test if this one is in the right direction
  x = v_start->x + d / sqrt(1 + pow(v_slope, 2));
  y = v_slope * x + v_intercept;
  
  // Calculate Distance to end Vertex
  double rem = sqrt(pow(x - v_end->x, 2) +
                    pow(y - v_end->y, 2));
  
  // We stepped in the wrong direction
  if (rem > v_length) { 
    x = v_start->x - d / sqrt(1 + pow(v_slope, 2));
    y = v_slope * x + v_intercept;
  }
  
  return std::make_pair(x, y);
}

// If the line defined by y=mx+c crosses this edge, put the intersection point in a,
// and return true.  Else return false.
bool Edge::intersect(double m, double c, Vertex *a) {
  if(v_slope == m) {  // The lines are parallel
    return false;
  }
  
  // The lines are not parrallel, thus they must intersect
  Vertex intersect;  
  intersect.x = (v_intercept - c) / (m - v_slope);
  intersect.y = m*intersect.x + c;

  if ( on_segment(&intersect) ) {
    a->x = intersect.x;
    a->y = intersect.y;
    return true;
  }
  return false;
}  

// Given three collinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool Edge::on_segment(Vertex *a)
{
    if (a->x <= std::max(v_start->x, v_end->x) && a->x >= std::min(v_start->x, v_end->x) &&
        a->y <= std::max(v_start->y, v_end->y) && a->y >= std::min(v_start->y, v_end->y))
       return true;
 
    return false;
}
