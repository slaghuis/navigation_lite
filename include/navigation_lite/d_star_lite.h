#ifndef D_STAR_LITE_H
#define D_STAR_LITE_H

#include <iostream>
#include <string>
#include <memory>
#include <limits>       // std::numeric_limits
#include <algorithm>    // std::sort
#include <vector>       // std::vector
#include <array>        // std::array
#include <functional>   // std::function
#include <cmath>        // std::sqrt

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "cube.h"

const float INF = std::numeric_limits<float>::max();

using namespace std;

class Node
{
  public:
    Node(int x, int y, int z) 
      : g(INF)
      , rhs(INF)
      , rhs_from(NULL)
    {
      point.at(0) = x;
      point.at(1) = y;
      point.at(2) = z;   
    }
    
    bool isOverConsistent() 
    {
      return (g > rhs);
    }
  
    bool isConsistent() 
    {
      return (g == rhs);
    } 

    shared_ptr<Node> nextStep()
    {
      return rhs_from;
    }
        
    void setRhsScore( float score, shared_ptr<Node> caller )
    {
      rhs = score;
      rhs_from = caller;
    }  

    void setGScore( float score )
    {
      g = score;
    }  

    float gScore() {
      return g;
    }
    
    float rhsScore() {
      return rhs;
    }
    
    void getPoint(array<int, 3> &p) {
      for(size_t i=0; i < p.size(); i++) {
        p.at(i) = point.at(i);
      } 
    }  
    
  private:
    array<int, 3> point;
    float g, rhs;
    shared_ptr<Node> rhs_from;
};

class DStarLite {
  public:
    DStarLite(size_t x, size_t y, size_t z ) 
      : dim_x(x)
      , dim_y(y)
      , dim_z(z)
      , cost_map(x, y, z, NULL)
    { }
  
    void setGoal(int x, int y, int z);
    void setStart(int x, int y, int z);
    void setTestFunction( function<bool(int, int, int)> func);
    void initialize();
    int computeShortestPath();
    void clearCostmap();
    void replan(int x, int y, int z);
    void updateVertex(shared_ptr<Node> node);
    int extractPath(vector<geometry_msgs::msg::PoseStamped> &waypoints); 
  private:
    function<bool(int, int, int)> testFunction;
    size_t dim_x, dim_y, dim_z;
    Cube< shared_ptr<Node> > cost_map;
  
    vector< shared_ptr<Node> > open_list;
    array<int, 3> goal;
    array<int, 3> start;

    void expand(shared_ptr<Node> node);    
    bool isOccupied(int x, int y, int z);
};

#endif     //D_STAR_LITE_H
