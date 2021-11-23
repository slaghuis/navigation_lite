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
 * Motion planning algorithm based on the D* Lite algorithm. 
 * D*Lite algortim by S. Koenig and M. Likhachev, 2002.
 * http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf
 *
 * Thank you to Howie Choset http://www.cs.cmu.edu/~choset for publishing his 
 * presentation and the explination slides by Ayorkor Mills-Tettey
 * https://www.cs.cmu.edu/~motionplanning/lecture/AppH-astar-dstar_howie.pdf
 * ***********************************************************************/
 
#include "navigation_lite/d_star_lite.h"

// Utility Function ////////////////////////////////////////////////////////////////////////////////////////////
bool compareNodes(shared_ptr<Node> n1, shared_ptr<Node> n2)
{
  return n1->rhsScore() > n2->rhsScore();
}

// Public Methods ///////////////////////////////////////////////////////////////////////////////////////////////////
void DStarLite::setGoal(int x, int y, int z) {
  goal.at(0) = x;
  goal.at(1) = y;
  goal.at(2) = z;
}

void DStarLite::setStart(int x, int y, int z) {
  start.at(0) = x;
  start.at(1) = y;
  start.at(2) = z;
}

void DStarLite::setTestFunction( function<bool(int, int, int)> func )
{
  testFunction = func;
}

void DStarLite::initialize()
{
  replanning = false;
  
  // declare the first node at the position of the goal
  cost_map(goal.at(0),goal.at(1),goal.at(2)) = make_shared<Node>( goal.at(0), goal.at(1), goal.at(2) );      
  cost_map(start.at(0),start.at(1),start.at(2)) = make_shared<Node>( start.at(0), start.at(1), start.at(2) );      
  
  // Set the first node rhs to 0. g = inf from birth
  cost_map(goal.at(0),goal.at(1),goal.at(2))->setRhsScore(0.0, NULL);
  
  // Put the goal on the open list because it is inconsistent
  open_list.push_back( cost_map(goal.at(0),goal.at(1),goal.at(2)) );
}

int DStarLite::computeShortestPath() {

  int count = 0;
  
  do {
    // On entry, only the start node is in the list, hence it is sorted.  Every next iteration has been sorted before the loop repeats.  
    // Pop the minimum item off the open list
    shared_ptr<Node>  minimum_node = open_list.back();
    open_list.pop_back();
    if( minimum_node->isOverConsistent() ) {
      minimum_node->setGScore(minimum_node->rhsScore());
    }                          
    
    // Expand the popped node – call UpdateVertex() on all predecessors in the graph. 
    // This computes rhs values for the predecessors and puts them on the open list if they become inconsistent.
    expand( minimum_node );    
    
    // Sort the open_list
    // sort( open_list.begin(), open_list.end(), [shared_ptr<Node>  n1, shared_ptr<Node>  n2](){ return n1->rhsScore() < n2->rhsScore(); })
    sort( open_list.begin(), open_list.end(), compareNodes);    

    count++;
    
  } while (!( (cost_map(start.at(0), start.at(1), start.at(2))->isConsistent()) &&
           (open_list.back()->rhsScore() >= cost_map(start.at(0), start.at(1), start.at(2))->rhsScore()) ) );
  
  // If the start node is consistent AND top key on the open list is not less than the key of the start node
  // So we have the optimal path and can break out of the loop.
                   
  // I can now read the shortest path and populate a response.
  return count;
}

void DStarLite::replan(int point_x, int point_y, int point_z) {
  replanning = true;
  
  shared_ptr<Node> node;
  node = cost_map(start.at(point_x), start.at(point_y), start.at(point_z));
  
  if( node == NULL) return;
  
  // Consider the outgoing edges from node.  For each of these call updateVertex on the node.
  // But the node is now occupied, thus rhs(SCORE) will be INF
  node->setRhsScore(INF, NULL);  // The node is occupied!
  
  updateVertex(node);
  if(!node->isConsistent()) {
    // The node is inconsistent.  Add it to the open list;
    bool found = false;
    for(size_t i=0; i < open_list.size(); i++) {
      found = (open_list.at(i) == node);
      if (found) { break; }  
    }
    if (!found) {
      open_list.push_back(node);
    }
  }  

  // Next, consider the incoming edges of node
  array<int, 3> point;
  point.at(0) = point_x;
  point.at(1) = point_y;
  point.at(2) = point_z;
  
  for(auto z = -1; z <= 1; z++) {
    if ((point.at(2)+z < 0) || (point.at(2)+z >= (int)dim_z)) continue;  // Map only functions for dim_z >= z >=0
    for(auto y = -1; y <= 1; y++) {
      if ((point.at(1)+y < 0) || (point.at(1)+y >= (int)dim_y)) continue;  // Map only functions for dim_y >= y >=0
      for(auto x = -1; x <= 1; x++) {
        if ((point.at(0)+x < 0) || (point.at(0)+x >= (int)dim_x)) continue;  // Map only functions for dim_x >= x >=0
        shared_ptr<Node> neighbor_node;
        if( cost_map(point.at(0)+x, point.at(1)+y, point.at(2)+z) == NULL) {
          // Instantiate a new object
          neighbor_node = std::make_shared<Node>( point.at(0)+x, point.at(1)+y, point.at(2)+z );      
          cost_map(point.at(0)+x, point.at(1)+y, point.at(2)+z) = neighbor_node;              
        } else { 
          neighbor_node = cost_map(point.at(0)+x, point.at(1)+y, point.at(2)+z);
        }
       
        updateVertex(neighbor_node);
        if (! neighbor_node->isConsistent() ) {
          // The node is inconsistent.  Add it to the open list;
          bool found = false;
          for(size_t i=0; i < open_list.size(); i++) {
          found = (open_list.at(i) == neighbor_node);
            if (found) { break; }  
          }
          if (!found) {
            open_list.push_back(neighbor_node);
          }
        }  
      }  
    }
  }            
  
}

int DStarLite::extractPath(vector<geometry_msgs::msg::PoseStamped> &waypoints) {
  shared_ptr<Node> node;
  node = cost_map(start.at(0), start.at(1), start.at(2));
  array<int, 3> point;
  bool found = false;
  int count = 0;
  do {
    
    node->getPoint(point);    
    // cout << "Path at (" << point.at(0) << " , " << point.at(1) << " , " << point.at(2) << ")" << endl;    

    geometry_msgs::msg::PoseStamped pose;
    // pose.header.stamp = this->now();
    pose.header.frame_id = "map"; 
    pose.pose.position.x = point.at(0);
    pose.pose.position.y = point.at(1);
    pose.pose.position.z = point.at(2);
  
    // Orientation is irrelevant, as the controller server will turn the drone as required.
    pose.pose.orientation.x = 0; 
    pose.pose.orientation.y = 0; 
    pose.pose.orientation.z = 0; 
    pose.pose.orientation.w = 1; ;
    
    if (count > 0) {   // Skip the start node.  We are there already!
      waypoints.push_back(pose);
    }
    
    count++;  

    found = (point.at(0) == goal.at(0)) && (point.at(1) == goal.at(1)) && (point.at(2) == goal.at(2));
    if (!found) {
      node = node->nextStep();
    }
  } while( !found );
  
  return count;
}

// Private Methods /////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DStarLite::isOccupied(int x, int y, int z) {
  return testFunction(x,y,z);
}

void DStarLite::expand(shared_ptr<Node> node) {
  // Expand the popped node – call UpdateVertex() on all predecessors in the graph.
  // KEY LEARNING - The predecessor has to be in "the graph".  i.e. It is not in the 
  // graph of another node.
  
  // Calculate the 9+8+9 neighbors and confirm they are in the cost map.
  array<int, 3> point;
  node->getPoint(point);
   
  for(auto z = -1; z <= 1; z++) {
    if ((point.at(2)+z < 0) || (point.at(2)+z >= (int)dim_z)) continue;  // Map only functions for dim_z >= z >=0
    for(auto y = -1; y <= 1; y++) {
      if ((point.at(1)+y < 0) || (point.at(1)+y >= (int)dim_y)) continue;  // Map only functions for dim_y >= y >=0
      for(auto x = -1; x <= 1; x++) {
        if ((point.at(0)+x < 0) || (point.at(0)+x >= (int)dim_x)) continue;  // Map only functions for dim_x >= x >=0
        shared_ptr<Node> neighbor_node;

        if ((x==0) && (y==0) && (z==0)) { continue; };  // Dont add the node to itself!

        if( cost_map(point.at(0)+x, point.at(1)+y, point.at(2)+z) == NULL) {
          // Instantiate a new object
          neighbor_node = std::make_shared<Node>( point.at(0)+x, point.at(1)+y, point.at(2)+z );      
          cost_map(point.at(0)+x, point.at(1)+y, point.at(2)+z) = neighbor_node;              
        } else { 
          neighbor_node = cost_map(point.at(0)+x, point.at(1)+y, point.at(2)+z);
        }
        
        if( (neighbor_node->nextStep() == NULL) && (neighbor_node != node->nextStep()) && (! isOccupied(point.at(0)+x, point.at(1)+y, point.at(2)+z) ) ){

          // For every predecessor, call UpdateVertex
          updateVertex(neighbor_node);
          if (! neighbor_node->isConsistent() ) {
            // The node is inconsistent.  Add it to the open list;
            bool found = false;
            for(size_t i=0; i < open_list.size(); i++) {
            found = (open_list.at(i) == neighbor_node);
              if (found) { break; }  
            }
            if (!found) {
              open_list.push_back(neighbor_node);
            }
          }
        } 
      }  
    }
  }            
}


void DStarLite::updateVertex(shared_ptr<Node> node)
{

  shared_ptr<Node> best_candidate = NULL;
  float best_score = INF;
  
  array<int, 3> point;
  node->getPoint(point);
  
  if ( isOccupied(point.at(0), point.at(1), point.at(2) ) ) { 
    best_score = INF;
    // possibly set the nextStep = NULL
  } else {  
    // Check every neighbor that is not NULL (had to be expanded at some stage
    // Select the neighbor with the minimum g + transition + heuristic
    // update node->rhs = neighbor.g + transition
    
    for(auto z = -1; z <= 1; z++) {
      if ((point.at(2)+z < 0) || (point.at(2)+z >= (int)dim_z)) continue;  // Map only functions for dim_z >= z >=0
      for(auto y = -1; y <= 1; y++) {
        if ((point.at(1)+y < 0) || (point.at(1)+y >= (int)dim_y)) continue;  // Map only functions for dim_y >= y >=0
        for(auto x = -1; x <= 1; x++) {
          if ((point.at(0)+x < 0) || (point.at(0)+x >= (int)dim_x)) continue;  // Map only functions for dim_x >= x >=0
          
          shared_ptr<Node> neighbor_node;

          if ((x==0) && (y==0) && (z==0)) { continue; };  // Dont add the node to itself!
          
          if( cost_map(point.at(0)+x, point.at(1)+y, point.at(2)+z) != NULL) {
            neighbor_node = cost_map(point.at(0)+x, point.at(1)+y, point.at(2)+z);
            
            if(neighbor_node->gScore() == INF) { continue; };
            
            float transition_cost = 0.0;
            
            array<int, 3> neighbor_point;
            neighbor_node->getPoint(neighbor_point);
       
            if((neighbor_point.at(0) == point.at(0)) || (neighbor_point.at(1) == point.at(1)) ) {  
               transition_cost = 1.0;  // No diagonal movement
            } else {                    
              transition_cost = 1.4;  // Diagonal movement
            }
       
            if (!(neighbor_point.at(2) == point.at(2))) {
              transition_cost += 0.4;  // Vertical movement
            }  
            
            if ((neighbor_node->gScore() + transition_cost) < best_score) {
              best_score = neighbor_node->gScore() + transition_cost;
              best_candidate = neighbor_node;
            }
               
          }
        }
      }
    }
    if(best_score < INF) {
      node->setRhsScore( best_score, best_candidate );
    }
          
  }
    
}
    
    
