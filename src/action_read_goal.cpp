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

// Action server to Navigation Server
// Read the goal from the node, and write it to the blackboard <ReadGoal pose="{target_pose}" />

#include "navigation_lite/action_read_goal.h"

namespace NavigationNodes
{
 
BT::NodeStatus NavLiteReadGoalAction::tick()
{   
  setOutput("pose", goal_ );
  return BT::NodeStatus::SUCCESS;
    
}
  
}  // namespace