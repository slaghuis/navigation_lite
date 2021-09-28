# Navigation Lite
A lightweight Navigation stack for the drone, aimed at full 3D navigation.  Built of the architecture ofr the Navigation2 stack.  I developed this one to learn how to. Much of this code can be used to build plugins for the Navigation2 stack.  The big deviation comes in with the incorporation of the Map Server in the stack.  This passes a UFOMap specific message type to send the map.

## Navigation Server
Main interface waiting for messages to fly from point A to point B.  This uses a Behviour Tree to achieve the mission.  It is assumed that all the coordinates is in the map frame, NEU orientation.

## Commander Server
Will move the drone along a collection of waypoints (the output of the planner server), using obstacle avoidance and a local planner  (Still work in progress).  Will move down the route, but the local planner and obstacle avoidance is still outstanding.

## Recovery Server
Executes recovery actions to assist the Commander Server.

## Planner Server
Reads an UFO Octree Map from the Map Server and calculates a global flight plan.  Returns a sequence of waypoints for the Commander Server to follow.

## Map server
Reads a list of sensors and their transforms from the parameter file and populates OctoMap (UFO Map package).  Published this map to the rest of the naviagation stack.

# Transforms
The Map Server reads transforms for the range sensors from the configuration file. This is the base_link->sensors transform.  The Map Server further publishes a map in the map frame. The drone_mavsdk node published a map->odom and a odom->base_link transdorm. Ths then forms the complete tf2 tree.  Test it with
```
ros2 run tf2_tools view_frames.py
```
