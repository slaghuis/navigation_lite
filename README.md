# Navigation Lite
A lightweight Navigation stack for the drone, with full 3D navigation.  Built on the architecture of the Navigation2 stack.  I developed this one to learn how to. Much of this code can be used to build plugins for the Navigation2 stack.  The big deviation comes in with the incorporation of the Map Server in the stack. Launch the navigation stack like so:

```
ros2 launch navigation_lite navigation_lite.launch.py
```

## Navigation Server
Main interface waiting for messages nav_lite/navigate_to_pose to fly from point A to point B.  This uses a Behviour Tree to achieve the mission.  It is assumed that all the coordinates is in the map frame, NEU orientation, with angles in radians.

## Controller Server
Will move the drone along a collection of waypoints (typically the output of the planner server), using obstacle avoidance and a local planner. Obstacle avoidance is achieved by testing the path to the next waypoint against the latest published map.  The past has to be free space.  If the path is not free space, it implies that an obstacle has moved accros the path (and was detected by a sensor).  The acton server should then stop motion, and return unsuccessfull (A number of waypoints were not reached).

## Recovery Server
Executes recovery actions.  Recivery action implimented are wait and spin.  Spin will rotate the drone at the current altitude a set arc.  Hopefully this gives the sensors time to inform the map server of any obstacles.  A wait recovery is also available.  This will maintain altitude and position for a set duration of time.  This might allow an obstacle (the dog for instance) to move along, and the sensors to detect a clear path again. Future recovery actions could inclue to change the altitude x meters.  

## Planner Server
Reads a UFO Octree Map from the Map Server and calculates a global flight plan.  Returns a sequence of waypoints for the Controller Server to follow. Uses D* Lite path planning.  Still needs to impliment replanning and services to clear the cost map.  Calculating a plan over 4 meters takes 2 (two) seconds on a Raspebrry Pi 4.  This slow performance is due to the fact that every node (one cubic meter) can have 26 (twenty six) neighbors that have to be expanded (each to their 26 neigbours.  Longer paths become exponentially slower.  A cool improvement would be a more optimistic path planning algorithm that will assuma clear path, and then execute an avoidance once an obstacle has been detected.

## Map server
Reads a list of sensors and their transforms from the parameter file and populates an octo map data structure. (UFO Map package).  The node publishes this map to the rest of the naviagation stack in a propriotary message type.

# Transforms
The Map Server reads transforms for the range sensors from the configuration file. This is the base_link->sensors transform.  The Map Server further uses a map in the map frame. The drone_mavsdk node publishes a odom->base_link transdorm. What is needed is a static transform to publish a map->odom transform.  
```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```
This then forms the complete tf2 tree to produce a map->odom->base_link->sensor transform tree.  Test it with
```
ros2 run tf2_tools view_frames.py
```
# Code Status
This code has flown on a drone!  The drone was controlled by a Pixhawk mini 4.0 controlled via a Raspberry Pi 4 4Gb companion computer via UART.  This Navigation stack used the [drone_mavsdk](https://github.com/slaghuis/drone_mavsdk) node to effect the movement.  The mission was controlled via the [flight_control](https://github.com/slaghuis/flight_control) node.
