# navigation_lite
A lightweight Navigation stack for the drone, aimed at full 3D navigation.

## Navigation Server
Main interface waiting for messages to fly frompoint A to point B.  This uses a Behviour Tree to achieve the mission.

## Commander Server
Will move the drone from one waypoint to another, using bstacle avoidance and a local planner

## Recovery Server
Executes recovery actions to assist the Commander Server.

## Planner Server
Reads an UFO Octree Map from the Map Server and calculates a global flight plan.  Returns a sequence of waypoints

## Map server
Reads a list of sensors and their transforms from the parameter file and populates OctoMap (UFO Map package).  Published this map to the rest of the naviagation stack.

# Transforms
The Map Server reads transforms from the configuration file. This is the sensor->base_link transform.  The Map Server further publishes a map.  This map needs a transform from map->odom.  (Not implimented)  The Drone node needs a odom->base_link transform.  Then I should have a map->odom->base_link->sensor transforms up and running.  If I know the GPS for the startng position I can publish a world->map->odom... 
