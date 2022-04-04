cmake_minimum_required(VERSION 3.5)
project(navigation_lite)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(BehaviorTreeV3 REQUIRED)
find_package(ufomap REQUIRED)
  
add_library(navigation_action_server SHARED
  src/navigation_server.cpp
  src/action_wait.cpp
  src/action_spin.cpp
  src/action_follow_waypoints.cpp
  src/action_compute_path_to_pose.cpp
  src/action_read_goal.cpp
  src/control_recovery_node.cpp
  src/control_pipeline_sequence.cpp
  src/control_round_robin_node.cpp
  src/decorator_rate_controller.cpp)
target_include_directories(navigation_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(navigation_action_server BT::behaviortree_cpp_v3)  
target_compile_definitions(navigation_action_server
  PRIVATE "NAVIGATION_LITE_BUILDING_DLL")
ament_target_dependencies(navigation_action_server
  "navigation_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "builtin_interfaces"
  "tf2" 
  "tf2_ros" 
  "tf2_msgs"
  "tf2_geometry_msgs")
rclcpp_components_register_node(navigation_action_server PLUGIN "navigation_lite::NavigationServer" EXECUTABLE navigation_server)

add_library(controller_action_server SHARED
  src/controller_server.cpp
  src/ufomap_ros_msgs_conversions.cpp)
target_include_directories(controller_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(controller_action_server
  PRIVATE "NAVIGATION_LITE_BUILDING_DLL")
ament_target_dependencies(controller_action_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "builtin_interfaces"
  "tf2" 
  "tf2_ros" 
  "tf2_msgs"
  "tf2_geometry_msgs"
  "navigation_interfaces"
  "drone_interfaces")
target_link_libraries(controller_action_server
    UFO::Map
)
rclcpp_components_register_node(controller_action_server PLUGIN "navigation_lite::ControllerServer" EXECUTABLE controller_server)

add_library(planner_action_server SHARED
  src/planner_server.cpp
  src/ufomap_ros_msgs_conversions.cpp
  src/d_star_lite.cpp)
target_include_directories(planner_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(planner_action_server
  PRIVATE "NAVIGATION_LITE_BUILDING_DLL")
ament_target_dependencies(planner_action_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "builtin_interfaces"
  "tf2"
  "tf2_ros"
  "tf2_msgs"
  "tf2_geometry_msgs" 
  "navigation_interfaces"
  "drone_interfaces")
target_link_libraries(planner_action_server
    UFO::Map
)   
rclcpp_components_register_node(planner_action_server PLUGIN "navigation_lite::PlannerServer" EXECUTABLE planner_server)

add_library(recovery_action_server SHARED
  src/recovery_server.cpp)
target_include_directories(recovery_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(recovery_action_server
  PRIVATE "NAVIGATION_LITE_BUILDING_DLL")
ament_target_dependencies(recovery_action_server
  "navigation_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "builtin_interfaces"
  "tf2" 
  "tf2_ros" 
  "tf2_msgs"
  "tf2_geometry_msgs"
  "drone_interfaces")
 
rclcpp_components_register_node(recovery_action_server PLUGIN "navigation_lite::RecoveryServer" EXECUTABLE recovery_server)

add_library(map_publish_server SHARED
  src/map_server.cpp
  src/ufomap_ros_conversions.cpp
  src/ufomap_ros_msgs_conversions.cpp)
target_include_directories(map_publish_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(map_publish_server
  PRIVATE "NAVIGATION_LITE_BUILDING_DLL")
ament_target_dependencies(map_publish_server
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "navigation_interfaces"
  "tf2"
  "tf2_ros"
  "tf2_msgs"
  "tf2_geometry_msgs" )
target_link_libraries(map_publish_server
    UFO::Map
) 
target_include_directories(map_publish_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_node(map_publish_server PLUGIN "navigation_lite::MapServer" EXECUTABLE map_server)

install(TARGETS
  navigation_action_server
  controller_action_server
  planner_action_server
  recovery_action_server
  map_publish_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
