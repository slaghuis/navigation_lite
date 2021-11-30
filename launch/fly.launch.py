import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','map','odom']          
    )

    map_odom_ned_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','1.57', '0', '3.14','map','odom_ned']          
    )
    
    drone_node = Node(
        package="drone",
        executable="drone_node",
        name="drone_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"connection_url": "serial:///dev/ttyAMA1:921600"},
            {"height_topic": "vl53l1x/range"},
            {"height_sensor_z_offset": 0.153}
        ]
    )
        
    odom_tf2_broadcaster = Node(
        package="drone",
        executable="odom_tf2_broadcaster",
        output="screen",
        emulate_tty=True
    )    
     
    sonar_sensor=Node(
        package = 'sonar',
        name = 'sonar_node',
        executable = 'sonar_node',
        output="screen",
        emulate_tty=True
    )
    
    lidar_sensor=Node(
        package = 'garmin_lidar',
        name = 'lidar_node',
        executable = 'lidar_node',
        output="screen",
        emulate_tty=True
    )
     
    altitude_sensor=Node(
        package = 'vl53l1x',
        name = 'vl53l1x_node',
        executable = 'vl53l1x_node',
        output="screen",
        emulate_tty=True
    ) 
           
    config_map_server = os.path.join(
        get_package_share_directory('my_robot'),
        'config',
        'sensors.yaml'
        )
                
    map_server=Node(
        package = 'navigation_lite',
        name = 'map_server',
        executable = 'map_server',
        output="screen",
        emulate_tty=True,
        parameters = [config_map_server]
    )
    
        
    controller_server=Node(
        package = 'navigation_lite',
        name = 'controller_server',
        executable = 'controller_server',
        parameters=[
            {'max_speed_xy'          : 0.7},
            {'max_speed_z'           : 0.33},
            {'max_yaw_speed'         : 0.5},
            {'waypoint_radius_error' : 0.5},
            {'yaw_threshold'         : 0.087},
            {'pid_xy'                : [0.7, 0.0, 0.0]},
            {'pid_z'                 : [0.7, 0.0, 0.0]},
            {'pid_yaw'               : [0.7, 0.0, 0.0]},
            {'holddown'              : 2},
            {'map_frame'             : "map"}
        ],
        output="screen",
        emulate_tty=True
    )
    
    navigation_server=Node(
        package = 'navigation_lite',
        name = 'navigation_server',
        executable = 'navigation_server',
        output="screen",
        emulate_tty=True
    )
    
    planner_server=Node(
        package = 'navigation_lite',
        name = 'planner_server',
        executable = 'planner_server',
        output="screen",
        emulate_tty=True
    )

    recovery_server=Node(
        package = 'navigation_lite',
        name = 'recovery_server',
        executable = 'recovery_server',
        parameters=[
            {'max_speed_xy'          : 0.7},
            {'max_speed_z'           : 0.33},
            {'max_yaw_speed'         : 0.5},
            {'waypoint_radius_error' : 0.3},
            {'yaw_threshold'         : 0.087},
            {'pid_xy'                : [0.7, 0.0, 0.0]},
            {'pid_z'                 : [0.7, 0.0, 0.0]},
            {'pid_yaw'               : [0.7, 0.0, 0.0]},
            {'holddown'              : 2}
        ],
        output="screen",
        emulate_tty=True
    )
    
    flight_control_node = Node(
        package="flight_control",
        executable="flight_control_node",
        parameters=[
            {"mission_bt_file"         : "./src/my_robot/behaviour_trees/takeoff_positive_land.xml"},
            {"navigation_bt_file"      : "./src/navigation_lite/behavior_trees/navigate.xml"},            
            {"minimum_battery_voltage" : 13.6}
        ]
    )

    
    # ld.add_action(sonar_sensor)
    # ld.add_action(altitude_sensor)
    # ld.add_action(lidar_sensor)

    ld.add_action(drone_node)
    ld.add_action(odom_tf2_broadcaster)
    ld.add_action(map_odom_tf)  
    ld.add_action(map_odom_ned_tf)
    ld.add_action(navigation_server)
    ld.add_action(recovery_server)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(map_server)
         
    ld.add_action(flight_control_node)


    return ld
