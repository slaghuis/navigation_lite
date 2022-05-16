import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():

    ld = LaunchDescription()
    
    controller_server=Node(
        package = 'navigation_lite',
        name = 'controller_server',
        executable = 'controller_server',
        parameters=[
            {"max_speed_xy"          : 0.7},
            {"max_accel_xy"          : 0.2},
            {"max_speed_z"           : 0.33},
            {"max_yaw_speed"         : 0.5},
            {"waypoint_radius_error" : 0.3},
            {"yaw_threshold"         : 0.087},
            {"pid_xy"                : [0.7, 0.0, 0.0]},
            {"pid_z"                 : [0.7, 0.0, 0.0]},
            {"pid_yaw"               : [0.7, 0.0, 0.0]},
            {"holddown"              : 2}
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
        parameters=[
            {"weight"       : "map"}
        ],    
        output="screen",
        emulate_tty=True
    )
    
    recovery_server=Node(
        package = 'navigation_lite',
        name = 'recovery_server',
        executable = 'recovery_server',
        parameters=[
            {"max_speed_xy"          : 0.7},
            {"max_speed_z"           : 0.33},
            {"max_yaw_speed"         : 0.5},
            {"waypoint_radius_error" : 0.3},
            {"yaw_threshold"         : 0.087},
            {"pid_xy"                : [0.7, 0.0, 0.0]},
            {"pid_z"                 : [0.7, 0.0, 0.0]},
            {"pid_yaw"               : [0.7, 0.0, 0.0]},
            {"holddown"              : 2}
        ],
        output="screen",
        emulate_tty=True
    )
    
    ld.add_action(navigation_server)
    ld.add_action(recovery_server)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    
    return ld
