import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('navigation_lite'),
        'config',
        'sensors.yaml'
        )
        
    map_server=Node(
        package = 'navigation_lite',
        name = 'map_server',
        executable = 'map_server',
        output="screen",
        emulate_tty=True,
        parameters = [config]
    )
    
    controller_server=Node(
        package = 'navigation_lite',
        name = 'controller_server',
        executable = 'controller_server',
        output="screen",
        emulate_tty=True,
    )
    
    navigation_server=Node(
        package = 'navigation_lite',
        name = 'navigation_server',
        executable = 'navigation_server',
        output="screen",
        emulate_tty=True,
    )
    
    planner_server=Node(
        package = 'navigation_lite',
        name = 'planner_server',
        executable = 'planner_server',
        output="screen",
        emulate_tty=True,
    )

    recovery_server=Node(
        package = 'navigation_lite',
        name = 'recovery_server',
        executable = 'recovery_server',
        output="screen",
        emulate_tty=True,
    )

    ld.add_action(navigation_server)
    ld.add_action(recovery_server)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(map_server)
    
    return ld
