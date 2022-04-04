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
        
    node=Node(
        package = 'navigation_lite',
        name = 'map_server',
        executable = 'map_server',
        output="screen",
        emulate_tty=True,
        parameters=[
            {'map_frame'          : 'map'},
            {'map_topic'          : 'nav_lite/map'},
            {'pointcloud_topic'   : 'sensors/pointcloud'}
        ]
    )
    ld.add_action(node)
    return ld
