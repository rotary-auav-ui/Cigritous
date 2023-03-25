from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('cigritous'),
        'config',
        'vio_bridge.yaml'
        )

    node = Node(
        package='cigritous',
        executable='vio_bridge',
        name='vio_bridge',
        parameters=[config])

    return LaunchDescription([node])