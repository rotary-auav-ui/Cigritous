from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    node = Node(
        package='cigritous',
        executable='program_vision.py',
        name='program_vision')

    return LaunchDescription([node])
