from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    vio = Node(
        package='cigritous',
        executable='vio_bridge',
        name='vio_bridge')

    return LaunchDescription([vio])
