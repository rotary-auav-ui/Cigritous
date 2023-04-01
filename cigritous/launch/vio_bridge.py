from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():

    # set /dev/ttymxc2 according to your usb address detected by FCU
    call_micrortps_agent = ExecuteProcess(
        cmd=['micrortps_agent', ' -d', ' /dev/ttymxc2', '-b', '921600'], 
        shell=True, 
        output='screen')
    
    config = os.path.join(
        get_package_share_directory('cigritous'),
        'config',
        'vio_bridge.yaml')

    node = Node(
        package='cigritous',
        executable='vio_bridge',
        name='vio_bridge',
        parameters=[config])
    
    delay_node = TimerAction(
        actions=[node], 
        period=3.0)

    return LaunchDescription([
        delay_node, 
        call_micrortps_agent])