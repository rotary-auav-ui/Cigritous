from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    
    # set /dev/ttymxc2 according to your usb address detected by FCU
    call_micrortps_agent = ExecuteProcess(
        cmd=['micrortps_agent', ' -d', ' /dev/ttymxc2', '-b', '921600'], 
        shell=True, 
        output='screen')

    node = Node(
        package='cigritous',
        executable='program_vision.py',
        name='program_vision')
    
    delay_node = TimerAction(
        actions=[node], 
        period=3.0)

    return LaunchDescription([
        delay_node, 
        call_micrortps_agent])
