from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    Node(
        package='betaflight_msp_bridge',
        executable='msp_bridge',
        name='msp_bridge',
        output='screen',
        parameters=[{'port':'/dev/ttyACM0','baud':115200}],
    ),
    Node(
        package='betaflight_msp_bridge',
        executable='safety_gate',
        name='safety_gate',
        output='screen'
    ),
    # Node(
    # package='betaflight_msp_bridge',
    #     executable='keyboard_teleop',
    #     name='keyboard_teleop',
    #     output='screen'
    #     ),
    ])