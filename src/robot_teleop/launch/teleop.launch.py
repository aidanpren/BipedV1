import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{
                'device_name': 'Sony Interactive Entertainment Wireless Controller',
                'autorepeat_rate': 20.0,
            }],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            parameters=[
                os.path.join(
                    get_package_share_directory('robot_teleop'),
                    'config',
                    'teleop_twist_joy.yaml'
                )
            ],
            remappings=[('cmd_vel', 'joy_vel')]
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[
                os.path.join(
                    get_package_share_directory('robot_teleop'),
                    'config',
                    'twist_mux.yaml')
            ],
            remappings=[('cmd_vel_out', 'cmd_vel')]
            
        )
    ])