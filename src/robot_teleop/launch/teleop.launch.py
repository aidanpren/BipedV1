import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Whether the leg-height stick is live. Defaults TRUE because sim legs work
    # and are harmless; real.launch.py passes 'false' explicitly.
    # See the note on leg_joy at the bottom of this file.
    legs = DeclareLaunchArgument(
        'legs', default_value='true',
        description='Publish /leg_position_cmd from the right stick. '
                    'false on the real robot until leg bring-up is done.')

    return LaunchDescription([
        legs,
        Node(
            package='joy',
            executable='joy_node',
            # output='screen' is NOT cosmetic: joy_node reports "Couldn't open
            # joystick" when no pad is connected, and the launch default of
            # output='log' hides it in a file. The symptom is then a silent
            # /joy_vel with no error anywhere you are looking. Cost an hour on
            # 2026-08-01.
            output='screen',
            parameters=[{
                # NO device_name: joy's name-matching is broken here (fails even
                # against the exact joy_enumerate_devices string). Default
                # device_id 0 is unambiguous — SDL only enumerates the pad on
                # this machine (the js0 accelerometer isn't an SDL device).
                'autorepeat_rate': 20.0,
                'deadzone': 0.12
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
            output='screen',
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
            output='screen',
            remappings=[('cmd_vel_out', 'cmd_vel')]
        ),
        # right-stick -> leg height. Publishes /leg_position_cmd (turns), the
        # same topic the dashboard slider and the real leg_controller use.
        #
        # CONDITIONAL since 2026-08-02. With legs:=false this node is not
        # launched at all, so the stick cannot produce a leg command even by
        # accident. That is a stronger guarantee than "leg_controller is not
        # running so nothing is listening": it stays true the moment
        # leg_controller DOES come back, which is exactly when a stray
        # setpoint would matter most.
        Node(
            package='robot_teleop',
            executable='leg_joy',
            output='screen',
            condition=IfCondition(LaunchConfiguration('legs')),
        )
    ])