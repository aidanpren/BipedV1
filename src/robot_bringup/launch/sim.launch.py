import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """One-command SIM stack: Gazebo + the full robot software.

    Grouped below into SIM-ONLY (Gazebo) and SHARED (everything the real robot
    also runs). When you write real.launch.py later, the SHARED block is what
    you copy — swap only the SIM-ONLY block for the micro-ROS/Pico bringup.
    """

    # ---- SIM-ONLY: Gazebo, robot spawn, IMU bridge, ros2_control ----------
    # On the real robot the Pico provides the IMU + wheel interface instead.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_description'),
            'launch', 'gazebo.launch.py')),
    )

    # ---- SHARED: runs identically on sim and real hardware -----------------
    # balance controller (reads /imu + /joint_states, writes wheel effort)
    balance = Node(
        package='robot_base',
        executable='balance_controller',
        output='screen',
    )

    # mode supervisor (latched /mode + SetMode service + controller interlock)
    mode_manager = Node(
        package='robot_teleop',
        executable='mode_manager',
        output='screen',
    )

    # teleop input chain: joy -> teleop_twist_joy -> twist_mux -> /cmd_vel
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_teleop'),
            'launch', 'teleop.launch.py')),
    )

    # dashboard: rosbridge WebSocket (:9090) + static server (:8000) serving
    # the web page to any browser or phone on the network.
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_dashboard'),
            'launch', 'dashboard.launch.py')),
    )

    return LaunchDescription([
        gazebo,
        balance,
        mode_manager,
        teleop,
        dashboard,
    ])
