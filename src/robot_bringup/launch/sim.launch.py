import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """One-command SIM stack: Gazebo + the full robot software.

    Grouped below into SIM-ONLY (Gazebo) and SHARED (everything the real robot
    also runs). real.launch.py runs the same SHARED block — only the layer
    underneath it changes (Gazebo here, ODrive/IMU hardware there).

    Tunables live in config/sim.yaml, NOT in the node source. real.launch.py
    loads config/real.yaml instead, so both tunings exist at once and
    `diff sim.yaml real.yaml` shows how the real robot differs from the model.
    """
    params = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'sim.yaml')

    # ---- SIM-ONLY: Gazebo, robot spawn, IMU bridge, ros2_control ----------
    # On the real robot odrive_bridge + imu_node provide these interfaces.
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
        parameters=[params],
    )

    # mode supervisor (latched /mode + SetMode service + controller interlock
    # + the DS4 mode buttons, whose indices come from sim.yaml)
    mode_manager = Node(
        package='robot_teleop',
        executable='mode_manager',
        output='screen',
        parameters=[params],
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
