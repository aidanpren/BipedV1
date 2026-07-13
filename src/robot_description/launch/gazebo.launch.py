import os
import shutil

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_description')

    # gz_ros2_control's env hook does NOT set GZ_SIM_SYSTEM_PLUGIN_PATH, so
    # Gazebo can't find libgz_ros2_control-system.so on its own. Point it at
    # the ROS lib dir where the plugin lives, or no controller_manager starts.
    gz_plugin_path = AppendEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.path.join(get_package_prefix('gz_ros2_control'), 'lib'),
    )
    urdf = os.path.join(pkg, 'urdf', 'biped_sim.urdf')
    world = os.path.join(pkg, 'worlds', 'biped_world.sdf')
    controllers = os.path.join(pkg, 'config', 'controllers.yaml')

    # WORKAROUND for a ros2_control bug (gh ros2_control#2309): when the
    # controller_manager forwards its own node args to each controller it DROPS
    # any argument containing the substring "robot_description" (a filter meant
    # for remaps) but fails to also pop a preceding --params-file flag. Our
    # package is literally NAMED robot_description, so the yaml's installed
    # path trips the filter and every controller dies at load with
    # "Couldn't parse params file: '--params-file -p'". Fix: hand the plugin
    # and spawners a copy of the yaml at a path without that substring.
    controllers_neutral = '/tmp/biped_controllers.yaml'
    shutil.copyfile(controllers, controllers_neutral)
    controllers = controllers_neutral

    with open(urdf) as f:
        robot_desc = f.read()
    # Plain URDF can't do $(find ...), so bake the real controllers.yaml path in.
    robot_desc = robot_desc.replace('__CONTROLLERS_YAML__', controllers)

    # 1. Start Gazebo Harmonic with our world (-r = run immediately).
    #    Our world enables the IMU system the stock empty.sdf lacks.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + world}.items(),
    )

    # 2. robot_state_publisher: publishes the URDF on /robot_description + TF
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
    )

    # 3. Spawn the robot into Gazebo from the /robot_description topic.
    #    z=0.5 keeps the wheels clear of the ground (base sits ~0.43 m above
    #    its lowest point) so it drops cleanly instead of starting embedded.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'biped', '-z', '0.5'],
        output='screen',
    )

    # 4. Bridge gz topics -> ROS: the IMU, and /clock so every use_sim_time
    #    node (controller_manager included) gets sim time instead of warning
    #    "No clock received". '[' = one-way gz->ROS.
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # 5. Controller spawners. These ask the controller_manager (started by the
    #    gz_ros2_control plugin once the robot spawns) to load + activate each
    #    controller. They poll for the manager, so a bit of start-up race is OK.
    #    --param-file is required: without it the manager forwards an EMPTY
    #    params_file to the controller node ("--params-file -p" parse error).
    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controllers],
        output='screen',
    )
    wheel_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['wheel_effort_controller', '--param-file', controllers],
        output='screen',
    )

    return LaunchDescription([
        gz_plugin_path,  # must be set BEFORE Gazebo starts
        gz_sim, rsp, spawn, imu_bridge, jsb_spawner, wheel_spawner,
    ])
