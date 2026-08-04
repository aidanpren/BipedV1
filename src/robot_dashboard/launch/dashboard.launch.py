import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Dashboard: rosbridge, a static web server, and the backend node.

    Shared between sim and real hardware — the page only talks to topics and
    services, so it neither knows nor cares which is running.

    THREE processes, and it is worth knowing which does what when something
    breaks:
      * rosbridge (:9090)  the WebSocket. Page shows "reconnecting" without it.
      * http.server (:8000) serves the files. Page does not load at all.
      * dashboard_backend  git/build/restart and saving tuning to YAML. Only
                           the Deploy and Save tiles stop working; everything
                           else on the page is unaffected, because everything
                           else talks to the robot's own nodes directly.
    """
    web_dir = os.path.join(get_package_share_directory('robot_dashboard'), 'web')

    # NAME THIS 'web_port', NOT 'port'.
    # IncludeLaunchDescription passes the PARENT's launch configurations down
    # into the included description, and an included <arg>'s default only
    # applies if that configuration isn't already set. rosbridge's launch file
    # declares its own <arg name="port" default="9090"/>, so a generic 'port'
    # here silently overrode it — rosbridge tried to bind the web server's port
    # and died with "Address already in use" forever. Scope-safe names only.
    web_port_arg = DeclareLaunchArgument(
        'web_port', default_value='8000',
        description='Port the dashboard page is served on.')

    # the WebSocket bridge the page speaks to. Both args passed explicitly:
    #  - port: pinned so it can never inherit anything from a parent launch.
    #  - address 0.0.0.0: bind all interfaces so a PHONE on the network can
    #    reach it, not just localhost.
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch', 'rosbridge_websocket_launch.xml')),
        launch_arguments={'port': '9090', 'address': '0.0.0.0'}.items(),
    )

    # static server for index.html + the vendored roslib.min.js.
    # --bind 0.0.0.0 is what lets a phone load it; the default (localhost only)
    # would refuse anything but this machine.
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', LaunchConfiguration('web_port'),
             '--bind', '0.0.0.0', '--directory', web_dir],
        output='screen',
    )

    # The filesystem/shell half of the dashboard: git pull, colcon build,
    # restarting the stack, and writing a tuning session back into real.yaml.
    #
    # SCOPE-SAFE ARG NAME. 'backend', not 'enable' — see the web_port comment
    # above for what a generic name does when this file is included.
    backend_arg = DeclareLaunchArgument(
        'dashboard_backend', default_value='true',
        description='Run the deploy/save backend node. false leaves the '
                    'dashboard read-mostly: every tile still works except '
                    'Deploy and Save.')
    params_arg = DeclareLaunchArgument(
        'dashboard_params', default_value='',
        description='Optional params YAML for dashboard_backend. Empty means '
                    'use the node\'s own defaults, which auto-discover the '
                    'workspace and work unchanged on the Pi and the laptop.')

    # OpaqueFunction, because `parameters=` needs a LIST decided at description
    # time and an empty path in that list is a launch error, not an empty
    # config. A substitution cannot express "this entry is absent"; a function
    # that runs after the arguments are resolved can.
    def backend_node(context):
        params = LaunchConfiguration('dashboard_params').perform(context)
        return [Node(
            package='robot_dashboard', executable='dashboard_backend',
            output='screen',
            parameters=[params] if params else [],
            condition=IfCondition(LaunchConfiguration('dashboard_backend')),
        )]

    return LaunchDescription([web_port_arg, backend_arg, params_arg,
                              rosbridge, web_server,
                              OpaqueFunction(function=backend_node)])
