import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Dashboard: the rosbridge WebSocket + a static server for the web page.

    Shared between sim and real hardware — the page only talks to topics and
    services, so it neither knows nor cares which is running.
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

    return LaunchDescription([web_port_arg, rosbridge, web_server])
