"""Dashboard part of the demo (Connor): the HTTP bridge feeding the GUI page.

Included by demo.launch.py, see README.md in this folder.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    namespace = LaunchConfiguration('namespace')
    ld.add_action(DeclareLaunchArgument('namespace', default_value='parrot1'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('world', default_value=''))

    ld.add_action(TimerAction(
        period=4.0,
        actions=[Node(
            package='41068_assignment',
            executable='web_dashboard_bridge.py',
            namespace=namespace,
            name='web_dashboard_bridge',
            output='screen',
        )],
    ))

    return ld
