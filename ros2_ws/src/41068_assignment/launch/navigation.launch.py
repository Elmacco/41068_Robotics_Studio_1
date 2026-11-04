"""Localisation + Nav2 for one robot.

Copy of 41068_ignition_bringup/launch/41068_navigation.launch.py, reading the
slam/nav2 params from this package's config/ instead, so the demo runs
entirely from 41068_assignment.

Localisation is odometry + GPS: the EKF (spawn_parrot.launch.py) fuses
Gazebo's odometry pose as a GPS + compass stand-in, so <robot>_odom is
already the world frame and <robot>_map -> <robot>_odom is the identity,
published here as a static transform. The map itself is still built live,
by terrain_mapper from the depth camera.

slam:=True starts SLAM Toolbox instead, which publishes its own correction
for map -> odom (the course package's setup). On top of GPS it only added
error: in simple_trees it rotated the map 40 degrees while the drone sat
still, putting everything in the map frame metres off.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def _params_filename(prefix, suffix):
    return [
        TextSubstitution(text=prefix),
        suffix,
        TextSubstitution(text='.yaml')
    ]


_TRUE_STRINGS = "['true', '1', 'yes', 'on']"


def _is_true_expression(name):
    return ["'", LaunchConfiguration(name), "'.lower() in ", _TRUE_STRINGS]


def _if_true(name):
    return IfCondition(PythonExpression(_is_true_expression(name)))


def _unless_true(name):
    return IfCondition(PythonExpression(['not ('] + _is_true_expression(name) + [')']))


def generate_launch_description():

    ld = LaunchDescription()

    config_path = PathJoinSubstitution([
        FindPackageShare('41068_assignment'),
        'config'
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )

    namespace = LaunchConfiguration('namespace')
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace, e.g. husky1 or parrot1'
    )

    config_filename_suffix = LaunchConfiguration('config_filename_suffix')
    config_filename_suffix_launch_arg = DeclareLaunchArgument(
        'config_filename_suffix',
        default_value='',
        description='Robot filename suffix, e.g. _husky1 or _parrot1'
    )

    slam_launch_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Flag to launch SLAM Toolbox for map -> odom; otherwise it is '
                    'the identity (localisation from odometry + GPS)'
    )

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='False',
        description='Flag to launch Nav2'
    )

    slam_params_file = PathJoinSubstitution([
        config_path,
        _params_filename('slam_params', config_filename_suffix)
    ])

    nav2_params_file = PathJoinSubstitution([
        config_path,
        _params_filename('nav2_params', config_filename_suffix)
    ])

    # Start Simultaneous Localisation and Mapping (SLAM).
    # Use the official slam_toolbox launch file, but wrap it in the robot
    # namespace and remap TF to relative names so it uses /<robot>/tf.
    slam = GroupAction(scoped=True, condition=_if_true('slam'), actions=[
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        # Some packages publish/subscribe absolute /map topics even when
        # started under a namespace. Force them back to relative names so
        # they become /<robot>/map and /<robot>/map_updates.
        SetRemap(src='/map', dst='map'),
        SetRemap(src='/map_updates', dst='map_updates'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file,
                # Keep slam_toolbox lifecycle handling inside its own launch
                # file. This avoids mismatches between executable/node type
                # across ROS distributions.
                'use_lifecycle_manager': 'True',
                'autostart': 'True',
            }.items()
        )
    ])

    # Without SLAM, map and odom coincide: the EKF's odom frame is already
    # anchored to the world by the GPS stand-in.
    map_to_odom = GroupAction(scoped=True, condition=_unless_true('slam'), actions=[
        PushRosNamespace(namespace),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--frame-id', [namespace, '_map'],
                       '--child-frame-id', [namespace, '_odom']],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])

    # Start Navigation Stack.
    # We push the namespace ourselves rather than relying on nav2_bringup's
    # use_namespace handling, which differs across distro/package versions.
    navigation_group = GroupAction(scoped=True, condition=_if_true('nav2'), actions=[
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        # Some packages publish/subscribe absolute /map topics even when
        # started under a namespace. Force them back to relative names so
        # they become /<robot>/map and /<robot>/map_updates.
        SetRemap(src='/map', dst='map'),
        SetRemap(src='/map_updates', dst='map_updates'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
            }.items()
        )
    ])

    navigation = navigation_group

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(namespace_launch_arg)
    ld.add_action(config_filename_suffix_launch_arg)
    ld.add_action(slam_launch_arg)
    ld.add_action(nav2_launch_arg)
    ld.add_action(slam)
    ld.add_action(map_to_odom)
    ld.add_action(navigation)

    return ld
