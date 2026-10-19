from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _if_true(name):
    return IfCondition(PythonExpression(
        ["'", LaunchConfiguration(name), "'.lower() in ['true', '1', 'yes', 'on']"]
    ))


def generate_launch_description():

    ld = LaunchDescription()

    bringup_path = FindPackageShare('41068_ignition_bringup')
    bringup_config_path = PathJoinSubstitution([bringup_path, 'config'])
    assignment_path = FindPackageShare('41068_assignment')

    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time',
    ))

    world = LaunchConfiguration('world')
    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value='assignment_world',
        description='World file (without .sdf) to load from 41068_assignment/worlds',
    ))

    rviz = LaunchConfiguration('rviz')
    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz',
    ))

    ld.add_action(DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Flag to launch SLAM Toolbox',
    ))

    ld.add_action(DeclareLaunchArgument(
        'nav2',
        default_value='False',
        description='Flag to launch Nav2. Nav2 also starts SLAM.',
    ))

    # Reuse the common Gazebo server systems (physics, sensors, etc.) curated
    # in 41068_ignition_bringup so this world doesn't need to redeclare them.
    server_config_file = PathJoinSubstitution([bringup_config_path, 'ignition_server.config'])
    ld.add_action(SetEnvironmentVariable(name='IGN_GAZEBO_SERVER_CONFIG_PATH', value=server_config_file))
    ld.add_action(SetEnvironmentVariable(name='GZ_SIM_SERVER_CONFIG_PATH', value=server_config_file))

    # Let Gazebo resolve model:// URIs (e.g. grass_plane) from the bringup
    # package's models/ directory, since our world file lives in this package.
    ld.add_action(SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=bringup_path))
    ld.add_action(SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=bringup_path))

    # Start Gazebo with our world.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py',
            ])
        ),
        launch_arguments={
            'ign_args': [
                PathJoinSubstitution([assignment_path, 'worlds', [world, '.sdf']]),
                ' -r',
            ]
        }.items(),
    ))

    # Bridge the global simulation clock.
    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_clock',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([bringup_config_path, 'gazebo_bridge_clock.yaml']),
            'use_sim_time': use_sim_time,
        }],
    ))

    # --- Spawn the Parrot drone, reusing its URDF/xacro, bridge config, and
    # localization config from 41068_ignition_bringup. ---
    namespace = 'parrot1'
    frame_prefix = 'parrot1_'
    gz_model_name = 'parrot1'

    # Use our own parrot_thermal.urdf.xacro (adds a thermal camera on top of
    # the stock Parrot from 41068_ignition_bringup) instead of bringup's
    # plain parrot.urdf.xacro. See that file for why it's not just added to
    # the shared bringup xacro directly.
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([assignment_path, 'urdf_parrot', 'parrot_thermal.urdf.xacro']),
            ' ',
            'prefix:=', frame_prefix,
            ' ',
            'gz_model_name:=', gz_model_name,
        ]),
        value_type=str,
    )

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    ))

    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=namespace,
        name='robot_localization',
        output='screen',
        parameters=[
            PathJoinSubstitution([bringup_config_path, 'robot_localization_parrot1.yaml']),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odometry/filtered', 'odom'),
        ],
    ))

    spawn_parrot = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_' + namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-topic', '/' + namespace + '/robot_description',
            '-name', gz_model_name,
            '-x', '0.0',
            '-y', '-4.0',
            '-z', '0.8',
        ],
    )
    # Give Gazebo a moment to come up before spawning the entity.
    ld.add_action(TimerAction(period=3.0, actions=[spawn_parrot]))

    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='gazebo_bridge',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([bringup_config_path, 'gazebo_bridge_parrot1.yaml']),
            'use_sim_time': use_sim_time,
        }],
    ))

    # Second, separate bridge node just for the thermal camera topic added
    # in parrot_thermal.urdf.xacro. Kept out of the bridge node above (which
    # uses bringup's unmodified gazebo_bridge_parrot1.yaml) so that config
    # stays a faithful copy of the course-provided one.
    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='gazebo_bridge_thermal',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution(
                [assignment_path, 'config', 'gazebo_bridge_parrot1_thermal.yaml']
            ),
            'use_sim_time': use_sim_time,
        }],
    ))

    # SLAM/Nav2, reusing the same navigation launch file, namespace, and
    # slam_params_parrot1.yaml / nav2_params_parrot1.yaml configs as the main
    # bringup package.
    ld.add_action(TimerAction(
        period=7.0,
        actions=[GroupAction(
            scoped=True,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_path, 'launch', '41068_navigation.launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'namespace': namespace,
                    'config_filename_suffix': '_' + namespace,
                    'slam': LaunchConfiguration('slam'),
                    'nav2': LaunchConfiguration('nav2'),
                }.items(),
            )],
        )],
    ))

    # RViz, reusing the pre-built Parrot display config from
    # 41068_ignition_bringup (Lidar, RobotModel, Camera, Depth Pointcloud,
    # Map, Nav2 panels). Placed in the robot namespace so relative topics
    # (tf, scan, camera/image, ...) resolve without extra remapping.
    ld.add_action(TimerAction(
        period=5.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            namespace=namespace,
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', PathJoinSubstitution([bringup_config_path, '41068_parrot1.rviz'])],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        )],
        condition=_if_true('rviz'),
    ))

    return ld
