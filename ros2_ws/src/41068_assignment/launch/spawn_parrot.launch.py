from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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

    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([bringup_path, 'urdf_parrot', 'parrot.urdf.xacro']),
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
            '-y', '0.0',
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
