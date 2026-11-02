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

    # Everything this launch uses (configs, worlds, models, URDF) lives in
    # this package. Most of it started as copies from 41068_ignition_bringup;
    # see the README for which ones were changed.
    assignment_path = FindPackageShare('41068_assignment')
    config_path = PathJoinSubstitution([assignment_path, 'config'])

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
        description='World file (without .sdf) to load from <world_package>/worlds',
    ))

    world_package = LaunchConfiguration('world_package')
    ld.add_action(DeclareLaunchArgument(
        'world_package',
        default_value='41068_assignment',
        description='Package whose worlds/ folder holds the world. Our worlds '
                    '(assignment_world, simple_trees, hiking_trail_1) are all in '
                    '41068_assignment; use 41068_ignition_bringup for the course '
                    'ones like large_demo',
    ))

    ld.add_action(DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Flag to open the Gazebo window. False runs the simulator '
                    'server only (sensors still render), which is lighter and '
                    'avoids the WSL2 rendering crash noted in the README.',
    ))

    # Spawn pose. This is also the drone's home: return_home flies back here.
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')
    ld.add_action(DeclareLaunchArgument('x', default_value='0.0', description='Spawn x (m)'))
    ld.add_action(DeclareLaunchArgument('y', default_value='-4.0', description='Spawn y (m)'))
    ld.add_action(DeclareLaunchArgument('z', default_value='0.8', description='Spawn height (m)'))

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

    ld.add_action(DeclareLaunchArgument(
        'web_dashboard',
        default_value='False',
        description='Flag to launch the local HTTP dashboard bridge for the webpage.',
    ))

    ld.add_action(DeclareLaunchArgument(
        'mission',
        default_value='True',
        description='Flag to launch the terrain mapper and mission manager '
                    '(mapping / passive / active modes). Mapping mode also '
                    'needs nav2:=True.',
    ))

    ld.add_action(DeclareLaunchArgument(
        'terrain_map',
        default_value='',
        description='terrain.npz saved by an earlier run (terrain/save service) '
                    'to start from, e.g. ~/41068_maps/hiking_trail_1_<time>/terrain.npz. '
                    'Empty starts with a blank map.',
    ))

    # Common Gazebo server systems (physics, sensors, etc.), so the world
    # files don't need to declare them.
    server_config_file = PathJoinSubstitution([config_path, 'ignition_server.config'])
    ld.add_action(SetEnvironmentVariable(name='IGN_GAZEBO_SERVER_CONFIG_PATH', value=server_config_file))
    ld.add_action(SetEnvironmentVariable(name='GZ_SIM_SERVER_CONFIG_PATH', value=server_config_file))

    randy_models_path = (
        '/home/student/subjects/41068_robotics_studio_1/'
        'Ethan_Colour_Detector_Module_Needs_Integration/41068_assignment/models'
    )
    # Let Gazebo resolve model:// URIs from all known model directories.
    # Keep assignment models first, then course models, then the extra local
    # model directory used by the upgraded world file.
    resource_path = [
        PathJoinSubstitution([assignment_path, 'models']), ':',
        PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'models']), ':',
        randy_models_path,
    ]
    ld.add_action(SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=resource_path))
    ld.add_action(SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path))

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
                PathJoinSubstitution([FindPackageShare(world_package), 'worlds', [world, '.sdf']]),
                ' -r',
                PythonExpression(["'' if '", LaunchConfiguration('gui'),
                                  "'.lower() in ['true', '1', 'yes', 'on'] else ' -s'"]),
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
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge_clock.yaml']),
            'use_sim_time': use_sim_time,
        }],
    ))

    # --- Spawn the Parrot drone, with the URDF/xacro, bridge config, and
    # localization config copied from 41068_ignition_bringup. ---
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
            PathJoinSubstitution([config_path, 'robot_localization_parrot1.yaml']),
            {
                'use_sim_time': use_sim_time,
                # Override bringup's odom0_config (velocities only) to also
                # fuse the absolute x, y and yaw that Gazebo's odometry
                # reports, standing in for GPS + compass.
                #
                # With velocities only, the EKF integrated two bogus readings
                # Gazebo sends right after spawning (about -400 and -67 m/s,
                # as if the drone jumped from the world origin to its spawn
                # point), so parrot1_odom - and SLAM's parrot1_map, which
                # starts aligned with it - ended up 11.3 m away from the
                # world. Fusing the pose keeps parrot1_map aligned with the
                # world, so world-file coordinates (bounds, trees, trails)
                # can be used directly as map coordinates.
                #
                # z is deliberately left out, so TF stays 2D like the rest of
                # the course stack. SLAM Toolbox is 2D and sets map -> odom so
                # that base_link sits at z = 0 in the map; with z fused, map
                # -> odom carried -altitude, refreshed only once per processed
                # scan, and Nav2 ignored lidar hits above 2 m in odom (its
                # max_obstacle_height). Nodes that need the altitude read it
                # from the 3D Gazebo odometry topic instead.
                #
                # Order: x y z, roll pitch yaw, vx vy vz, vroll vpitch vyaw,
                # ax ay az.
                'odom0_config': [True, True, False, False, False, True,
                                 True, True, False, False, False, True,
                                 False, False, False],
            },
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
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
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
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge_parrot1.yaml']),
            'use_sim_time': use_sim_time,
        }],
        # Only the flight controller talks to Gazebo; everything else
        # (Nav2, the dashboard, mission modes) publishes on cmd_vel, which
        # the flight controller passes on with altitude hold added.
        remappings=[('cmd_vel', 'cmd_vel_gz')],
    ))

    ld.add_action(Node(
        package='41068_assignment',
        executable='flight_controller.py',
        namespace=namespace,
        name='flight_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_altitude': ParameterValue(spawn_z, value_type=float),
        }],
    ))

    # Terrain map from the depth camera (always running, so every flight adds
    # to it), trail detection and the ranger route service. Mapping bounds and
    # the ranger station come from config/worlds/<world>.yaml.
    ld.add_action(Node(
        package='41068_assignment',
        executable='terrain_mapper.py',
        namespace=namespace,
        name='terrain_mapper',
        output='screen',
        parameters=[
            PathJoinSubstitution([assignment_path, 'config', 'worlds', [world, '.yaml']]),
            {'use_sim_time': use_sim_time, 'world_name': world,
             'map_load_file': LaunchConfiguration('terrain_map')},
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=_if_true('mission'),
    ))

    # Mission modes, driven by mission_command (from the dashboard or the
    # command line, see the README). Home is the spawn pose.
    ld.add_action(Node(
        package='41068_assignment',
        executable='mission_manager.py',
        namespace=namespace,
        name='mission_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'home_x': ParameterValue(spawn_x, value_type=float),
            'home_y': ParameterValue(spawn_y, value_type=float),
            'home_z': ParameterValue(spawn_z, value_type=float),
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=_if_true('mission'),
    ))

    # Second, separate bridge node just for the thermal camera topic added
    # in parrot_thermal.urdf.xacro. Kept out of the bridge node above (which
    # uses an unmodified copy of bringup's gazebo_bridge_parrot1.yaml) so that
    # config stays a faithful copy of the course-provided one.
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

    # SLAM/Nav2, using our copy of bringup's navigation launch file with the
    # slam_params_parrot1.yaml / nav2_params_parrot1.yaml configs in config/.
    ld.add_action(TimerAction(
        period=7.0,
        actions=[GroupAction(
            scoped=True,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([assignment_path, 'launch', 'navigation.launch.py'])
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

    ld.add_action(TimerAction(
        period=4.0,
        actions=[Node(
            package='41068_assignment',
            executable='web_dashboard_bridge.py',
            namespace=namespace,
            name='web_dashboard_bridge',
            output='screen',
        )],
        condition=_if_true('web_dashboard'),
    ))

    # RViz, with the pre-built Parrot display config copied from
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
            arguments=['-d', PathJoinSubstitution([config_path, '41068_parrot1.rviz'])],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        )],
        condition=_if_true('rviz'),
    ))

    return ld
