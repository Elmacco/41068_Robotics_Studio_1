"""Detection part of the demo (Ethan): people in the RGB camera, flagged red
or not, and their map position from the depth camera.

  target_detector.py   camera/image -> target_detections, target_colour
  target_projector.py  target_detections + depth + thermal_camera/image ->
                       points_of_interest (the people list Active mode uses),
                       target_pose, target_marker

Also bridges the RGB-D camera_info topics, which the projector needs and the
main bridge config doesn't include.

target_detector.py needs the "ultralytics" pip package. Without it this part
is skipped with a log line rather than crashing the demo:
  pip install -r requirements.txt

Included by demo.launch.py, see README.md in this folder.
"""

import importlib.util

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    colour_enabled = LaunchConfiguration('colour_enabled')
    ld.add_action(DeclareLaunchArgument('namespace', default_value='parrot1'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('world', default_value=''))
    ld.add_action(DeclareLaunchArgument('colour_enabled', default_value='True'))

    if importlib.util.find_spec('ultralytics') is None:
        ld.add_action(LogInfo(
            msg='[demo] detection: the ultralytics pip package is not installed, skipping. '
                'Install it with: pip install -r src/41068_assignment/requirements.txt',
        ))
        return ld

    assignment_path = FindPackageShare('41068_assignment')

    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='gazebo_bridge_camera_info',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution(
                [assignment_path, 'config', 'gazebo_bridge_parrot1_camera_info.yaml']
            ),
            'use_sim_time': use_sim_time,
        }],
    ))

    ld.add_action(Node(
        package='41068_assignment',
        executable='target_detector.py',
        namespace=namespace,
        name='target_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'colour_enabled': colour_enabled,
            # Bundled copy, so it doesn't need to download on first run.
            'model': PathJoinSubstitution([assignment_path, 'weights', 'yolo26n.pt']),
            # Annotated frames on target_detections/debug_image for the demo.
            'debug': True,
        }],
    ))

    ld.add_action(Node(
        package='41068_assignment',
        executable='target_projector.py',
        namespace=namespace,
        name='target_projector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_name': namespace,
        }],
        # Gazebo's rgbd_camera publishes one camera_info (camera/camera_info)
        # for both images, which share a resolution and FOV; nothing is ever
        # published on camera/depth/camera_info.
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('camera/depth/camera_info', 'camera/camera_info'),
        ],
    ))

    return ld
