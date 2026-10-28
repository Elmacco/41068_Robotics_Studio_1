"""One-command launch for the interactive demo / presentation.

Starts the sim, drone, SLAM/Nav2, RViz and the mission nodes (mapping mode)
through spawn_parrot.launch.py, then adds each team member's part from
launch/modules/<file>. A part whose launch file isn't there yet is skipped
with a log line instead of failing the launch, so every part can be listed
here from the start and simply turns on once its file is merged.

  ros2 launch 41068_assignment demo.launch.py
  ros2 launch 41068_assignment demo.launch.py detection:=False   # drop a part

See launch/modules/README.md for what a module launch file receives.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

# (flag, file in launch/modules/, owner). Add a row when a new part appears.
MODULES = [
    ('dashboard', 'dashboard.launch.py', 'Connor'),
    ('thermal', 'thermal.launch.py', 'Connor'),
    ('detection', 'detection.launch.py', 'Ethan'),
    ('active_mode', 'active_mode.launch.py', 'Will'),
    ('behaviour_tree', 'behaviour_tree.launch.py', 'Alec'),
]


def _if_true(name):
    return IfCondition(PythonExpression(
        ["'", LaunchConfiguration(name), "'.lower() in ['true', '1', 'yes', 'on']"]
    ))


def generate_launch_description():

    ld = LaunchDescription()

    launch_dir = os.path.join(get_package_share_directory('41068_assignment'), 'launch')

    # Demo defaults. Anything not declared here (gui, mission, terrain_map,
    # use_sim_time, ...) can still be passed on the command line and reaches
    # spawn_parrot.launch.py as usual.
    demo_args = [
        ('world', 'simple_trees', 'World file (without .sdf) to load from <world_package>/worlds'),
        ('world_package', '41068_assignment', 'Package whose worlds/ folder holds the world'),
        # Keep x/y on the launch_pad pose in worlds/simple_trees.sdf.
        ('x', '2.0', 'Spawn x (m), the launch pad in simple_trees'),
        ('y', '0.0', 'Spawn y (m), the launch pad in simple_trees'),
        ('z', '0.8', 'Spawn height (m)'),
        ('rviz', 'True', 'Flag to launch RViz'),
        ('nav2', 'True', 'Flag to launch Nav2 (and SLAM). Mapping mode needs it.'),
    ]
    for name, default, description in demo_args:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_parrot.launch.py')),
        launch_arguments={
            **{name: LaunchConfiguration(name) for name, _, _ in demo_args},
            # The dashboard bridge is started by the dashboard module instead.
            'web_dashboard': 'False',
        }.items(),
    ))

    # Every module gets the same few arguments, so they don't need to know
    # how the rest of the demo was launched.
    module_args = {
        'namespace': 'parrot1',
        'use_sim_time': LaunchConfiguration('use_sim_time', default='True'),
        'world': LaunchConfiguration('world'),
    }.items()

    for flag, filename, owner in MODULES:
        ld.add_action(DeclareLaunchArgument(
            flag,
            default_value='True',
            description=f'Flag to start {owner}\'s part (launch/modules/{filename})',
        ))
        path = os.path.join(launch_dir, 'modules', filename)
        if os.path.exists(path):
            ld.add_action(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(path),
                launch_arguments=module_args,
                condition=_if_true(flag),
            ))
        else:
            ld.add_action(LogInfo(
                msg=f'[demo] {flag}: launch/modules/{filename} not added yet ({owner}), skipping',
                condition=_if_true(flag),
            ))

    return ld
