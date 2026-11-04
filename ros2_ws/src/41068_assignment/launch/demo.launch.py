"""One-command launch for the interactive demo / presentation.

Starts the sim, drone, Nav2, RViz and the mission nodes (mapping mode)
through spawn_parrot.launch.py, then adds each team member's part from
launch/modules/<file>. A part whose launch file isn't there yet is skipped
with a log line instead of failing the launch, so every part can be listed
here from the start and simply turns on once its file is merged.

  ros2 launch 41068_assignment demo.launch.py
  ros2 launch 41068_assignment demo.launch.py detection:=False   # drop a part
  ros2 launch 41068_assignment demo.launch.py terrain_map:=''    # map from scratch

The terrain map starts from the world's preloaded map, maps/<world>/terrain.npz
(see maps/README.md), so trails, routes and the dashboard map work straight
away. Mapping mode finishes at once on a complete map; to show it mapping,
start from a blank map with terrain_map:=''.

See launch/modules/README.md for what a module launch file receives.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

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

    share_dir = get_package_share_directory('41068_assignment')
    launch_dir = os.path.join(share_dir, 'launch')

    # Demo defaults. Anything not declared here (gui, mission,
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
        ('nav2', 'True', 'Flag to launch Nav2. Mapping mode needs it.'),
        ('terrain_map',
         PathJoinSubstitution([os.path.join(share_dir, 'maps'), LaunchConfiguration('world'),
                               'terrain.npz']),
         "Terrain map to start from: the world's preloaded map. '' starts blank "
         '(to show Mapping mode); a missing file also starts blank, with a warning.'),
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
