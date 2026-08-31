# 41068 Assignment Package

Proof-of-concept package for the assignment: spawn the Parrot drone in a
custom world, run SLAM + Nav2 for it, and fly it through a fixed route of
waypoints while streaming sensor data (camera, lidar, IMU).

## Layout

```
launch/
  spawn_parrot.launch.py   Spawns the drone + SLAM/Nav2/RViz in assignment_world
worlds/
  assignment_world.sdf     Ground plane + a single tree near the flight path
scripts/
  mission_waypoints.py     Shared waypoint route + RViz marker builder
  publish_waypoints.py     Preview the route in RViz (no Nav2 calls)
  waypoint_follower.py     Fly the route via Nav2's NavigateToPose action
```

## Prerequisites

Built on top of `41068_ignition_bringup` (URDF, sensor bridge configs,
localization configs are all reused from there). Also needs:

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-bringup
```

Build:

```bash
cd ros2_ws
colcon build --packages-select 41068_assignment
source install/setup.bash
```

## Running it

1. Launch the sim with SLAM + Nav2:

   ```bash
   ros2 launch 41068_assignment spawn_parrot.launch.py rviz:=True nav2:=True
   ```

   (`nav2:=True` starts SLAM Toolbox automatically as well.)

2. (Optional) Preview the planned route in RViz before flying it — this only
   publishes markers, it never sends the drone anywhere:

   ```bash
   ros2 run 41068_assignment publish_waypoints.py --ros-args -r __ns:=/parrot1
   ```

   In RViz, add a **MarkerArray** display on the `/parrot1/waypoint_markers`
   topic to see the route: numbered spheres at each waypoint, colour-coded
   grey (visited) / green (current target) / orange (upcoming).

3. Fly the route:

   ```bash
   ros2 run 41068_assignment waypoint_follower.py --ros-args -r __ns:=/parrot1
   ```

   Stop the preview node first (or just leave it running — both publish to
   the same topic, so the live mission's colours will simply take over).

## Changing the route

Edit `DEFAULT_WAYPOINTS` in `scripts/mission_waypoints.py`. Each waypoint is
`(x, y, yaw_degrees)` in the drone's `parrot1_map` frame. Both
`publish_waypoints.py` and `waypoint_follower.py` read from this same list,
so there's only one place to change it. The route can also be overridden per
run via the `waypoints` ROS parameter (a flat list of the same triples).

By default the route does not loop once it reaches the last waypoint; pass
`--ros-args -p loop:=true` to `waypoint_follower.py` to repeat it.

## Notes on the Nav2/SLAM setup

- `nav2_params_parrot1.yaml` and `slam_params_parrot1.yaml` (in
  `41068_ignition_bringup/config/`) are shared with the main bringup launch,
  but the Nav2 tolerances and costmap inflation radius have been tuned down
  for the drone's small footprint and this world's short waypoint spacing.
- The global costmap uses a rolling window (`rolling_window: true`, 20x20m)
  rather than relying solely on the SLAM-built static map's extent. Without
  this, path planning can fail for any goal or start pose that falls outside
  whatever area SLAM has explored so far.
- If Gazebo crashes on startup with an `Ogre::UnimplementedException`, this
  is a known WSL2 + Mesa D3D12 rendering limitation, not a bug in this
  package. Work around it with:

  ```bash
  export LIBGL_ALWAYS_SOFTWARE=1
  ```
