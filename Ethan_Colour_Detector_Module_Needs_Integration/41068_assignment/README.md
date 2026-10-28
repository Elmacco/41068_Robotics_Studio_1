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

## YOLO test: approach the person

`person1` (Rescue Randy) is at (-5, 0) in `assignment_world.sdf`. To fly the
drone up to them and face them, run the detector and the follower with the
`person_approach` route:

```bash
ros2 run 41068_assignment target_detector.py --ros-args -r __ns:=/parrot1 -p debug:=true
ros2 run 41068_assignment waypoint_follower.py --ros-args -r __ns:=/parrot1 \
  -p route:=person_approach -p yaw_goal_tolerance:=0.15
rqt_image_view /parrot1/target_detections/debug_image
```

The route stops `PERSON_STANDOFF` (4.5 m) from the person, facing them. If you
move the person in the world file, update `PERSON_XY` in `mission_waypoints.py`.
`yaw_goal_tolerance` tightens Nav2's goal yaw tolerance at runtime (the shared
config uses 1.57 rad, too loose to guarantee facing the person).

## Three people in a line: red, normal, green

`assignment_world.sdf` has three people in a line along y at x = -5, 2 m apart:
`person_red` (-5, -2), `person1` (-5, 0, the original outfit) and `person_green`
(-5, 2). The red and green ones are recoloured copies of Rescue Randy,
`rescue_randy_red` and `rescue_randy_green`, kept in this package's `models/`
folder (`launch/spawn_parrot.launch.py` adds it to Gazebo's resource path).

**One-time setup** - build the two recoloured models from the textures in
`tools/textures/`. The script copies the Rescue Randy model that Gazebo has
already downloaded into your Fuel cache (launch the world once first if you've
never seen Randy appear), swaps in the red/green texture and renames the copies:

```bash
python3 tools/make_randy_variants.py
cd ~/ros2_ws && colcon build --symlink-install --packages-select 41068_assignment
```

If it can't find the model, or can't tell which image is the colour texture, it
says so; use `--source <folder containing model.sdf>` and/or
`--replace <texture file name>`.

**The `line_inspection` route** flies the drone from its spawn point to a hold
point 7 m out from the middle of the line (at (2, 0)), facing away from the
line. It then turns on the spot in 30 degree steps, pausing 2.5 s after each,
until the camera is facing the line, and stops there (no looping).

```bash
ros2 run 41068_assignment waypoint_follower.py --ros-args -r __ns:=/parrot1 -p route:=line_inspection
```

The route sets its own pause and a tight yaw tolerance (0.10 rad) automatically,
because the shared Nav2 config's 1.57 rad tolerance would swallow the small turn
steps. `-p goal_pause_seconds:=N` and `-p yaw_goal_tolerance:=R` override them.
Tune the route with `LINE_STANDOFF`, `LINE_APPROACH_HOPS`, `LINE_TURN_STEP_DEG`
and `LINE_PAUSE_SECONDS` in `scripts/mission_waypoints.py`. If you move the
people in the world file, update `LINE_PEOPLE_XY` / `LINE_CENTER_XY` there too.

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