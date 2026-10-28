# 41068 Assignment Package

Spawns the Parrot drone in a world, runs SLAM + Nav2 for it, and flies it in
one of three mission modes:

- **Mapping**: explores the world on its own until the camera has seen all of
  it, building a terrain map (ground height, obstacles, trails), then saves
  the map and flies home.
- **Passive**: flies along a trail the ranger picks, at a fixed altitude above
  the trees.
- **Active**: drops lower over the current spot, then reports when a
  person-detection message is confirmed by the thermal camera.

From the terrain map it can also plan the least-effort walking route for a
ranger to any point, e.g. to a person who needs help. The older fixed
waypoint route (`waypoint_follower.py`) still works too.

## Layout

```
GUI/
  index.html, app.js, style.css   Static browser dashboard for the drone UI
launch/
  spawn_parrot.launch.py   Spawns the drone + SLAM/Nav2/RViz + mission nodes
  demo.launch.py           Presentation launch: spawn_parrot + each team part
  modules/                 One launch file per team part (see its README)
config/worlds/
  <world>.yaml             Per-world mapping bounds and ranger station
worlds/
  assignment_world.sdf     Ground plane + a few trees
urdf_parrot/               Parrot + thermal camera (copies of bringup's xacros
                           with tunable camera FOV/tilt and 3D odometry)
scripts/                   Nodes (executables):
  flight_controller.py     Only node that talks to Gazebo: passes on cmd_vel
                           and holds the target altitude
  terrain_mapper.py        Terrain map from the depth camera, trail detection,
                           ranger route service, map saving
  mission_manager.py       Mapping / Passive / Active / Return Home modes
  waypoint_follower.py     Fly the fixed route via Nav2's NavigateToPose action
  publish_waypoints.py     Preview the fixed route in RViz (no Nav2 calls)
  web_dashboard_bridge.py  HTTP bridge for the GUI
                           Library modules (imported by the nodes):
  terrain_map.py           The terrain map itself (numpy, no ROS)
  trail_detector.py        Trails from the map (distance transform + gradient)
  route_planner.py         Ranger routes (A2 grid search with energy costs)
  coverage_explorer.py     Frontier selection for Mapping mode (AT3 explorer)
  mapping_mode.py          Mapping mode's Nav2 goal handling (AT3)
  mission_modes.py         Path following (pure pursuit) and hovering modes
  grid_tools.py            Distance transform, A2 smoother, small helpers
  node_params.py           Exposes the modules' settings as ROS parameters
  mission_waypoints.py     Shared fixed route + RViz marker builder
test/                      Unit tests for the library modules (no simulator)
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
colcon build --packages-select 41068_ignition_bringup 41068_assignment
source install/setup.bash
```

Build both packages after pulling: worlds and models such as
`hiking_trail_1` / `trail_plane` live in `41068_ignition_bringup`, and if its
install is out of date Gazebo exits straight away (`ign gazebo ... process
has died ... exit code 255` near the top of the launch output), the drone is
never spawned, and Nav2 then waits for `parrot1_odom` for ever.

## Demo

```bash
ros2 launch 41068_assignment demo.launch.py
```

This starts Will's `simple_trees` world with the drone on the launch pad, plus
Nav2, RViz and mapping mode. It then adds each team member's part from
`launch/modules/` and skips any part that hasn't been merged yet. Turn a part
off with its flag, e.g. `detection:=False`. See
[launch/modules/README.md](launch/modules/README.md) to add your part.

## Running it

1. Launch the sim with SLAM + Nav2:

   ```bash
  ros2 launch 41068_assignment spawn_parrot.launch.py rviz:=True nav2:=True web_dashboard:=True
   ```

   (`nav2:=True` starts SLAM Toolbox automatically as well.)

  The `web_dashboard:=True` flag starts the local HTTP bridge that feeds the browser page live telemetry and camera streams.

2. Serve the webpage:

  ```bash
  cd /home/student/subjects/41068_robotics_studio_1/ros2_ws/src/41068_assignment/GUI
  python3 -m http.server 8000
  ```

  Then open `http://localhost:8000` in your browser.

3. (Optional) Preview the planned route in RViz before flying it — this only
   publishes markers, it never sends the drone anywhere:

   ```bash
   ros2 run 41068_assignment publish_waypoints.py --ros-args -r __ns:=/parrot1
   ```

   In RViz, add a **MarkerArray** display on the `/parrot1/waypoint_markers`
   topic to see the route: numbered spheres at each waypoint, colour-coded
   grey (visited) / green (current target) / orange (upcoming).

4. Fly the route:

   ```bash
   ros2 run 41068_assignment waypoint_follower.py --ros-args -r __ns:=/parrot1
   ```

   Stop the preview node first (or just leave it running — both publish to
   the same topic, so the live mission's colours will simply take over).

## Mission modes

### Launching

```bash
# Default world (assignment_world)
ros2 launch 41068_assignment spawn_parrot.launch.py nav2:=True rviz:=True

# The trail world from 41068_ignition_bringup (trails to detect, 25 x 25 m)
ros2 launch 41068_assignment spawn_parrot.launch.py nav2:=True rviz:=True \
    world_package:=41068_ignition_bringup world:=hiking_trail_1
```

`nav2:=True` is needed for Mapping mode. Other launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `world` / `world_package` | `assignment_world` / `41068_assignment` | Loads `<world_package>/worlds/<world>.sdf`. Needs a matching `config/worlds/<world>.yaml` (bounds + ranger station) when `mission:=True` |
| `x`, `y`, `z` | `0.0`, `-4.0`, `0.8` | Spawn pose, which is also *home* for Return Home |
| `gui` | `True` | `False` runs Gazebo without its window (sensors still work) |
| `mission` | `True` | Start `terrain_mapper` and `mission_manager` |
| `terrain_map` | empty | A `terrain.npz` saved earlier, to start from instead of a blank map |

### Commands

Modes are switched by a word on `/parrot1/mission_command`, the same topic the
web dashboard's buttons publish to. From a terminal:

```bash
ros2 topic pub --once /parrot1/mission_command std_msgs/msg/String "{data: 'mapping'}"
ros2 topic echo /parrot1/mission_state        # e.g. "MAPPING: 42% observed, flying to F7 at [3.1, -2.0]"
```

| Command | What happens |
|---|---|
| `mapping` | Settle at 0.8 m, then fly to frontiers (the edge of the ground the camera has seen) until none are left. Then save the terrain map and Return Home |
| `passive T3` | Climb on the spot to 8 m, fly to the nearer end of trail T3 and along it, then hover at its end |
| `active` | Drop to 3 m over the current spot and hover; when a `person_detection` Pose2D is received, confirm it when the latest thermal frame contains at least 20 pixels at or above 30 C and report the confirmed position |
| `return_home` | Climb to 8 m, fly straight home, descend to the spawn height |
| `stop` / `take_over` | Stop all autonomy and hover where it is; the dashboard's manual sliders keep working |
| `takeoff` | Hover at 0.8 m |
| `save_map` | Save the terrain map now (see below) |

Trail names (T1 = longest) come from the trail detector. List them with:

```bash
ros2 topic echo --once /parrot1/trails --field data
```

Trails are only detected where the camera has seen them, so run Mapping mode
(or load a saved map) before Passive mode. Names are re-assigned by length each
time detection runs (every 5 s), so check the list right before sending a
`passive` command.

While Mapping is active, a recent thermal frame is used as the coarse person
cue and a `person_detection` Pose2D supplies the precise map-frame location.
When both are available, Mapping cancels its current Nav2 goal, the mission
switches to Active mode, and the terminal logs the confirmed position. A
thermal frame older than the `thermal_max_age` parameter (default 1 second) is
not used for confirmation.

### Ranger route

`/parrot1/plan_ranger_route` (`nav_msgs/srv/GetPlan`) returns the least-effort
walking route over the terrain map. It also publishes it on
`/parrot1/ranger_route` for RViz. With no start frame, the route starts at the
world's ranger station (`config/worlds/<world>.yaml`):

```bash
# From the ranger station to (8, 8)
ros2 service call /parrot1/plan_ranger_route nav_msgs/srv/GetPlan \
    "{goal: {pose: {position: {x: 8.0, y: 8.0}}}}"

# From a given start
ros2 service call /parrot1/plan_ranger_route nav_msgs/srv/GetPlan \
    "{start: {header: {frame_id: parrot1_map}, pose: {position: {x: -10.0, y: 10.0}}},
      goal: {pose: {position: {x: 10.0, y: -10.0}}}}"
```

The route prefers trails, walks around hills and low branches when the detour
is cheaper, and never squeezes through a gap narrower than 0.6 m.
`terrain_mapper` logs its length, energy and how much of it is on trail.

### Saving and loading maps

Mapping mode saves the terrain map when it finishes. To save it at any time:

```bash
ros2 service call /parrot1/terrain/save std_srvs/srv/Trigger
```

This writes `~/41068_maps/<world>_<date>_<time>/` with `terrain.npz` (the map),
`trails.json` and `terrain.png` (a preview: white free, black obstacle, brown
trail, grey unseen). To start a later run from it:

```bash
ros2 launch 41068_assignment spawn_parrot.launch.py nav2:=True \
    world_package:=41068_ignition_bringup world:=hiking_trail_1 \
    terrain_map:=$HOME/41068_maps/<folder>/terrain.npz
```

The SLAM map (the lidar map Nav2 uses) is separate:

```bash
# Occupancy image + yaml (for map_server / viewing)
ros2 run nav2_map_server map_saver_cli -f $HOME/41068_maps/slam_map \
    --ros-args -r map:=/parrot1/map -p map_subscribe_transient_local:=true
# Pose graph (to continue mapping or localise against later)
ros2 service call /parrot1/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$HOME/41068_maps/slam_map'}"
```

(SLAM Toolbox's own `save_map` service fails here (result 255): it runs the
map saver on `/map`, but this stack's map is `/parrot1/map`.)

### Seeing it in RViz

Add these displays (all under `/parrot1/`):

| Topic | Display | Shows |
|---|---|---|
| `terrain/occupancy` | Map | What the camera has seen: unknown / free / obstacle |
| `terrain/cloud` | PointCloud2 (Color Transformer: RGB8) | The terrain at its ground height: green grass, brown trail, dark green under branches, red obstacle |
| `terrain/trail_markers` | MarkerArray | Detected trails with their names |
| `exploration_markers` | MarkerArray | Mapping mode's frontiers (cyan), current goal (green arrow), failed goals (red) |
| `ranger_route` | Path | The last ranger route planned |

### How it works

```
 depth camera ──► terrain_mapper ──► terrain/occupancy ──► mission_manager ──► Nav2 (Mapping)
 (RGB-D cloud)      │   │    │                              │    │
                    │   │    └─► trails ───────────────────►┘    ├─► cmd_vel (Passive / Return Home)
                    │   └─► plan_ranger_route service             └─► target_altitude
                    └─► terrain/save
 dashboard ─► mission_command ─► mission_manager
 Nav2, dashboard sliders, mission_manager ─► cmd_vel ─► flight_controller ─► cmd_vel_gz ─► Gazebo
```

- **Terrain map** (`terrain_map.py`). SLAM's map can't be the terrain map: SLAM
  Toolbox only marks free space along lidar beams that hit something, so open
  ground stays "unknown", and a horizontal lidar can't see the ground's
  height or colour. The depth camera can. Every camera point is binned into
  a 0.1 m grid cell. The lowest point in a cell is its ground height. Points
  0.3–1.2 m above the surrounding ground are obstacles a person can't get
  past (trunks, rocks, bushes). Points 1.2–2.0 m up are low branches, passable
  but slow. Ground points' colour (brown vs green) votes on whether the cell
  is trail. Terrain steeper than 35° is an obstacle (the A2 "large gradient"
  rule). Above 3 m the camera sees tree tops instead of the ground, so it
  then only adds trail colour to ground already mapped.
- **Mapping mode** (`coverage_explorer.py`, `mapping_mode.py`). This is the AT3
  frontier explorer and its Nav2 goal handling, run on the terrain map: a
  frontier is the edge between ground the camera has seen and ground it
  hasn't. The terrain grid only covers the world's area, so the world's edges
  bound the exploration. The drone flies at 0.8 m, so the lidar scans at chest
  height (what a walker would hit) and the camera sees under the canopy.
- **Trails** (`trail_detector.py`). Take the trail cells' distance transform
  (distance to the nearest grass). Its ridge, where the gradient is flat
  (|∇d| < 0.6, as in AT3's `cave_geometry.py`), is the trail centreline. The
  trail mask is thinned to a skeleton and split at junctions. Sections that
  mostly lie on the ridge are kept as trails; spurs into the trail edges
  don't. Each is smoothed with the A2 smoother.
- **Ranger route** (`route_planner.py`). This is A2's planner on the terrain
  grid: Dijkstra (A2's A* with heuristic weight 0, as A2 recommends for
  energy costs) with the A2 energy cost `|(μmg cos θ + mg sin θ) dx|`. θ is
  the slope from the terrain map, and μ depends on the terrain: trail 0.05,
  grass 0.10, low branches 0.25, unseen ground 0.30. Obstacles are grown by a
  person's half-width (0.3 m) with the distance transform, so narrow gaps
  aren't used. The path comes from parent back-pointers and is smoothed with
  A2's smoother.
- **Passive mode / Return Home** (`mission_modes.py`). The drone climbs on the
  spot to 8 m, which clears the tallest tree (oak, 6.5 m). It then steers along
  the trail by pure pursuit, aiming at a point 2 m ahead on the path. Nav2
  isn't used up there: the lidar sees nothing at that height, and Nav2's tree
  outlines from the low-altitude map would push the drone off the trail.
- **Flight controller** (`flight_controller.py`). Nav2 only commands forward
  speed and turn rate, so this node adds the vertical speed that holds
  `target_altitude` (a proportional controller). It is the only publisher to
  Gazebo, so the two never fight.

### Frames and altitude

- `parrot1_map` lines up with the world, so world-file coordinates (trees,
  bounds, trails) are map coordinates. The EKF fuses Gazebo's absolute x, y
  and heading, standing in for GPS and a compass. Before this, it integrated
  two bogus velocity readings Gazebo sends right after spawning, which put
  the map 11.3 m away from the world.
- TF is 2D (the drone's z is 0 in TF), as SLAM Toolbox and Nav2 expect. The
  altitude comes from `/parrot1/odometry`, which is now 3D.

### Tuning

Each module's settings are in a dataclass at the top of its file
(`TerrainConfig`, `TrailConfig`, `RouteConfig`, `FollowConfig`), with a
comment on each. They are also ROS parameters, e.g. `terrain.max_slope_deg`,
`trails.min_trail_length`, `route.mu_trail`, `follow.lookahead`. Override
them in `config/worlds/<world>.yaml` or with `--ros-args -p`, writing floats
as floats (`1.0`, not `1`). The mission manager's altitudes and timeouts
are ROS parameters declared at the top of `mission_manager.py`.

### Tests

```bash
cd ros2_ws/src/41068_assignment
python3 -m pytest test
```

They cover the terrain map, trail detection, routing, frontier selection and
path following on small synthetic maps, in under a second.

### Known limits

- **Nav2 only sees obstacles below 2 m** (`max_obstacle_height` in bringup's
  `nav2_params_parrot1.yaml`), so Mapping mode stays at 0.8 m.
- **Hills.** On a heightmap world the lidar is still a flat plane at a fixed
  height, so rising ground ahead looks like a wall to SLAM and Nav2, and
  0.8 m above the spawn point is not 0.8 m above a hill. The terrain map
  itself handles height; flying over hills needs height-above-ground control
  and lidar filtering, not done yet.
- **Trails are junction-to-junction sections.** Passive mode flies one
  section; following a route through the trail network is not done yet.
- **Passive flights at 8 m don't improve the walking map under trees**: the
  camera only sees the canopy from up there.

## Changing the route

Edit `DEFAULT_WAYPOINTS` in `scripts/mission_waypoints.py`. Each waypoint is
`(x, y, yaw_degrees)` in the drone's `parrot1_map` frame, which now equals
the world frame (see [Frames and altitude](#frames-and-altitude)). Both
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
- The EKF parameters come from bringup's `robot_localization_parrot1.yaml`,
  with `odom0_config` overridden in `spawn_parrot.launch.py` (see
  [Frames and altitude](#frames-and-altitude)).
- Gazebo's `cmd_vel` is fed only by `flight_controller.py` (the bridge's
  `cmd_vel` is remapped to `cmd_vel_gz`). Anything that wants to move the
  drone publishes on `/parrot1/cmd_vel` as before.
- If Gazebo crashes on startup with an `Ogre::UnimplementedException`, this
  is a known WSL2 + Mesa D3D12 rendering limitation, not a bug in this
  package. Work around it with:

  ```bash
  export LIBGL_ALWAYS_SOFTWARE=1
  ```

  or run without the Gazebo window (`gui:=False`).
