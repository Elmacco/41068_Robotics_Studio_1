# 41068 Assignment Package

Spawns the Parrot drone in a world, runs Nav2 for it, and flies it in
one of three mission modes:

- **Mapping**: explores the world on its own until the camera has seen all of
  it, building a terrain map (ground height, obstacles, trails), then saves
  the map and flies home.
- **Passive**: once Mapping has found trails, patrols one the ranger picks,
  low (0.8 m) with Nav2, watching for people.
- **Active**: when Passive sees a person, flies to a standoff facing them,
  confirms them and checks whether their shirt is red close up, then Passive
  carries on.

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
  navigation.launch.py     Localisation (map -> odom) + Nav2 (copy of bringup's)
config/
  <world>.yaml in worlds/  Per-world mapping bounds and ranger station
  *.yaml, *.rviz, ...      Bridge, EKF, SLAM, Nav2, RViz and Gazebo server
                           configs (copies of bringup's, see below)
worlds/
  assignment_world.sdf     Ground plane + a few trees
  simple_trees.sdf         Will's demo world (launch pad, trail, trees)
  hiking_trail_1.sdf       Trail world for trail detection (25 x 25 m)
models/                    grass_plane, trail_plane, elevated_trail, launch_pad,
                           rescue_randy_base/_red/_green (Ethan's)
weights/yolo26n.pt         YOLO weights for target_detector.py (offline use)
maps/<world>/terrain.npz   Preloaded terrain map the demo starts from (see
                           maps/README.md for recording it)
requirements.txt           pip-only dependencies (ultralytics)
urdf_parrot/               Parrot + thermal camera (copies of bringup's xacros
                           and meshes, with tunable camera FOV/tilt and 3D
                           odometry)
scripts/                   Nodes (executables):
  flight_controller.py     Only node that talks to Gazebo: passes on cmd_vel
                           and holds the target altitude
  terrain_mapper.py        Terrain map from the depth camera, trail detection,
                           ranger route service, map saving
  mission_manager.py       Mapping / Passive / Active / Return Home modes
  waypoint_follower.py     Fly the fixed route via Nav2's NavigateToPose action
  publish_waypoints.py     Preview the fixed route in RViz (no Nav2 calls)
  web_dashboard_bridge.py  HTTP bridge for the GUI
  target_detector.py       People in the RGB image (YOLO), flagged red or not
  target_projector.py      Detection + depth + thermal -> map-frame people
                           list (points_of_interest) and target_pose
                           Library modules (imported by the nodes):
  terrain_map.py           The terrain map itself (numpy, no ROS)
  trail_detector.py        Trails from the map (distance transform + gradient)
  trail_graph.py           Ranger routes over a roadmap (A2 PRM + Dijkstra)
  route_planner.py         Walking costs, and the A2 grid search to compare
  map_overview.py          Top-down picture of the map for the dashboard
  coverage_explorer.py     Frontier selection for Mapping mode (AT3 explorer)
  mapping_mode.py          Mapping mode's Nav2 goal handling (AT3)
  mission_modes.py         Path following (pure pursuit) and hovering modes
  passive_mode.py          Passive mode: patrol a trail low with Nav2
  active_mode.py           Active mode: fly to a found red-shirted person
  nav_goal.py              One Nav2 goal at a time, for Passive and Active
  person_approach.py       Its standoff goal and state machine (AT3 Planning 2/3)
  person_registry.py       People list: merges repeat sightings, stable ids
  thermal_check.py         Body-heat check inside a YOLO person box
  grid_tools.py            Distance transform, A2 smoother, small helpers
  node_params.py           Exposes the modules' settings as ROS parameters
  mission_waypoints.py     Shared fixed route + RViz marker builder
  colour_classifier.py     Red-shirt check for target_detector.py
  test_colour_classifier.py  Standalone test for it (python3 <file>)
test/                      Unit tests for the library modules (no simulator)
```

Colour detection is enabled by default when the detection module is launched.
To start the full demo with detection enabled:

```bash
ros2 launch 41068_assignment demo.launch.py detection:=True
```

To run only the detection module and explicitly enable colour classification:

```bash
ros2 launch 41068_assignment launch/modules/detection.launch.py colour_enabled:=true
```

The detector publishes `target_detections`, each person flagged `red` or
`none`. The projector adds a person to its list (`points_of_interest`) only
when the thermal frame within 1 s of the detection has at least 20 pixels at
or above 30 C inside that person's own box, in at least 3 frames. Active mode
works from that list, and checks each person's shirt from the detector's
flags at its standoff.

## Prerequisites

Everything this package launches (worlds, models, URDF, configs) is inside
this package. Most of it started as copies from `41068_ignition_bringup`, see
[Copied from the course package](#copied-from-the-course-package). That
package is still needed for course worlds like `large_demo`. Also needs:

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-vision-msgs
# Person detection (target_detector.py); pulls in PyTorch
pip install -r src/41068_assignment/requirements.txt
```

Build:

```bash
cd ros2_ws
colcon build --packages-select 41068_ignition_bringup 41068_assignment
source install/setup.bash
```

Rebuild after pulling: worlds and models are installed copies, and if the
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

The terrain map starts from the world's preloaded map,
`maps/<world>/terrain.npz`, so trails, routes and the dashboard map work
straight away (see [maps/README.md](maps/README.md); without the file it
starts blank). To show Mapping mode building the map, start blank:

```bash
ros2 launch 41068_assignment demo.launch.py terrain_map:=''
```

## Running it

1. Launch the sim with Nav2:

   ```bash
  ros2 launch 41068_assignment spawn_parrot.launch.py rviz:=True nav2:=True web_dashboard:=True
   ```

   The drone localises from odometry + GPS: the EKF fuses Gazebo's odometry
   pose as a GPS + compass stand-in, so `parrot1_map` is the world frame and
   `parrot1_map -> parrot1_odom` is the identity. The map is built live from
   the depth camera (terrain_mapper). `slam:=True` runs SLAM Toolbox for
   `map -> odom` instead, as the course package did; on top of GPS it only
   added error (in simple_trees it rotated the map 40 degrees while the drone
   sat still).

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

# The trail world (trails to detect, 25 x 25 m)
ros2 launch 41068_assignment spawn_parrot.launch.py nav2:=True rviz:=True \
    world:=hiking_trail_1
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
ros2 topic echo /parrot1/active_mode          # true while Active mode is running
```

The latched `active_mode` topic publishes `std_msgs/Bool`: `true` whenever the
mission enters Active mode and `false` whenever it leaves. The GUI can use it
as a direct mode flag without parsing `mission_state`.

| Command | What happens |
|---|---|
| `mapping` | Settle at 0.8 m, then fly to frontiers (the edge of the ground the camera has seen) until none are left. Then save the terrain map and Return Home |
| `passive T3` | At 0.8 m, fly with Nav2 to the nearer end of trail T3 and along it, then hover at its end. A person seen on the way starts Active mode to check their shirt, then the patrol carries on. `passive` alone takes the trail nearest the drone |
| `active` | Hover where it is (dropping to 3 m if higher; it never climbs) until a person is on the people list, fly to a standoff facing them, confirm them and check their shirt close up, then go Idle |
| `return_home` | Climb to 8 m, fly straight home, descend to the spawn height |
| `stop` / `take_over` | Stop all autonomy and hover where it is; the dashboard's manual sliders keep working |
| `takeoff` | Hover at 0.8 m |
| `save_map` | Save the terrain map now (see below) |

Trail names (T1 = longest) come from the trail detector. List them with:

```bash
ros2 topic echo --once /parrot1/trails --field data
```

Trails are only detected where the camera has seen them, so run Mapping mode
(or load a saved map) before Passive mode. The dashboard's Passive button
stays greyed out until there are trails; pick one under the buttons (or
*Nearest trail*). Names are re-assigned by length each time detection runs
(every 5 s), so from a terminal check the list right before sending a
`passive` command.

While Passive is patrolling, any person on the people list who was
seen in the last 2 s (`person_recent`) interrupts it: Passive cancels its Nav2
goal and Active mode approaches the person, then Passive resumes from the
trail waypoint it was heading to. Each person (P1, P2, ...) interrupts once,
so they do not stop it again while still in view. Mapping does not stop for
people: it only maps, and the patrol afterwards finds them.

Active mode is the close-range inspection from 49274 AT3 (Planning 2 and 3),
in `person_approach.py`:

- **Standoff goal.** Candidates on rings 3.0, 3.5, 4.0, 4.5 and 5.0 m round the
  person, 24 each. A candidate must be observed free ground with 1.0 m of
  clearance from obstacles and unknown ground (0.5 m as a fallback), reachable
  on Mapping's wavefront, and in line of sight of the person (A2's
  `is_occluded`, ignoring the last metre, which is the person). All rings
  are compared on travel + 2 m per metre further out, and the goal faces the
  person. A goal beyond Nav2's rolling costmap is reached in 9 m hops.
- **State machine.** PLANNING -> APPROACHING -> VERIFYING -> DONE. On arrival
  the drone turns to face the person (Nav2 ends with any yaw), and the person
  counts as *visited* only if the people list sees them again within 5 s
  (after at least 1.5 s there). A Nav2 failure, the 40 s approach timeout or
  not seeing them retries once from a different standoff, then *abandons*.
  The goal is re-planned if the person's estimate moves more than 0.75 m.
  With no valid standoff for 5 s (ground round them not mapped yet) it
  abandons too.
- **Shirt check.** From afar the torso is a few pixels wide and the
  detector's red flag is unreliable, so every person is approached, not only
  red ones. Each detector frame at the standoff is a vote from the largest
  box in view (the person, 3-5 m away and being faced), and the shirt is
  red with half the votes or more (`active.shirt_red_fraction`). With no
  frames there (an abandoned approach) it falls back to the people list's
  type. The result is logged and published, latched, on `checked_people`:

  ```bash
  ros2 topic echo /parrot1/checked_people
  # {"people": [{"id": "P3", "x": -15.0, "y": 12.0, "shirt": "red",
  #   "how": "7/9 close-up frames red", "close_up": true, "red_frames": 7,
  #   "frames": 9, "visited": true}]}
  ```

Every threshold is an `active.*` parameter of `mission_manager`.

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

The route keeps to the trails (walking on grass costs ten times as much),
leaves them only for the last stretch to a point off the trail or to cross
between separate trail systems, walks around hills and low branches when the
detour is cheaper, and never squeezes through a gap narrower than 0.6 m.
`terrain_mapper` logs its length, energy and how much of it is on trail, and
publishes the same on `/parrot1/ranger_route/info`.

### Dashboard map

The dashboard's Top Down Map panel shows the terrain map from above (grey
unmapped, green grass, brown trail shaded by its distance transform, black
obstacles), the detected trails with their names and networks, the ranger
station, the drone and its heading, and points of interest. Click a point of
interest, or anywhere on the map, to plan the least-effort walking route from
the station to it; the route and its length, energy and share on trail appear
under the map. Show Roadmap overlays the route graph.

Points of interest come from `/parrot1/points_of_interest` (`std_msgs/String`,
JSON), drawn red, green or white by `type`. Nothing publishes it yet: it is
the slot for the people found by the detection part. To try it by hand:

```bash
ros2 topic pub --once /parrot1/points_of_interest std_msgs/msg/String \
    "{data: '{\"points\": [{\"id\": \"P1\", \"x\": -15, \"y\": 12, \"type\": \"red\"}]}'}" \
    --qos-durability transient_local
```

### Saving and loading maps

Mapping mode saves the terrain map when it finishes. To save it at any time:

```bash
ros2 service call /parrot1/terrain/save std_srvs/srv/Trigger
```

This writes `~/41068_maps/<world>_<date>_<time>/` with `terrain.npz` (the map),
`trails.json` and `terrain.png` (a preview: white free, black obstacle, brown
trail, grey unseen). Copy a good one into `maps/<world>/` to make it the
demo's preloaded map ([maps/README.md](maps/README.md)). To start a later run
from it:

```bash
ros2 launch 41068_assignment spawn_parrot.launch.py nav2:=True \
    world:=hiking_trail_1 \
    terrain_map:=$HOME/41068_maps/<folder>/terrain.npz
```

With `slam:=True`, SLAM Toolbox's lidar map is separate (nothing uses it):

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
 depth camera ──► terrain_mapper ──► terrain/occupancy ──► mission_manager ──► Nav2 (Mapping, Passive, Active)
 (RGB-D cloud)      │   │    │                              │    │
                    │   │    └─► trails ───────────────────►┘    ├─► cmd_vel (Return Home)
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
  but slow. A cell is trail by colour (its ground points are mostly brown,
  `hiking_trail_1`) or by height (most of its points are 0.2–0.3 m above
  the grass within 1.5 m, `simple_trees`, whose trail is a raised path and
  whose grass is tinted brown), set per world by `terrain.trail_source`.
  Both are majority votes: views of the same spot disagree by up to ~0.3 m,
  and judging the path by its lowest point, as before, lost 22% of it in
  strips along its edges, which split trails into pieces. Terrain steeper
  than 35° is an obstacle (the A2 "large gradient" rule) if it also rises at
  least 0.3 m within 1 m, so the raised path's 0.23 m edge is a step, not a
  wall. Above 3 m the camera sees tree tops instead of the ground, so it then
  only adds trail colour to ground already mapped.
- **Mapping mode** (`coverage_explorer.py`, `mapping_mode.py`). This is the AT3
  frontier explorer and its Nav2 goal handling, run on the terrain map: a
  frontier is the edge between ground the camera has seen and ground it
  hasn't. The terrain grid only covers the world's area, so the world's edges
  bound the exploration. The drone flies at 0.8 m, so the lidar scans at chest
  height (what a walker would hit) and the camera sees under the canopy.
  A goal is given up once, for 8 s, the camera has seen no new ground and the
  drone has got no closer, and a goal reached having seen under 1 m² of new
  ground counts as failed; both are blacklisted over 3 m. That stops the
  drone circling a frontier it cannot see into, such as the ground hidden
  behind the standing person by the launch pad. 25 s is only a backstop
  (`stall_time`, `min_goal_gain`, `goal_timeout` in `mission_manager.py`).
- **Trails** (`trail_detector.py`). Take the trail cells' distance transform
  (distance to the nearest grass). Its ridge, where the gradient is flat
  (|∇d| < 0.6, as in AT3's `cave_geometry.py`), is the trail centreline. The
  trail mask is thinned to a skeleton and split at junctions. Sections that
  mostly lie on the ridge are kept as trails; spurs into the trail edges
  don't. Each is smoothed with the A2 smoother. Trails on the same connected
  piece of trail share a network (N1, N2, ...): `simple_trees` has two,
  either side of the launch pad.
- **Ranger route** (`trail_graph.py`, `route_planner.py`). This is A2's PRM
  with Dijkstra (A2's A* with heuristic weight 0, as A2 recommends for energy
  costs). Nodes go every 1 m along each trail's centreline, the ridge of its
  distance transform (A2 Task 10's idea), plus 300 randomly sampled free
  cells (A2 Task 7) so a route can leave the trail; edges join nodes within
  4 m whose straight line is clear. The edge cost is A2's energy
  `|(μmg cos θ + mg sin θ) dx|`, with θ from the terrain map and μ averaged
  over the cells the edge crosses: trail 0.05, grass 0.5, low branches 1.25,
  unseen ground 1.5. Obstacles are grown by a person's half-width (0.3 m)
  with the distance transform, so narrow gaps aren't used. The route comes
  from parent back-pointers. On `simple_trees` the roadmap has ~500 nodes,
  is built in ~0.3 s whenever the map changes and answers a route in 3–5 ms;
  A2's grid search over every cell (`route.planner: grid`) takes 0.5–1.4 s
  for routes of the same cost, and is still used if the roadmap finds no
  route. Uniform sampling alone gave routes costing 1.5–2× more, as random
  nodes rarely land on a 1.5 m trail.
- **Passive mode** (`passive_mode.py`). The trail is cut into waypoints every
  2 m from its end nearer the drone, and Nav2 is sent to one at a time, each
  facing along the trail so the camera looks where a hiker would be. Within
  1 m of a waypoint the next goal goes out (Nav2 preempts the running one), so
  the drone does not stop at each. A waypoint Nav2 cannot reach is skipped;
  more than 3 in a row ends the patrol. A trail end beyond Nav2's 20 m costmap
  is reached in 9 m hops, as Active mode does. It flies at 0.8 m, not above
  the trees: the depth camera reaches 10 m, so from 8 m (camera tilted 45°
  down) the projector could only place people within about 6 m, and Active
  mode would then have to descend near the oaks, whose canopy starts at
  1.04 m. Settings are `passive.*` parameters (`PassiveConfig`).
- **Return Home** (`mission_modes.py`). The drone climbs on the spot to 8 m,
  which clears the tallest tree (oak, 6.5 m). It then steers home by pure
  pursuit, aiming at a point 2 m ahead on the path. Nav2 isn't used up there:
  the lidar sees nothing at that height, and Nav2's tree outlines from the
  low-altitude map would push the drone off course.
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
- `terrain_mapper` places each camera cloud at the odometry altitude from
  the moment the cloud was taken, not the latest one, and skips clouds taken
  while the drone was climbing, descending (`max_climb_rate`) or tilted
  (`max_tilt`). Using the latest odometry, which also queued behind the
  cloud processing, put the ground 1-2 m too low wherever the drone climbed
  (the corner where Mapping ended, as Return Home climbed on the spot), and
  since a cell's ground is the lowest point seen there, it stayed wrong.

### Tuning

Each module's settings are in a dataclass at the top of its file
(`TerrainConfig`, `TrailConfig`, `RouteConfig`, `GraphConfig`,
`FollowConfig`, `PassiveConfig`), with a comment on each. They are also ROS
parameters, e.g. `terrain.trail_source`, `trails.min_trail_length`,
`route.mu_trail`, `graph.grass_nodes`, `follow.lookahead`,
`passive.spacing`. Override
them in `config/worlds/<world>.yaml` or with `--ros-args -p`, writing floats
as floats (`1.0`, not `1`). The mission manager's altitudes and timeouts
are ROS parameters declared at the top of `mission_manager.py`.

### Tests

```bash
cd ros2_ws/src/41068_assignment
python3 -m pytest test
```

They cover the terrain map, trail detection, the route roadmap and grid
search, the dashboard map picture, frontier selection and path following on
small synthetic maps, in a few seconds.

### Known limits

- **Nav2 only sees obstacles below 2 m** (`max_obstacle_height` in
  `config/nav2_params_parrot1.yaml`), so Mapping mode stays at 0.8 m.
- **Hills.** On a heightmap world the lidar is still a flat plane at a fixed
  height, so rising ground ahead looks like a wall to SLAM and Nav2, and
  0.8 m above the spawn point is not 0.8 m above a hill. The terrain map
  itself handles height; flying over hills needs height-above-ground control
  and lidar filtering, not done yet.
- **Trails are junction-to-junction sections.** Passive mode flies one
  section; following a route through the trail network is not done yet.
- **Trails by height need flat ground.** The height rule compares each cell
  with the lowest ground within 1.5 m, which on a slope is downhill ground:
  4° is already 0.1 m. It suits `simple_trees` (flat, raised path); worlds
  with hills should use `trail_source: colour`.
- **The lidar never sees the raised path**: it scans at ~1 m. Only the depth
  camera does, so the trail is only in the terrain map, not SLAM's map.

## Changing the route

Edit `DEFAULT_WAYPOINTS` in `scripts/mission_waypoints.py`. Each waypoint is
`(x, y, yaw_degrees)` in the drone's `parrot1_map` frame, which now equals
the world frame (see [Frames and altitude](#frames-and-altitude)). Both
`publish_waypoints.py` and `waypoint_follower.py` read from this same list,
so there's only one place to change it. The route can also be overridden per
run via the `waypoints` ROS parameter (a flat list of the same triples).

By default the route does not loop once it reaches the last waypoint; pass
`--ros-args -p loop:=true` to `waypoint_follower.py` to repeat it.

## Copied from the course package

These started as copies of files in `41068_ignition_bringup`, so the demo
runs from this package alone. The course package is unchanged and still
holds the originals. If the course updates one, re-copy it and reapply the
change listed here.

| Here | Changed from the original |
|---|---|
| `config/ignition_server.config`, `gazebo_bridge_clock.yaml`, `gazebo_bridge_parrot1.yaml`, `robot_localization_parrot1.yaml`, `41068_parrot1.rviz` | Not changed |
| `config/nav2_params_parrot1.yaml`, `slam_params_parrot1.yaml` | The team's drone tuning (made in the course package before it was copied) |
| `launch/navigation.launch.py` | Reads params from this package's `config/` |
| `urdf_parrot/common_properties.urdf.xacro`, `meshes/` | Not changed |
| `worlds/simple_trees.sdf`, `worlds/hiking_trail_1.sdf` | Team-made worlds, not changed |
| `models/grass_plane`, `elevated_trail`, `launch_pad` | Not changed |
| `models/rescue_randy_*`, `scripts/target_*.py`, `colour_classifier.py`, `test_colour_classifier.py`, `config/gazebo_bridge_parrot1_camera_info.yaml`, `requirements.txt`, `weights/` | Copied from `Ethan_Colour_Detector_Module_Needs_Integration/`, not the course package. `target_projector.py` has two fixes: the point is rotated from optical axes into `camera_link` axes (the frame Gazebo stamps the depth image with), and the TF lookup no longer fails when TF lags the image |
| `models/trail_plane` | Collision is a flat plane matching the visual; the hills heightmap is commented out until its visual loads (it was invisible but solid). Only the two textures it uses were copied |

## Notes on the Nav2/SLAM setup

- `nav2_params_parrot1.yaml` and `slam_params_parrot1.yaml` (in `config/`)
  are copies of the course ones, but the Nav2 tolerances and costmap inflation radius have been tuned down
  for the drone's small footprint and this world's short waypoint spacing.
- The footprint is 0.6 x 0.6 m (the drone's collision box is 0.5 x 0.5 m;
  the course's 1 x 1 m kept it over 1 m from everything) with a 0.5 m
  inflation radius, and DWB may move sideways at up to 0.3 m/s (the drone
  is holonomic), so it edges round obstacles instead of turning on the spot
  and driving. The velocity smoother has the same y limits.
- The global costmap uses a rolling window (`rolling_window: true`, 20x20m)
  rather than relying solely on the SLAM-built static map's extent. Without
  this, path planning can fail for any goal or start pose that falls outside
  whatever area SLAM has explored so far.
- The EKF parameters come from `config/robot_localization_parrot1.yaml`,
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
