# Preloaded terrain maps

`demo.launch.py` starts the terrain mapper from `maps/<world>/terrain.npz`, so
trails, the route roadmap, ranger routes and the dashboard map all work as
soon as the demo is up, without flying a full mapping run first. If the file
is missing the mapper warns and starts blank.

The map frame is the world frame on every run (the EKF fuses Gazebo's
absolute pose, see `spawn_parrot.launch.py`), so a map recorded once lines up
with every later run of the same world. `TerrainMap.load` refuses a map saved
with different bounds or resolution.

## Recording a map

1. Start from a blank map and run Mapping mode until it finishes (it saves
   the map and flies home):

   ```bash
   ros2 launch 41068_assignment demo.launch.py terrain_map:=''
   ros2 topic pub --once /parrot1/mission_command std_msgs/msg/String "{data: 'mapping'}"
   ```

   Or save at any point with the dashboard's Save Map button, or
   `ros2 service call /parrot1/terrain/save std_srvs/srv/Trigger`.

2. The mapper logs the folder it saved to,
   `~/41068_maps/<world>_<date>_<time>/`. Check its `terrain.png` (white
   seen, grey unseen, brown trail, black obstacles), then copy the map in and
   rebuild:

   ```bash
   mkdir -p src/41068_assignment/maps/simple_trees
   cp ~/41068_maps/simple_trees_<date>_<time>/terrain.npz src/41068_assignment/maps/simple_trees/
   colcon build --packages-select 41068_assignment
   ```

To show Mapping mode in the demo, launch with `terrain_map:=''`; on a
complete map it has nothing left to explore and finishes at once.
