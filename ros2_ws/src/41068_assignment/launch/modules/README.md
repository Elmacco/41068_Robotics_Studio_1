# Demo modules

`demo.launch.py` (one folder up) starts the sim, drone, Nav2 and mapping mode,
then includes one launch file per team member's part from this folder.
If a part's file isn't here yet, the demo logs that it's skipped and carries on,
so you can merge your part whenever it's ready.

| Flag | File | Owner |
|---|---|---|
| `dashboard` | `dashboard.launch.py` ✅ | Connor |
| `thermal` | `thermal.launch.py` | Connor |
| `detection` | `detection.launch.py` ✅ | Ethan |
| `active_mode` | `active_mode.launch.py` | Will |
| `behaviour_tree` | `behaviour_tree.launch.py` | Alec |

Every flag defaults to `True`. Turn a part off for a run with, for example,
`ros2 launch 41068_assignment demo.launch.py detection:=False`.

## Adding your part

1. Create `launch/modules/<your file>` with a normal `generate_launch_description()`.
   `dashboard.launch.py` is a working example to copy.
2. Declare and use these arguments, which the demo always passes in:
   - `namespace`: `parrot1`. Put your nodes in it so relative topics like
     `camera/image`, `odom` and `scan` resolve to the drone's topics.
   - `use_sim_time`: pass it to your nodes' `use_sim_time` parameter.
   - `world`: the world name, e.g. `simple_trees`, if your part needs per-world settings.
3. Make sure your scripts are installed. Add them to `install(PROGRAMS ...)` in
   `CMakeLists.txt`, and add imported helper modules to `install(FILES ...)`.
   This folder is installed already because it's inside `launch/`.
4. Run `colcon build --packages-select 41068_assignment`, then
   `ros2 launch 41068_assignment demo.launch.py`. The skip message for your part
   should be gone.

Only edit your own file. Ask before changing `spawn_parrot.launch.py` or
`demo.launch.py`, because everyone's part runs through them.
