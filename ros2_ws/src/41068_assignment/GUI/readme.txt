Terminal 1:

source /opt/ros/humble/setup.bash
cd /home/student/subjects/41068_robotics_studio_1/ros2_ws
source install/setup.bash
ros2 launch 41068_assignment spawn_parrot.launch.py \
  world_package:=41068_ignition_bringup world:=simple_trees \
  gui:=True rviz:=False nav2:=True web_dashboard:=True
  


Terminal 2:

source /opt/ros/humble/setup.bash
source /home/student/subjects/41068_robotics_studio_1/ros2_ws/install/setup.bash
cd /home/student/subjects/41068_robotics_studio_1/ros2_ws/src/41068_assignment/GUI
python3 -m http.server 8000


In Browser
http://10.0.0.40:8000/
