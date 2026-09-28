TO RUN GUI:

  	Start drone simulation

source /opt/ros/humble/setup.bash
cd /home/student/subjects/41068_robotics_studio_1/ros2_ws
source install/setup.bash
ros2 launch 41068_assignment spawn_parrot.launch.py rviz:=True nav2:=True


  	In a new terminal start ROS bridge
  
 source /opt/ros/humble/setup.bash
cd /home/student/subjects/41068_robotics_studio_1/ros2_ws
source install/setup.bash
ros2 run 41068_assignment web_dashboard_bridge.py


	New terminal - Serve GUI

cd /home/student/subjects/41068_robotics_studio_1/ros2_ws/src/41068_assignment/GUI
python3 -m http.server 8000


  	Then in browser launch:
http://localhost:8000
