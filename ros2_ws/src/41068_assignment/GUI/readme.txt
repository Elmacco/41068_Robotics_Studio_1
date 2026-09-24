TO RUN GUI:

  In Terminal: Launch parrot simulation

ros2 launch 41068_assignment spawn_parrot.launch.py rviz:=True nav2:=True

  In a new terminal

cd /home/student/subjects/41068_robotics_studio_1/ros2_ws/src/41068_assignment/GUI
python3 -m http.server 8000

  Then in browser launch:
http://localhost:8000
