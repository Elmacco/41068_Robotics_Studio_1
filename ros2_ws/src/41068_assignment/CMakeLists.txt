cmake_minimum_required(VERSION 3.8)
project(41068_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

install(
  DIRECTORY launch worlds config urdf_parrot models
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY GUI
  DESTINATION share/${PROJECT_NAME}
)

# YOLO weights for target_detector.py, so the demo works offline
install(
  DIRECTORY weights
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/waypoint_follower.py scripts/publish_waypoints.py
            scripts/web_dashboard_bridge.py scripts/flight_controller.py
            scripts/terrain_mapper.py scripts/mission_manager.py
            scripts/target_detector.py scripts/target_projector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Library modules imported by the nodes above (not executables themselves)
install(
  FILES scripts/mission_waypoints.py scripts/grid_tools.py scripts/terrain_map.py
        scripts/trail_detector.py scripts/route_planner.py scripts/coverage_explorer.py
        scripts/node_params.py scripts/mapping_mode.py scripts/mission_modes.py
        scripts/active_mode.py scripts/colour_classifier.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
