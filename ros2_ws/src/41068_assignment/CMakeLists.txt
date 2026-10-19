cmake_minimum_required(VERSION 3.8)
project(41068_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

install(
  DIRECTORY launch worlds config urdf_parrot
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY GUI
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/waypoint_follower.py scripts/publish_waypoints.py
            scripts/web_dashboard_bridge.py scripts/flight_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES scripts/mission_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
