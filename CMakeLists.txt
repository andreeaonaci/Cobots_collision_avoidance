cmake_minimum_required(VERSION 3.8)
project(ur5e_collision_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ──────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ── Install Python package ─────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

# ── Install Python nodes ───────────────────────────────────────────
install(PROGRAMS
  ur5e_collision_avoidance/scripts/collision_monitor.py
  ur5e_collision_avoidance/scripts/robot1_controller.py
  ur5e_collision_avoidance/scripts/robot2_controller.py
  ur5e_collision_avoidance/scripts/scenario_orchestrator.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install other resources ────────────────────────────────────────
install(DIRECTORY
  launch
  config
  worlds
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# ── Optional: Tests ───────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
