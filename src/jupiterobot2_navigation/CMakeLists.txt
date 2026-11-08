cmake_minimum_required(VERSION 3.8)
project(jupiterobot2_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CONTROL_STRUCTURE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# --- INSTALLATION SECTION ---

# 1. Install the Python script
install(PROGRAMS
  scripts/single_point.py
  scripts/multiple_points.py
  scripts/trial.py
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Install launch files (if you create any later)
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()