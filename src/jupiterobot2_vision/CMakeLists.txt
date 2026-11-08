cmake_minimum_required(VERSION 3.8)
project(jupiterobot2_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install your python scripts
install(PROGRAMS
  scripts/yolo_node.py
  scripts/mediapipe_node.py
  scripts/experiment3.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()