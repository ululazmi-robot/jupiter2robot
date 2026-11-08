cmake_minimum_required(VERSION 3.8)
project(jupiterobot2_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the ROS 2 build system
find_package(ament_cmake REQUIRED)

# Install the launch folder so ROS 2 can find your Python scripts
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()