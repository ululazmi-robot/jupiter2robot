cmake_minimum_required(VERSION 3.8)
project(jupiterobot2_teleop)

# Default to C++14 or higher for ROS 2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CONTROL_STRUCTURE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

###########
## Build ##
###########

# Build the C++ Joystick Node
add_executable(joystick_teleop src/joystick_teleop.cpp)
ament_target_dependencies(joystick_teleop
  rclcpp
  geometry_msgs
  sensor_msgs
)

#############
## Install ##
#############

# 1. Install the C++ joystick executable
install(TARGETS joystick_teleop
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Install the Python keyboard script
# Note: This assumes your file is located at src/teleop_keyboard.py
install(PROGRAMS
  src/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_keyboard
)

# 3. Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_package()