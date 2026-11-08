cmake_minimum_required(VERSION 3.8)
project(jupiterobot2_voice)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CONTROL_STRUCTURE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# --- INSTALLATION SECTION ---

# 1. Install the Python script
install(PROGRAMS
  scripts/voice_node.py
  scripts/tts_node.py
  scripts/stt_node.py
  scripts/backgroud_stt.py
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Install launch files
# Git cannot store empty directories, so 'launch/' only exists on a fresh
# clone if it actually holds a launch file. Guard it instead of hard-failing.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
else()
  message(STATUS "jupiterobot2_voice: no launch/ directory, skipping launch install")
endif()

ament_package()