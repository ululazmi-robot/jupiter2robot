cmake_minimum_required(VERSION 3.8)
project(vip_single)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

install(PROGRAMS
  scripts/vip_manager_simulation.py
  scripts/vip_manager_hardware.py
  scripts/vip_manager_hardware_spare.py
  scripts/vip_manager_hardware_demo.py
  scripts/vip_manager_hardware_move.py
  scripts/vip_manager_hardware_basic_move.py
  scripts/vip_manager_sound_player_node.py
  scripts/demo_fcoee_1.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files and sounds (both optional: git cannot store empty directories,
# and the two DESTINATION lines in the original were invalid CMake anyway)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
else()
  message(STATUS "vip_single: no launch/ directory, skipping launch install")
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sounds")
  install(DIRECTORY sounds
    DESTINATION share/${PROJECT_NAME}
  )
else()
  message(STATUS "vip_single: no sounds/ directory, skipping sounds install")
endif()

ament_package()