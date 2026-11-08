cmake_minimum_required(VERSION 3.8)
project(jupiterobot2_gazebo)

################################################################################
# Compile options
################################################################################
add_compile_options(-std=c++17)

################################################################################
# Find dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_ros REQUIRED)

################################################################################
# Install launch, models, rviz
################################################################################
# Only install optional directories that actually exist in this checkout.
set(_gz_share_dirs "")
foreach(_d launch models rviz worlds config gazebo_building maps urdf)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_d}")
    list(APPEND _gz_share_dirs ${_d})
  else()
    message(STATUS "jupiterobot2_gazebo: no ${_d}/ directory, skipping")
  endif()
endforeach()

install(
  DIRECTORY ${_gz_share_dirs}
  DESTINATION share/${PROJECT_NAME}
)

# model.config at the package root is used by the models/ tree
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/model.config")
  install(FILES model.config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

################################################################################
# Install custom teleop script (Executables go to LIB)
################################################################################
install(PROGRAMS
  scripts/custom_teleop.py
  scripts/preflight_check.py
  scripts/restore_rest.py
  DESTINATION lib/${PROJECT_NAME}
)
################################################################################
# Export package
################################################################################
ament_package()

