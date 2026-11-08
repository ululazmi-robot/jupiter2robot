cmake_minimum_required(VERSION 3.8)
project(so100_description)

find_package(ament_cmake REQUIRED)

# Same install-the-directory-if-it-exists pattern as jupiterobot2_description,
# so adding/removing folders never breaks the build.
foreach(_dir urdf meshes rviz launch)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}")
    install(DIRECTORY ${_dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

# The safety guard is a plain Python module used by tooling, not a ROS node.
ament_python_install_module(so100_description/safety.py)

ament_package()
