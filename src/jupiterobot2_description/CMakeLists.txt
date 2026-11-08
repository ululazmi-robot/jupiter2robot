cmake_minimum_required(VERSION 3.8)
project(jupiterobot2_description)

find_package(ament_cmake REQUIRED)

# Install only the directories that exist. The panel opens share/rviz/
# jupiter_access.rviz by absolute path, so rviz/ MUST be installed -- it was
# previously missing from this list, which made the panel's 'v' key fail with
# "rviz config missing" even though the file was in the source tree.
# Install the urdf/ tree including its subdirectories. install(DIRECTORY) is
# recursive, so config/, robot/, tower/, sensors/ and the gazebo package's
# plugins/ + sensors/ all land under share/<pkg>/urdf/ with their structure
# intact. Xacro resolves $(find pkg)/urdf/config/properties.xacro by path, so
# losing that structure breaks every include.
foreach(_dir urdf meshes launch rviz models)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}")
    install(DIRECTORY ${_dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
