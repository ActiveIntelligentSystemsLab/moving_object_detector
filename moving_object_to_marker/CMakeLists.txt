cmake_minimum_required(VERSION 2.8.3)
project(moving_object_to_marker)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(
  PROGRAMS 
    nodes/moving_object_to_marker
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)