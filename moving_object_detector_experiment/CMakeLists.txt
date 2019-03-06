cmake_minimum_required(VERSION 2.8.3)
project(moving_object_detector_experiment)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(FILES ExpResult.msg)
generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
)

catkin_install_python(
  PROGRAMS
    nodes/simulation_exp_result_calc
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

