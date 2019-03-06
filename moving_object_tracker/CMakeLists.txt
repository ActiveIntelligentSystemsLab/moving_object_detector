cmake_minimum_required(VERSION 2.8.3)
project(moving_object_tracker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kkl
  message_generation
  moving_object_msgs
  roscpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)
find_package(Eigen3 REQUIRED NO_MODULE)

add_message_files(
  FILES
    TrackerCovariance.msg
    TrackerCovarianceArray.msg
)

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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(moving_objects_tracker_node
  src/moving_objects_tracker.cpp
  src/moving_objects_tracker_node.cpp
)
add_dependencies(moving_objects_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(moving_objects_tracker_node
  ${catkin_LIBRARIES}
)
