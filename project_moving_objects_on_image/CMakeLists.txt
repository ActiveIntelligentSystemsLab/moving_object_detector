cmake_minimum_required(VERSION 2.8.3)
project(project_moving_objects_on_image)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  message_filters
  moving_object_msgs
  roscpp
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(project_moving_objects_on_image src/project_moving_objects_on_image.cpp)
target_link_libraries(project_moving_objects_on_image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
