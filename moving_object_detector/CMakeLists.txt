cmake_minimum_required(VERSION 2.8.3)
project(moving_object_detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_geometry
  image_transport
  message_generation
  opencv_apps
  pcl_conversions
  sensor_msgs
  std_msgs
  stereo_msgs
  tf2
  tf2_geometry_msgs
)

find_package(OpenCV)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED common)

add_message_files(
  FILES
  MovingObject.msg
  MovingObjectArray.msg
  Size3D.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/DynamicReconfigure.cfg
  cfg/Velocity2Color.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs image_geometry image_transport message_runtime opencv_apps pcl_conversions sensor_msgs std_msgs stereo_msgs tf2 tf2_geometry_msgs
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${openCV_INCLUDE_DIRS}
  ${PCL_LIBRARY_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node 
  src/clusterer.cpp
  src/flow_3d.cpp
  src/moving_object_detector.cpp
  src/moving_object_detector_node.cpp
  src/process_disparity_image.cpp
)

add_executable(velocity2color_node 
  src/velocity2color.cpp
)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
)

target_link_libraries(velocity2color_node
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
)
