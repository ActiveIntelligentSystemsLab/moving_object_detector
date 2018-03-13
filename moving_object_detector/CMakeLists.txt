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
  pcl_conversions
  sensor_msgs
  std_msgs
  stereo_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
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
  cfg/Clusterer.cfg
  cfg/MovingObjectDetector.cfg
  cfg/Velocity2Color.cfg
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    image_geometry
    image_transport
    message_runtime 
    pcl_conversions 
    sensor_msgs 
    std_msgs 
    stereo_msgs 
    tf2 
    tf2_geometry_msgs
    visualization_msgs
  DEPENDS
    OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${openCV_INCLUDE_DIRS}
  ${PCL_LIBRARY_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

set(CLUSTERER_NODE clusterer_node)
add_executable(${CLUSTERER_NODE} 
  src/clusterer.cpp
  src/${CLUSTERER_NODE}.cpp
)
add_dependencies(${CLUSTERER_NODE} ${PROJECT_NAME}_gencfg)
target_link_libraries(${CLUSTERER_NODE}
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
)

set(DISPLAY_MOVING_OBJECTS_NODE display_moving_objects_node)
add_executable(${DISPLAY_MOVING_OBJECTS_NODE} src/${DISPLAY_MOVING_OBJECTS_NODE}.cpp)
target_link_libraries(${DISPLAY_MOVING_OBJECTS_NODE}
  ${catkin_LIBRARIES}
)

set(MOVING_OBJECT_DETECTOR_NODE moving_object_detector_node)
add_executable(${MOVING_OBJECT_DETECTOR_NODE} 
  src/flow_3d.cpp
  src/moving_object_detector.cpp
  src/moving_object_detector_node.cpp
  src/process_disparity_image.cpp
)
add_dependencies(${MOVING_OBJECT_DETECTOR_NODE} ${PROJECT_NAME}_gencfg)
target_link_libraries(${MOVING_OBJECT_DETECTOR_NODE}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
)

set(VELOCITY2COLOR_NODE velocity2color_node)
add_executable(${VELOCITY2COLOR_NODE} src/${VELOCITY2COLOR_NODE}.cpp)
add_dependencies(${VELOCITY2COLOR_NODE} ${PROJECT_NAME}_gencfg)
target_link_libraries(${VELOCITY2COLOR_NODE}
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
)
