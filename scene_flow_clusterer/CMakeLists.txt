cmake_minimum_required(VERSION 2.8.3)
project(scene_flow_clusterer)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  moving_object_msgs
  nodelet
  pcl_conversions
  roscpp
  sensor_msgs
  scene_flow_constructor
  visualization_msgs
)
find_package(OpenCV)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED
  common
  segmentation
)

generate_dynamic_reconfigure_options(cfg/Clusterer.cfg)

catkin_package(LIBRARIES ${PROJECT_NAME})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/clusterer_nodelet.cpp
  src/color_set.cpp
  src/lookup_table.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_SEGMENTATION_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_executable(${PROJECT_NAME}_node src/clusterer_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
