cmake_minimum_required(VERSION 2.8.3)
project(velocity_pc_clusterer)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  moving_object_msgs
  pcl_conversions
  roscpp
  sensor_msgs
  velocity_estimator
  visualization_msgs
)
find_package(OpenCV)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED
  common
  segmentation
)

generate_dynamic_reconfigure_options(cfg/Clusterer.cfg)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(clusterer_node
  src/clusterer.cpp
  src/clusterer_node.cpp
  src/color_set.cpp
  src/lookup_table.cpp
)
add_dependencies(clusterer_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(clusterer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_SEGMENTATION_LIBRARIES}
)

install(
  TARGETS clusterer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
