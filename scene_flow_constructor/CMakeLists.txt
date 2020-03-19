cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(scene_flow_constructor CXX)

add_definitions(-msse3)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  disparity_image_proc
  dynamic_reconfigure
  image_geometry
  image_transport
  libviso2
  message_filters
  message_generation
  pcl_conversions
  pwc_net
  roscpp
  sensor_msgs
  sgm_gpu
  std_msgs
  stereo_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(cfg/SceneFlowConstructor.cfg)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/${PROJECT_NAME}_node.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

