cmake_minimum_required(VERSION 2.8.3)
project(scene_flow_constructor)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  disparity_srv
  disparity_image_proc
  dynamic_reconfigure
  image_geometry
  image_transport
  message_filters
  message_generation
  nodelet
  optical_flow_msgs
  optical_flow_srvs
  pcl_conversions
  roscpp
  sensor_msgs
  std_msgs
  stereo_msgs
  tf2_eigen
  viso2_stereo_server
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(cfg/SceneFlowConstructor.cfg)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
  DEPENDS
    PCL
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}_nodelet.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
