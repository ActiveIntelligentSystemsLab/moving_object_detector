cmake_minimum_required(VERSION 2.8.3)
project(velocity_estimator)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  disparity_image_proc
  dynamic_reconfigure
  image_geometry
  image_transport
  message_filters
  message_generation
  optical_flow_msgs
  pcl_conversions
  roscpp
  sensor_msgs
  std_msgs
  stereo_msgs
  tf2_eigen
  viso2_ros
)
find_package(OpenCV)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)

add_service_files(FILES InputSynchronizerPublish.srv)

generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options(
  cfg/Velocity2Color.cfg
  cfg/VelocityEstimator.cfg
)

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
  src/input_synchronizer_nodelet.cpp
  src/velocity_estimator_nodelet.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(velocity2color_node src/velocity2color_node.cpp)
add_dependencies(velocity2color_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(velocity2color_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
    velocity2color_node
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
