cmake_minimum_required(VERSION 2.8.3)
project(moving_object_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  kkl
  message_generation
  moving_object_msgs
  pcl_conversions
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  velocity_estimator
)

find_package(OpenCV)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED
  common
  filters
  search
  segmentation
)

add_message_files(
  FILES
  ExpResult.msg
  TrackerCovariance.msg
  TrackerCovarianceArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

generate_dynamic_reconfigure_options(cfg/Clusterer.cfg)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime 
    std_msgs 
)
# To use gprof
# set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(clusterer_node 
  src/clusterer.cpp
  src/clusterer_node.cpp
  src/color_set.cpp
  src/lookup_table.cpp
)
add_dependencies(clusterer_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(clusterer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_SEARCH_LIBRARIES}
  ${PCL_SEGMENTATION_LIBRARIES}
)
# To use gprof
# target_compile_options(clusterer_node
#   PUBLIC -pg
# )

add_executable(display_moving_objects_2d_node src/display_moving_objects_2d_node.cpp)
add_dependencies(display_moving_objects_2d_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(display_moving_objects_2d_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(moving_objects_tracker_node
  src/moving_objects_tracker.cpp
  src/moving_objects_tracker_node.cpp
)
add_dependencies(moving_objects_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(moving_objects_tracker_node
  ${catkin_LIBRARIES}
)

catkin_install_python(
  PROGRAMS 
    nodes/simulation_exp_result_calc
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
