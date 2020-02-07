cmake_minimum_required(VERSION 2.8.3)
project(viso2_stereo_server)

add_definitions(-msse3)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  libviso2
  message_filters
  message_generation
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(OpenCV REQUIRED)

add_service_files(FILES EstimateMotionFromStereo.srv)
generate_messages(DEPENDENCIES 
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  sensor_msgs
  std_msgs
)

option(ARM_CROSS_COMPILATION "ARM Cross Compilation" OFF)

if(ARM_CROSS_COMPILATION)
  SET(CMAKE_SYSTEM_PROCESSOR arm)
  SET(CMAKE_CXX_FLAGS -mfpu=neon)
else(ARM_CROSS_COMPILATION)
  add_definitions(-msse3)
endif(ARM_CROSS_COMPILATION)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/${PROJECT_NAME}_node.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(viso2_service_test_client
  src/viso2_service_test_client.cpp
  src/viso2_service_test_client_node.cpp
)
add_dependencies(viso2_service_test_client
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(viso2_service_test_client
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
