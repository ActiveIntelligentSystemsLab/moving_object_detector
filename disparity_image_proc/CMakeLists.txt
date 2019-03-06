cmake_minimum_required(VERSION 2.8.3)
project(disparity_image_proc)

find_package(catkin REQUIRED COMPONENTS
  image_geometry
  sensor_msgs
  stereo_msgs
  tf2
)
find_package(OpenCV)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    image_geometry
    sensor_msgs
    stereo_msgs
  DEPENDS
    OpenCV
    PCL
  INCLUDE_DIRS
    include
  LIBRARIES
    libdisparity_image_processor
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(libdisparity_image_processor src/disparity_image_processor.cpp)
target_link_libraries(libdisparity_image_processor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS libdisparity_image_processor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
