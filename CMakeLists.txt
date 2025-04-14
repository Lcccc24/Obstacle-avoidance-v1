cmake_minimum_required(VERSION 3.0.2)
project(dis2fcu)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  pcl_ros
  mavros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(Histogram src/histogram.cpp)
add_library(Common src/common.cpp)
add_library(Transform_buffer src/transform_buffer.cpp)

add_executable(dis2fcu_node src/Distance_to_fcu.cpp)

target_link_libraries(dis2fcu_node
  ${catkin_LIBRARIES}
  Histogram
  Common
  Transform_buffer
)

