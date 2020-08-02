cmake_minimum_required(VERSION 2.8.3)
project(sampling_measurement)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_runtime
  sampling_msgs
)

catkin_package(
   CATKIN_DEPENDS rospy sampling_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)