cmake_minimum_required(VERSION 3.0.2)
project(sampling_modeling)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sampling_msgs
  geometry_msgs
  std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

