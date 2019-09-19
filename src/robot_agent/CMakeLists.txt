cmake_minimum_required(VERSION 2.8.3)
project(robot_agent)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  actionlib
  actionlib_msgs
  message_runtime
  temperature_measurement
  sampling_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES cmu_swarm
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/robot_agent.cpp
)