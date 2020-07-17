cmake_minimum_required(VERSION 3.0.2)
project(sampling_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)