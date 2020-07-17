cmake_minimum_required(VERSION 3.0.2)
project(sampling_modeling)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sampling_msgs
  geometry_msgs
  std_srvs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

