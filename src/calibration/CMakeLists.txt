cmake_minimum_required(VERSION 2.8.3)
project(calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  Eigen3 REQUIRED
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_filters
  std_srvs
  roslib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES calibration
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(GPS_calibration_node node/GPS_calibration.cpp)
target_link_libraries(GPS_calibration_node ${catkin_LIBRARIES} )

add_executable(GPS_recording_node node/GPS_recording.cpp)
target_link_libraries(GPS_recording_node ${catkin_LIBRARIES} )

add_executable(heading_calibration_node node/heading_calibration.cpp)
target_link_libraries(heading_calibration_node ${catkin_LIBRARIES} )

add_executable(gps_range_node node/GPS_range.cpp)
target_link_libraries(gps_range_node ${catkin_LIBRARIES} )
