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
  sensor_msgs
  tf
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   DEPENDS eigen
   CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/robot_agent.cpp
 src/jackal_agent.cpp
  src/pelican_agent.cpp
)

add_executable(jackal_node node/jackal_node.cpp)
target_link_libraries(jackal_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(pelican_node node/pelican_node.cpp)
target_link_libraries(pelican_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(report_measurement_node node/report_measurement_node.cpp)
target_link_libraries(report_measurement_node ${PROJECT_NAME} ${catkin_LIBRARIES})

