cmake_minimum_required(VERSION 2.8.3)
project(sampling_core)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmuswarm_core
  cmuswarm_msgs
  cmuswarm_swarm
  roscpp
  rospy
  std_msgs
  visualization_msgs
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  SamplingGoal.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES cmu_swarm
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include
  libgp/include
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/gmm_utils.cpp
  src/utils.cpp
  src/sampling_visualization.cpp
)

add_executable(centralized_sampling_node node/centralized_sampling_node.cpp)
target_link_libraries(centralized_sampling_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/libgp/build/libgp.a)

add_executable(jackal_node node/jackal_node.cpp)
target_link_libraries(jackal_node ${PROJECT_NAME} ${catkin_LIBRARIES})
