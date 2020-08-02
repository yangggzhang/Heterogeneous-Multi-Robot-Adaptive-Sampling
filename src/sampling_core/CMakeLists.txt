cmake_minimum_required(VERSION 2.8.3)
project(sampling_core)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  actionlib
  actionlib_msgs
  message_runtime
  sampling_measurement
  sampling_utils
  sampling_msgs
  sampling_agent
  sampling_partition
  sampling_online_learning
  sampling_visualization
  roslib
  std_srvs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
   INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
   CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME}
  src/sampling_core_params.cpp
  src/sampling_core.cpp
  src/sampling_core_performance_evaluation.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(heterogeneous_adaptive_sampling_node node/heterogeneous_adaptive_sampling_node.cpp)
target_link_libraries(heterogeneous_adaptive_sampling_node ${PROJECT_NAME} ${catkin_LIBRARIES} )