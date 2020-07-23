cmake_minimum_required(VERSION 2.8.3)
project(sampling_core)

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
  sampling_measurement
  sampling_utils
  sampling_msgs
  sampling_partition
  sampling_online_learning
  sampling_visualization
  roslib
  std_srvs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

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
  ${Boost_INCLUDE_DIRS}
  include
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/sampling_core_params.cpp
  src/sampling_core.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(centralized_sampling_node node/centralized_sampling_node.cpp)
# target_link_libraries(centralized_sampling_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/libgp/build/libgp.a)

# add_executable(gmm_test_node node/gmm_test_node.cpp)
# target_link_libraries(gmm_test_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/libgp/build/libgp.a)

# add_executable(voronoi_test_node node/voronoi_test_node.cpp)
# target_link_libraries(voronoi_test_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/libgp/build/libgp.a)

# add_executable(sampling_simulation_node node/sampling_simulation_node.cpp)
# target_link_libraries(sampling_simulation_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/libgp/build/libgp.a)

