cmake_minimum_required(VERSION 2.8.3)
project(sampling_agent)

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
  sampling_msgs
  sensor_msgs
  tf
  tf2
  tf2_ros
  hector_navigation
  hector_navigation_msgs
  hector_uav_msgs
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
   CATKIN_DEPENDS message_runtime 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/sampling_agent.cpp
  src/sampling_agent_params.cpp
  src/jackal_agent.cpp
  src/jackal_agent_params.cpp
  src/pelican_agent.cpp
  src/pelican_agent_params.cpp
  src/hector_agent.cpp
  src/hector_agent_params.cpp
)

add_executable(sampling_agent_node node/sampling_agent_node.cpp)
target_link_libraries(sampling_agent_node ${PROJECT_NAME} ${catkin_LIBRARIES} )

install(TARGETS ${PROJECT_NAME} sampling_agent_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
