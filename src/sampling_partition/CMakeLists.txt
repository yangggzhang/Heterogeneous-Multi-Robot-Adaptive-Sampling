cmake_minimum_required(VERSION 3.0.2)
project(sampling_partition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  sampling_msgs
  sampling_utils
  sampling_visualization
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES sampling_partition
 DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/heterogeneity.cpp
  src/heterogeneity_distance.cpp
  src/heterogeneity_distance_dependent.cpp
  src/heterogeneity_topography_dependent.cpp
  src/weighted_voronoi_partition.cpp
  src/weighted_voronoi_partition_params.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(partition_node node/partition_node.cpp)
target_link_libraries(partition_node ${PROJECT_NAME} ${catkin_LIBRARIES} )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
