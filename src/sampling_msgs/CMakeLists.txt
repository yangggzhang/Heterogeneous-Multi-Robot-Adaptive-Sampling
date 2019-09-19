cmake_minimum_required(VERSION 2.8.3)
project(sampling_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  measurement.msg
)

add_service_files(
  FILES
  RequestGoal.srv
  RequestTemperatureMeasurement.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS std_msgs message_runtime
)