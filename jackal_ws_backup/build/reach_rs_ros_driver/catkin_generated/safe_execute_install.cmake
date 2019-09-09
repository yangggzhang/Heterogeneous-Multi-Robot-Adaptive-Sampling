execute_process(COMMAND "/home/administrator/catkin_ws/build/reach_rs_ros_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/administrator/catkin_ws/build/reach_rs_ros_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
