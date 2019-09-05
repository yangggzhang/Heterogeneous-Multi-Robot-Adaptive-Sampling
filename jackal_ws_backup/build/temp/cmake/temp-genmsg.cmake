# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "temp: 1 messages, 0 services")

set(MSG_I_FLAGS "-Itemp:/home/administrator/catkin_ws/src/temp/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(temp_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/administrator/catkin_ws/src/temp/msg/temp_odom.msg" NAME_WE)
add_custom_target(_temp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "temp" "/home/administrator/catkin_ws/src/temp/msg/temp_odom.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(temp
  "/home/administrator/catkin_ws/src/temp/msg/temp_odom.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/temp
)

### Generating Services

### Generating Module File
_generate_module_cpp(temp
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/temp
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(temp_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(temp_generate_messages temp_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/catkin_ws/src/temp/msg/temp_odom.msg" NAME_WE)
add_dependencies(temp_generate_messages_cpp _temp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(temp_gencpp)
add_dependencies(temp_gencpp temp_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS temp_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(temp
  "/home/administrator/catkin_ws/src/temp/msg/temp_odom.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/temp
)

### Generating Services

### Generating Module File
_generate_module_lisp(temp
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/temp
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(temp_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(temp_generate_messages temp_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/catkin_ws/src/temp/msg/temp_odom.msg" NAME_WE)
add_dependencies(temp_generate_messages_lisp _temp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(temp_genlisp)
add_dependencies(temp_genlisp temp_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS temp_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(temp
  "/home/administrator/catkin_ws/src/temp/msg/temp_odom.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/temp
)

### Generating Services

### Generating Module File
_generate_module_py(temp
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/temp
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(temp_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(temp_generate_messages temp_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/catkin_ws/src/temp/msg/temp_odom.msg" NAME_WE)
add_dependencies(temp_generate_messages_py _temp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(temp_genpy)
add_dependencies(temp_genpy temp_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS temp_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/temp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/temp
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(temp_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/temp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/temp
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(temp_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/temp)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/temp\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/temp
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(temp_generate_messages_py std_msgs_generate_messages_py)
endif()
