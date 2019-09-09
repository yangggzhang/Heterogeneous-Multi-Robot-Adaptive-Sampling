# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "jackal_msgs: 4 messages, 0 services")

set(MSG_I_FLAGS "-Ijackal_msgs:/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(jackal_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Feedback.msg" NAME_WE)
add_custom_target(_jackal_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jackal_msgs" "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Feedback.msg" "std_msgs/Header:jackal_msgs/DriveFeedback"
)

get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/DriveFeedback.msg" NAME_WE)
add_custom_target(_jackal_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jackal_msgs" "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/DriveFeedback.msg" ""
)

get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Drive.msg" NAME_WE)
add_custom_target(_jackal_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jackal_msgs" "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Drive.msg" ""
)

get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Status.msg" NAME_WE)
add_custom_target(_jackal_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jackal_msgs" "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Status.msg" "std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(jackal_msgs
  "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Feedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/DriveFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jackal_msgs
)
_generate_msg_cpp(jackal_msgs
  "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/DriveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jackal_msgs
)
_generate_msg_cpp(jackal_msgs
  "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Drive.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jackal_msgs
)
_generate_msg_cpp(jackal_msgs
  "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jackal_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(jackal_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jackal_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(jackal_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(jackal_msgs_generate_messages jackal_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Feedback.msg" NAME_WE)
add_dependencies(jackal_msgs_generate_messages_cpp _jackal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/DriveFeedback.msg" NAME_WE)
add_dependencies(jackal_msgs_generate_messages_cpp _jackal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Drive.msg" NAME_WE)
add_dependencies(jackal_msgs_generate_messages_cpp _jackal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Status.msg" NAME_WE)
add_dependencies(jackal_msgs_generate_messages_cpp _jackal_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jackal_msgs_gencpp)
add_dependencies(jackal_msgs_gencpp jackal_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jackal_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(jackal_msgs
  "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Feedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/DriveFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jackal_msgs
)
_generate_msg_lisp(jackal_msgs
  "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/DriveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jackal_msgs
)
_generate_msg_lisp(jackal_msgs
  "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Drive.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jackal_msgs
)
_generate_msg_lisp(jackal_msgs
  "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jackal_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(jackal_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jackal_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(jackal_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(jackal_msgs_generate_messages jackal_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Feedback.msg" NAME_WE)
add_dependencies(jackal_msgs_generate_messages_lisp _jackal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/DriveFeedback.msg" NAME_WE)
add_dependencies(jackal_msgs_generate_messages_lisp _jackal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Drive.msg" NAME_WE)
add_dependencies(jackal_msgs_generate_messages_lisp _jackal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Status.msg" NAME_WE)
add_dependencies(jackal_msgs_generate_messages_lisp _jackal_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jackal_msgs_genlisp)
add_dependencies(jackal_msgs_genlisp jackal_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jackal_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(jackal_msgs
  "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Feedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/DriveFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jackal_msgs
)
_generate_msg_py(jackal_msgs
  "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/DriveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jackal_msgs
)
_generate_msg_py(jackal_msgs
  "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Drive.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jackal_msgs
)
_generate_msg_py(jackal_msgs
  "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jackal_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(jackal_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jackal_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(jackal_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(jackal_msgs_generate_messages jackal_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Feedback.msg" NAME_WE)
add_dependencies(jackal_msgs_generate_messages_py _jackal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/DriveFeedback.msg" NAME_WE)
add_dependencies(jackal_msgs_generate_messages_py _jackal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Drive.msg" NAME_WE)
add_dependencies(jackal_msgs_generate_messages_py _jackal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/catkin_ws/src/jackal/jackal_msgs/msg/Status.msg" NAME_WE)
add_dependencies(jackal_msgs_generate_messages_py _jackal_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jackal_msgs_genpy)
add_dependencies(jackal_msgs_genpy jackal_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jackal_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jackal_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jackal_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(jackal_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jackal_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jackal_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(jackal_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jackal_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jackal_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jackal_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(jackal_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
