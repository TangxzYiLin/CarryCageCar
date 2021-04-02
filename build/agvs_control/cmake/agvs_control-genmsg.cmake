# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "agvs_control: 2 messages, 2 services")

set(MSG_I_FLAGS "-Iagvs_control:/home/iha/avg_cage-master/src/agvs_control/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(agvs_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/msg/date_realtime_feedback.msg" NAME_WE)
add_custom_target(_agvs_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "agvs_control" "/home/iha/avg_cage-master/src/agvs_control/msg/date_realtime_feedback.msg" ""
)

get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_mode.srv" NAME_WE)
add_custom_target(_agvs_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "agvs_control" "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_mode.srv" ""
)

get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_task.srv" NAME_WE)
add_custom_target(_agvs_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "agvs_control" "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_task.srv" ""
)

get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/msg/date_pads_cmd.msg" NAME_WE)
add_custom_target(_agvs_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "agvs_control" "/home/iha/avg_cage-master/src/agvs_control/msg/date_pads_cmd.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/msg/date_realtime_feedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agvs_control
)
_generate_msg_cpp(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/msg/date_pads_cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agvs_control
)

### Generating Services
_generate_srv_cpp(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_mode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agvs_control
)
_generate_srv_cpp(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_task.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agvs_control
)

### Generating Module File
_generate_module_cpp(agvs_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agvs_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(agvs_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(agvs_control_generate_messages agvs_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/msg/date_realtime_feedback.msg" NAME_WE)
add_dependencies(agvs_control_generate_messages_cpp _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_mode.srv" NAME_WE)
add_dependencies(agvs_control_generate_messages_cpp _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_task.srv" NAME_WE)
add_dependencies(agvs_control_generate_messages_cpp _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/msg/date_pads_cmd.msg" NAME_WE)
add_dependencies(agvs_control_generate_messages_cpp _agvs_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(agvs_control_gencpp)
add_dependencies(agvs_control_gencpp agvs_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS agvs_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/msg/date_realtime_feedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/agvs_control
)
_generate_msg_eus(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/msg/date_pads_cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/agvs_control
)

### Generating Services
_generate_srv_eus(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_mode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/agvs_control
)
_generate_srv_eus(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_task.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/agvs_control
)

### Generating Module File
_generate_module_eus(agvs_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/agvs_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(agvs_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(agvs_control_generate_messages agvs_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/msg/date_realtime_feedback.msg" NAME_WE)
add_dependencies(agvs_control_generate_messages_eus _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_mode.srv" NAME_WE)
add_dependencies(agvs_control_generate_messages_eus _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_task.srv" NAME_WE)
add_dependencies(agvs_control_generate_messages_eus _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/msg/date_pads_cmd.msg" NAME_WE)
add_dependencies(agvs_control_generate_messages_eus _agvs_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(agvs_control_geneus)
add_dependencies(agvs_control_geneus agvs_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS agvs_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/msg/date_realtime_feedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agvs_control
)
_generate_msg_lisp(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/msg/date_pads_cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agvs_control
)

### Generating Services
_generate_srv_lisp(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_mode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agvs_control
)
_generate_srv_lisp(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_task.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agvs_control
)

### Generating Module File
_generate_module_lisp(agvs_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agvs_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(agvs_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(agvs_control_generate_messages agvs_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/msg/date_realtime_feedback.msg" NAME_WE)
add_dependencies(agvs_control_generate_messages_lisp _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_mode.srv" NAME_WE)
add_dependencies(agvs_control_generate_messages_lisp _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_task.srv" NAME_WE)
add_dependencies(agvs_control_generate_messages_lisp _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/msg/date_pads_cmd.msg" NAME_WE)
add_dependencies(agvs_control_generate_messages_lisp _agvs_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(agvs_control_genlisp)
add_dependencies(agvs_control_genlisp agvs_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS agvs_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/msg/date_realtime_feedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/agvs_control
)
_generate_msg_nodejs(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/msg/date_pads_cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/agvs_control
)

### Generating Services
_generate_srv_nodejs(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_mode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/agvs_control
)
_generate_srv_nodejs(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_task.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/agvs_control
)

### Generating Module File
_generate_module_nodejs(agvs_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/agvs_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(agvs_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(agvs_control_generate_messages agvs_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/msg/date_realtime_feedback.msg" NAME_WE)
add_dependencies(agvs_control_generate_messages_nodejs _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_mode.srv" NAME_WE)
add_dependencies(agvs_control_generate_messages_nodejs _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_task.srv" NAME_WE)
add_dependencies(agvs_control_generate_messages_nodejs _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/msg/date_pads_cmd.msg" NAME_WE)
add_dependencies(agvs_control_generate_messages_nodejs _agvs_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(agvs_control_gennodejs)
add_dependencies(agvs_control_gennodejs agvs_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS agvs_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/msg/date_realtime_feedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agvs_control
)
_generate_msg_py(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/msg/date_pads_cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agvs_control
)

### Generating Services
_generate_srv_py(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_mode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agvs_control
)
_generate_srv_py(agvs_control
  "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_task.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agvs_control
)

### Generating Module File
_generate_module_py(agvs_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agvs_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(agvs_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(agvs_control_generate_messages agvs_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/msg/date_realtime_feedback.msg" NAME_WE)
add_dependencies(agvs_control_generate_messages_py _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_mode.srv" NAME_WE)
add_dependencies(agvs_control_generate_messages_py _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/srv/cmd_control_task.srv" NAME_WE)
add_dependencies(agvs_control_generate_messages_py _agvs_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/iha/avg_cage-master/src/agvs_control/msg/date_pads_cmd.msg" NAME_WE)
add_dependencies(agvs_control_generate_messages_py _agvs_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(agvs_control_genpy)
add_dependencies(agvs_control_genpy agvs_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS agvs_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agvs_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agvs_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(agvs_control_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/agvs_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/agvs_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(agvs_control_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agvs_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agvs_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(agvs_control_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/agvs_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/agvs_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(agvs_control_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agvs_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agvs_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agvs_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(agvs_control_generate_messages_py std_msgs_generate_messages_py)
endif()
