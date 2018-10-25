# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hdl_graph_slam_reconstruction: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ihdl_graph_slam_reconstruction:/home/xinwei4/xinwei_ws/src/hdl_graph_slam_reconstruction/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hdl_graph_slam_reconstruction_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xinwei4/xinwei_ws/src/hdl_graph_slam_reconstruction/msg/FloorCoeffs1.msg" NAME_WE)
add_custom_target(_hdl_graph_slam_reconstruction_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hdl_graph_slam_reconstruction" "/home/xinwei4/xinwei_ws/src/hdl_graph_slam_reconstruction/msg/FloorCoeffs1.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hdl_graph_slam_reconstruction
  "/home/xinwei4/xinwei_ws/src/hdl_graph_slam_reconstruction/msg/FloorCoeffs1.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hdl_graph_slam_reconstruction
)

### Generating Services

### Generating Module File
_generate_module_cpp(hdl_graph_slam_reconstruction
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hdl_graph_slam_reconstruction
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hdl_graph_slam_reconstruction_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hdl_graph_slam_reconstruction_generate_messages hdl_graph_slam_reconstruction_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xinwei4/xinwei_ws/src/hdl_graph_slam_reconstruction/msg/FloorCoeffs1.msg" NAME_WE)
add_dependencies(hdl_graph_slam_reconstruction_generate_messages_cpp _hdl_graph_slam_reconstruction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hdl_graph_slam_reconstruction_gencpp)
add_dependencies(hdl_graph_slam_reconstruction_gencpp hdl_graph_slam_reconstruction_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hdl_graph_slam_reconstruction_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(hdl_graph_slam_reconstruction
  "/home/xinwei4/xinwei_ws/src/hdl_graph_slam_reconstruction/msg/FloorCoeffs1.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hdl_graph_slam_reconstruction
)

### Generating Services

### Generating Module File
_generate_module_eus(hdl_graph_slam_reconstruction
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hdl_graph_slam_reconstruction
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hdl_graph_slam_reconstruction_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hdl_graph_slam_reconstruction_generate_messages hdl_graph_slam_reconstruction_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xinwei4/xinwei_ws/src/hdl_graph_slam_reconstruction/msg/FloorCoeffs1.msg" NAME_WE)
add_dependencies(hdl_graph_slam_reconstruction_generate_messages_eus _hdl_graph_slam_reconstruction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hdl_graph_slam_reconstruction_geneus)
add_dependencies(hdl_graph_slam_reconstruction_geneus hdl_graph_slam_reconstruction_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hdl_graph_slam_reconstruction_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hdl_graph_slam_reconstruction
  "/home/xinwei4/xinwei_ws/src/hdl_graph_slam_reconstruction/msg/FloorCoeffs1.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hdl_graph_slam_reconstruction
)

### Generating Services

### Generating Module File
_generate_module_lisp(hdl_graph_slam_reconstruction
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hdl_graph_slam_reconstruction
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hdl_graph_slam_reconstruction_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hdl_graph_slam_reconstruction_generate_messages hdl_graph_slam_reconstruction_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xinwei4/xinwei_ws/src/hdl_graph_slam_reconstruction/msg/FloorCoeffs1.msg" NAME_WE)
add_dependencies(hdl_graph_slam_reconstruction_generate_messages_lisp _hdl_graph_slam_reconstruction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hdl_graph_slam_reconstruction_genlisp)
add_dependencies(hdl_graph_slam_reconstruction_genlisp hdl_graph_slam_reconstruction_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hdl_graph_slam_reconstruction_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(hdl_graph_slam_reconstruction
  "/home/xinwei4/xinwei_ws/src/hdl_graph_slam_reconstruction/msg/FloorCoeffs1.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hdl_graph_slam_reconstruction
)

### Generating Services

### Generating Module File
_generate_module_nodejs(hdl_graph_slam_reconstruction
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hdl_graph_slam_reconstruction
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hdl_graph_slam_reconstruction_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hdl_graph_slam_reconstruction_generate_messages hdl_graph_slam_reconstruction_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xinwei4/xinwei_ws/src/hdl_graph_slam_reconstruction/msg/FloorCoeffs1.msg" NAME_WE)
add_dependencies(hdl_graph_slam_reconstruction_generate_messages_nodejs _hdl_graph_slam_reconstruction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hdl_graph_slam_reconstruction_gennodejs)
add_dependencies(hdl_graph_slam_reconstruction_gennodejs hdl_graph_slam_reconstruction_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hdl_graph_slam_reconstruction_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hdl_graph_slam_reconstruction
  "/home/xinwei4/xinwei_ws/src/hdl_graph_slam_reconstruction/msg/FloorCoeffs1.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hdl_graph_slam_reconstruction
)

### Generating Services

### Generating Module File
_generate_module_py(hdl_graph_slam_reconstruction
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hdl_graph_slam_reconstruction
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hdl_graph_slam_reconstruction_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hdl_graph_slam_reconstruction_generate_messages hdl_graph_slam_reconstruction_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xinwei4/xinwei_ws/src/hdl_graph_slam_reconstruction/msg/FloorCoeffs1.msg" NAME_WE)
add_dependencies(hdl_graph_slam_reconstruction_generate_messages_py _hdl_graph_slam_reconstruction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hdl_graph_slam_reconstruction_genpy)
add_dependencies(hdl_graph_slam_reconstruction_genpy hdl_graph_slam_reconstruction_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hdl_graph_slam_reconstruction_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hdl_graph_slam_reconstruction)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hdl_graph_slam_reconstruction
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(hdl_graph_slam_reconstruction_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hdl_graph_slam_reconstruction)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hdl_graph_slam_reconstruction
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(hdl_graph_slam_reconstruction_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hdl_graph_slam_reconstruction)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hdl_graph_slam_reconstruction
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(hdl_graph_slam_reconstruction_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hdl_graph_slam_reconstruction)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hdl_graph_slam_reconstruction
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(hdl_graph_slam_reconstruction_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hdl_graph_slam_reconstruction)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hdl_graph_slam_reconstruction\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hdl_graph_slam_reconstruction
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(hdl_graph_slam_reconstruction_generate_messages_py std_msgs_generate_messages_py)
endif()
