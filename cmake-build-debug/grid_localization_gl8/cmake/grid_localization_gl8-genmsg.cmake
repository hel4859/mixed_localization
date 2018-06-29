# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "grid_localization_gl8: 3 messages, 0 services")

set(MSG_I_FLAGS "-Igrid_localization_gl8:/home/hl/catkin_ws/src/grid_localization_gl8/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(grid_localization_gl8_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleSpeedFeedBack.msg" NAME_WE)
add_custom_target(_grid_localization_gl8_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "grid_localization_gl8" "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleSpeedFeedBack.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleIMU.msg" NAME_WE)
add_custom_target(_grid_localization_gl8_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "grid_localization_gl8" "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleIMU.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/GPTRA_MSG.msg" NAME_WE)
add_custom_target(_grid_localization_gl8_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "grid_localization_gl8" "/home/hl/catkin_ws/src/grid_localization_gl8/msg/GPTRA_MSG.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(grid_localization_gl8
  "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleSpeedFeedBack.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grid_localization_gl8
)
_generate_msg_cpp(grid_localization_gl8
  "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleIMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grid_localization_gl8
)
_generate_msg_cpp(grid_localization_gl8
  "/home/hl/catkin_ws/src/grid_localization_gl8/msg/GPTRA_MSG.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grid_localization_gl8
)

### Generating Services

### Generating Module File
_generate_module_cpp(grid_localization_gl8
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grid_localization_gl8
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(grid_localization_gl8_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(grid_localization_gl8_generate_messages grid_localization_gl8_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleSpeedFeedBack.msg" NAME_WE)
add_dependencies(grid_localization_gl8_generate_messages_cpp _grid_localization_gl8_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleIMU.msg" NAME_WE)
add_dependencies(grid_localization_gl8_generate_messages_cpp _grid_localization_gl8_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/GPTRA_MSG.msg" NAME_WE)
add_dependencies(grid_localization_gl8_generate_messages_cpp _grid_localization_gl8_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(grid_localization_gl8_gencpp)
add_dependencies(grid_localization_gl8_gencpp grid_localization_gl8_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS grid_localization_gl8_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(grid_localization_gl8
  "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleSpeedFeedBack.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grid_localization_gl8
)
_generate_msg_eus(grid_localization_gl8
  "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleIMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grid_localization_gl8
)
_generate_msg_eus(grid_localization_gl8
  "/home/hl/catkin_ws/src/grid_localization_gl8/msg/GPTRA_MSG.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grid_localization_gl8
)

### Generating Services

### Generating Module File
_generate_module_eus(grid_localization_gl8
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grid_localization_gl8
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(grid_localization_gl8_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(grid_localization_gl8_generate_messages grid_localization_gl8_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleSpeedFeedBack.msg" NAME_WE)
add_dependencies(grid_localization_gl8_generate_messages_eus _grid_localization_gl8_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleIMU.msg" NAME_WE)
add_dependencies(grid_localization_gl8_generate_messages_eus _grid_localization_gl8_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/GPTRA_MSG.msg" NAME_WE)
add_dependencies(grid_localization_gl8_generate_messages_eus _grid_localization_gl8_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(grid_localization_gl8_geneus)
add_dependencies(grid_localization_gl8_geneus grid_localization_gl8_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS grid_localization_gl8_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(grid_localization_gl8
  "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleSpeedFeedBack.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grid_localization_gl8
)
_generate_msg_lisp(grid_localization_gl8
  "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleIMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grid_localization_gl8
)
_generate_msg_lisp(grid_localization_gl8
  "/home/hl/catkin_ws/src/grid_localization_gl8/msg/GPTRA_MSG.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grid_localization_gl8
)

### Generating Services

### Generating Module File
_generate_module_lisp(grid_localization_gl8
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grid_localization_gl8
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(grid_localization_gl8_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(grid_localization_gl8_generate_messages grid_localization_gl8_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleSpeedFeedBack.msg" NAME_WE)
add_dependencies(grid_localization_gl8_generate_messages_lisp _grid_localization_gl8_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleIMU.msg" NAME_WE)
add_dependencies(grid_localization_gl8_generate_messages_lisp _grid_localization_gl8_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/GPTRA_MSG.msg" NAME_WE)
add_dependencies(grid_localization_gl8_generate_messages_lisp _grid_localization_gl8_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(grid_localization_gl8_genlisp)
add_dependencies(grid_localization_gl8_genlisp grid_localization_gl8_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS grid_localization_gl8_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(grid_localization_gl8
  "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleSpeedFeedBack.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grid_localization_gl8
)
_generate_msg_py(grid_localization_gl8
  "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleIMU.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grid_localization_gl8
)
_generate_msg_py(grid_localization_gl8
  "/home/hl/catkin_ws/src/grid_localization_gl8/msg/GPTRA_MSG.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grid_localization_gl8
)

### Generating Services

### Generating Module File
_generate_module_py(grid_localization_gl8
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grid_localization_gl8
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(grid_localization_gl8_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(grid_localization_gl8_generate_messages grid_localization_gl8_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleSpeedFeedBack.msg" NAME_WE)
add_dependencies(grid_localization_gl8_generate_messages_py _grid_localization_gl8_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/VehicleIMU.msg" NAME_WE)
add_dependencies(grid_localization_gl8_generate_messages_py _grid_localization_gl8_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hl/catkin_ws/src/grid_localization_gl8/msg/GPTRA_MSG.msg" NAME_WE)
add_dependencies(grid_localization_gl8_generate_messages_py _grid_localization_gl8_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(grid_localization_gl8_genpy)
add_dependencies(grid_localization_gl8_genpy grid_localization_gl8_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS grid_localization_gl8_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grid_localization_gl8)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grid_localization_gl8
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(grid_localization_gl8_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grid_localization_gl8)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grid_localization_gl8
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(grid_localization_gl8_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grid_localization_gl8)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grid_localization_gl8
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(grid_localization_gl8_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grid_localization_gl8)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grid_localization_gl8\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grid_localization_gl8
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(grid_localization_gl8_generate_messages_py std_msgs_generate_messages_py)
endif()
