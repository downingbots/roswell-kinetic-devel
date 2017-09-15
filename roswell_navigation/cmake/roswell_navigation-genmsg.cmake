# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "roswell_navigation: 0 messages, 2 services")

set(MSG_I_FLAGS "")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(roswell_navigation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/ReturnJointStates.srv" NAME_WE)
add_custom_target(_roswell_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "roswell_navigation" "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/ReturnJointStates.srv" ""
)

get_filename_component(_filename "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/TiltControl.srv" NAME_WE)
add_custom_target(_roswell_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "roswell_navigation" "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/TiltControl.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(roswell_navigation
  "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/ReturnJointStates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roswell_navigation
)
_generate_srv_cpp(roswell_navigation
  "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/TiltControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roswell_navigation
)

### Generating Module File
_generate_module_cpp(roswell_navigation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roswell_navigation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(roswell_navigation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(roswell_navigation_generate_messages roswell_navigation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/ReturnJointStates.srv" NAME_WE)
add_dependencies(roswell_navigation_generate_messages_cpp _roswell_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/TiltControl.srv" NAME_WE)
add_dependencies(roswell_navigation_generate_messages_cpp _roswell_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roswell_navigation_gencpp)
add_dependencies(roswell_navigation_gencpp roswell_navigation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roswell_navigation_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(roswell_navigation
  "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/ReturnJointStates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/roswell_navigation
)
_generate_srv_eus(roswell_navigation
  "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/TiltControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/roswell_navigation
)

### Generating Module File
_generate_module_eus(roswell_navigation
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/roswell_navigation
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(roswell_navigation_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(roswell_navigation_generate_messages roswell_navigation_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/ReturnJointStates.srv" NAME_WE)
add_dependencies(roswell_navigation_generate_messages_eus _roswell_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/TiltControl.srv" NAME_WE)
add_dependencies(roswell_navigation_generate_messages_eus _roswell_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roswell_navigation_geneus)
add_dependencies(roswell_navigation_geneus roswell_navigation_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roswell_navigation_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(roswell_navigation
  "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/ReturnJointStates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roswell_navigation
)
_generate_srv_lisp(roswell_navigation
  "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/TiltControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roswell_navigation
)

### Generating Module File
_generate_module_lisp(roswell_navigation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roswell_navigation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(roswell_navigation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(roswell_navigation_generate_messages roswell_navigation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/ReturnJointStates.srv" NAME_WE)
add_dependencies(roswell_navigation_generate_messages_lisp _roswell_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/TiltControl.srv" NAME_WE)
add_dependencies(roswell_navigation_generate_messages_lisp _roswell_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roswell_navigation_genlisp)
add_dependencies(roswell_navigation_genlisp roswell_navigation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roswell_navigation_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(roswell_navigation
  "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/ReturnJointStates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/roswell_navigation
)
_generate_srv_nodejs(roswell_navigation
  "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/TiltControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/roswell_navigation
)

### Generating Module File
_generate_module_nodejs(roswell_navigation
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/roswell_navigation
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(roswell_navigation_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(roswell_navigation_generate_messages roswell_navigation_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/ReturnJointStates.srv" NAME_WE)
add_dependencies(roswell_navigation_generate_messages_nodejs _roswell_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/TiltControl.srv" NAME_WE)
add_dependencies(roswell_navigation_generate_messages_nodejs _roswell_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roswell_navigation_gennodejs)
add_dependencies(roswell_navigation_gennodejs roswell_navigation_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roswell_navigation_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(roswell_navigation
  "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/ReturnJointStates.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roswell_navigation
)
_generate_srv_py(roswell_navigation
  "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/TiltControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roswell_navigation
)

### Generating Module File
_generate_module_py(roswell_navigation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roswell_navigation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(roswell_navigation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(roswell_navigation_generate_messages roswell_navigation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/ReturnJointStates.srv" NAME_WE)
add_dependencies(roswell_navigation_generate_messages_py _roswell_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ros/catkin_ws/src/roswell-kinetic-devel/roswell_navigation/srv/TiltControl.srv" NAME_WE)
add_dependencies(roswell_navigation_generate_messages_py _roswell_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roswell_navigation_genpy)
add_dependencies(roswell_navigation_genpy roswell_navigation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roswell_navigation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roswell_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roswell_navigation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/roswell_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/roswell_navigation
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roswell_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roswell_navigation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/roswell_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/roswell_navigation
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roswell_navigation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roswell_navigation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roswell_navigation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
