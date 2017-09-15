cmake_minimum_required(VERSION 2.8.3)
project(roswell_navigation)


find_package(catkin REQUIRED
  COMPONENTS
    message_generation
)

add_service_files(
  FILES
    TiltControl.srv
    ReturnJointStates.srv
)

generate_messages()
find_package(catkin)
catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS scripts/roswell_tilt_head.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

