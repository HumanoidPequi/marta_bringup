cmake_minimum_required(VERSION 3.0.2)

project(amanda_marta_vision_hello)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
    cv_bridge
)

catkin_install_python(
  PROGRAMS
    scripts/process_image.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
