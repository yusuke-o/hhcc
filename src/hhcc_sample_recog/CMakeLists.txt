cmake_minimum_required(VERSION 2.8.3)
project(hhcc_sample_recog)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  rospy
  )

catkin_package(
  )

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  )

install(PROGRAMS
  scripts/find_red_object.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
