cmake_minimum_required(VERSION 2.8.3)
project(hhcc_challenge)

find_package(catkin REQUIRED)

catkin_package(
)

install(PROGRAMS
  scripts/simple_pick.py
  scripts/simple_pick_2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
