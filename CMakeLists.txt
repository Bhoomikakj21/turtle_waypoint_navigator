cmake_minimum_required(VERSION 3.0.2)
project(turtle_waypoint_navigator)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  turtlesim
)

## Declare the catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/waypoint_navigator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
