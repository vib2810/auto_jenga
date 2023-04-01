cmake_minimum_required(VERSION 3.0.2)
project(dummy_pose_publisher)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package()

## Specify locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add executables
catkin_install_python(PROGRAMS
  src/dummy_pose_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and config files)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)