cmake_minimum_required(VERSION 3.0.2)
project(moveit_block_pose_planner)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
  jenga_msgs
  actionlib_msgs
)

add_service_files(
  FILES
  block_pose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  jenga_msgs
  actionlib_msgs
)
## Declare a catkin package
catkin_package()

## Specify locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)



## Add executables
catkin_install_python(PROGRAMS
  src/moveit_block_move.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and config files)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)