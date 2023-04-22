cmake_minimum_required(VERSION 2.8.3)
project(jenga_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  GetBlocks.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(my_node src/my_node.cpp)
# target_link_libraries(my_node ${catkin_LIBRARIES})
# add_dependencies(my_node my_package_generate_messages_cpp)