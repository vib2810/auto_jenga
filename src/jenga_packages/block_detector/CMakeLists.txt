cmake_minimum_required(VERSION 3.0.2)
project(block_detector)

# set(CMAKE_PREFIX_PATH "/usr/local/lib/python3.8/dist-packages/")
# set(CMAKE_PREFIX_PATH "/home/student/auto_jenga/src/jenga_packages/jenga_msgs")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  jenga_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs jenga_msgs
)

## Specify locations of header files
# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   /usr/local/lib/python3.8/dist-packages/
#   /home/student/auto_jenga/src/jenga_packages/jenga_msgs
# )

## Add executables
catkin_install_python(PROGRAMS
  src/rec_data_main.py
  src/azure_camera_calibration.py
  src/easy_tf_publisher.py
  src/block_segmenter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and config files)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

