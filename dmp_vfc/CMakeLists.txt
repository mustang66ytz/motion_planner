cmake_minimum_required(VERSION 2.8.3)
project(dmp_vfc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  std_msgs
  message_generation
  roscpp
  roslib
  rospy
  tf_conversions
  visualization_msgs
  xr1controllerros
)

# this is for the setup.py
catkin_python_setup()

add_message_files(
  FILES
  Waypoint.msg
)

add_service_files(
  FILES
  GenericJSONService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/robots
  ${CMAKE_CURRENT_SOURCE_DIR}/cameras
  ${CMAKE_CURRENT_SOURCE_DIR}/test
  ${CMAKE_CURRENT_SOURCE_DIR}/utils
  ${CMAKE_CURRENT_SOURCE_DIR}/action_handler
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############


#############
## Testing ##
#############

