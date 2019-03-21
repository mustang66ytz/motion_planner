cmake_minimum_required(VERSION 2.8.3)
project(newik)

message(status " ROS_PLATFORM is " $ENV{ROS_PLATFORM})

if("$ENV{ROS_PLATFORM}" STREQUAL "PC")

set(LibMotionPlanning "${CMAKE_CURRENT_SOURCE_DIR}/build/X86/libXR1MotionPlanning.so")

elseif("$ENV{ROS_PLATFORM}" STREQUAL "TX2")

set(LibMotionPlanning "${CMAKE_CURRENT_SOURCE_DIR}/build/TX2/libXR1MotionPlanning.so")

else()

message( FATAL_ERROR "Please define ROS_PLATFORM first" )

endif()

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  trajectoryIK.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(
  newik
  src/iksrv.cpp
)
add_dependencies(newik ${PROJECT_NAME}_generate_messages_cpp)

add_executable(trajectory_ik_server src/iksrv.cpp)

add_dependencies(trajectory_ik_server ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
  trajectory_ik_server newik
  ${catkin_LIBRARIES}
  ${LibMotionPlanning}
)


