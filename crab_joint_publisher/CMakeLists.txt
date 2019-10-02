cmake_minimum_required(VERSION 2.8.3)
project(crab_joint_publisher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  roscpp
  rospy
  sensor_msgs
  std_msgs
  crab_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crab_joint_publisher
  CATKIN_DEPENDS joint_state_publisher roscpp sensor_msgs std_msgs crab_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(joint_publisher src/joint_publisher.cpp)
add_dependencies(joint_publisher crab_msgs_gencpp)
target_link_libraries(joint_publisher ${catkin_LIBRARIES})

