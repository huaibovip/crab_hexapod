cmake_minimum_required(VERSION 2.8.3)
project(crab_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  crab_msgs
  joy
  roscpp
  rospy
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crab_teleop_joy
  CATKIN_DEPENDS roscpp rospy std_msgs crab_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories( include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(teleop_joy src/teleop_joy.cpp)
add_dependencies(teleop_joy crab_msgs_gencpp)
target_link_libraries(teleop_joy ${catkin_LIBRARIES})


