cmake_minimum_required(VERSION 2.8.3)
project(crab_gait)


find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  crab_msgs
)

find_package(orocos_kdl)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crab_rpy_kinematics
  CATKIN_DEPENDS crab_msgs roscpp
#  DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
##add_executable(gait_test src/gait_test.cpp)
##target_link_libraries(gait_test ${catkin_LIBRARIES})

add_executable(gait_kinematics src/gait_kinematics.cpp src/gait.cpp)
add_dependencies(gait_kinematics crab_msgs_gencpp)
target_link_libraries(gait_kinematics ${catkin_LIBRARIES})

