cmake_minimum_required(VERSION 2.8.3)
project(crab_body_kinematics)

find_package(catkin REQUIRED COMPONENTS 
  crab_msgs
  std_msgs
  kdl_parser
  roscpp
  rospy
)

find_package(orocos_kdl REQUIRED)

#generate_messages(
#   DEPENDENCIES
#   crab_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crab_rpy_kinematics
  CATKIN_DEPENDS crab_msgs kdl_parser roscpp rospy std_msgs
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
add_executable(rpy_body_kinematics src/rpy_body_kinematics.cpp)
add_dependencies(rpy_body_kinematics crab_msgs_gencpp)
target_link_libraries(rpy_body_kinematics ${catkin_LIBRARIES})

add_executable(body_kinematics src/body_kinematics.cpp)
add_dependencies(body_kinematics crab_msgs_gencpp)
target_link_libraries(body_kinematics ${catkin_LIBRARIES})


