cmake_minimum_required(VERSION 2.8.3)
project(crab_leg_kinematics)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  crab_msgs
  kdl_parser
  roscpp
  rospy
)

find_package(orocos_kdl)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS kdl_parser roscpp rospy std_msgs crab_msgs
#  DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(leg_ik_solver src/hp_chainiksolverpos_nr_jl.cpp src/chainiksolvervel_pinv.cpp src/svd_HH.cpp)
add_dependencies(leg_ik_solver crab_msgs_gencpp)
target_link_libraries(leg_ik_solver ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(leg_kinematics src/leg_kinematics.cpp)
add_dependencies(leg_kinematics crab_msgs_gencpp)
target_link_libraries(leg_kinematics leg_ik_solver ${catkin_LIBRARIES})

add_executable(leg_kinematics_pub src/leg_kinematics_pub.cpp)
add_dependencies(leg_kinematics_pub crab_msgs_gencpp)
target_link_libraries(leg_kinematics_pub leg_ik_solver ${catkin_LIBRARIES})

add_executable(leg_ik_service src/leg_ik_service.cpp)
add_dependencies(leg_ik_service crab_msgs_gencpp)
target_link_libraries(leg_ik_service leg_ik_solver ${catkin_LIBRARIES})


