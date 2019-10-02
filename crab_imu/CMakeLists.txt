cmake_minimum_required(VERSION 2.8.3)
project(crab_imu)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  crab_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp crab_msgs
#  DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( include src/IMU src/sys ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(imu		 	src/IMU/Compass.cpp
				src/IMU/DCM.cpp
				src/IMU/IMU.cpp
				src/IMU/L3G.cpp
				src/IMU/LSM303.cpp
				src/IMU/Matrix.cpp
				src/IMU/Vector.cpp)
target_link_libraries(imu ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(imu_control src/imu_control.cpp src/sys/Timer.cpp src/sys/I2C.cpp)
add_dependencies(imu_control crab_msgs_gencpp)
target_link_libraries(imu_control imu ${catkin_LIBRARIES})