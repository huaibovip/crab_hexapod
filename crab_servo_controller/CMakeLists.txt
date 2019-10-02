cmake_minimum_required(VERSION 2.8.3)
project(crab_servo_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs crab_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crab_servo_controller
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
   src/lib/PCA9685/src/ src/lib/I2C/src/ include ${catkin_INCLUDE_DIRS}
)

add_library(I2C src/lib/I2C/src/I2C.cpp)
target_link_libraries(I2C ${catkin_LIBRARIES})

add_library(PCA9685 src/lib/PCA9685/src/PCA9685.cpp)
target_link_libraries(I2C PCA9685 ${catkin_LIBRARIES})

add_executable(joint_sub src/joint_sub.cpp src/lib/I2C/src/I2C.cpp src/lib/PCA9685/src/PCA9685.cpp)
target_link_libraries(joint_sub I2C PCA9685 ${catkin_LIBRARIES})

add_executable(controller_sub src/controller_sub.cpp src/lib/I2C/src/I2C.cpp src/lib/PCA9685/src/PCA9685.cpp)
target_link_libraries(controller_sub I2C PCA9685 ${catkin_LIBRARIES})
