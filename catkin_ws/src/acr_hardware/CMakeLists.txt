cmake_minimum_required(VERSION 2.8.3)
project(acr_hardware)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Allow c++11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtleBridge src/turtleBridge/turtleBridge.cpp)
target_link_libraries(turtleBridge ${catkin_LIBRARIES})

add_executable(ultrasonicSensors src/ultrasonicSensors/ultrasonicSensors.cpp)
target_link_libraries(ultrasonicSensors ${catkin_LIBRARIES})
target_link_libraries(ultrasonicSensors wiringPi)

#############
## Install ##
#############


#############
## Testing ##
#############
