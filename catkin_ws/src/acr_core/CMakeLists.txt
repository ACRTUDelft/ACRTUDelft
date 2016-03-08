cmake_minimum_required(VERSION 2.8.3)
project(acr_core)

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

add_executable(WiFiControls src/WiFiControls.cpp)
target_link_libraries(WiFiControls ${catkin_LIBRARIES})


#############
## Install ##
#############


#############
## Testing ##
#############
