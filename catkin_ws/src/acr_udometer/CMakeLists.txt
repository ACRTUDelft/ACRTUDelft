cmake_minimum_required(VERSION 2.8.3)
project(acr_udometer)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
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


###########
## Build ##
###########

## Allow c++11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(udometer src/udometer.cpp)
target_link_libraries(udometer ${catkin_LIBRARIES})
target_link_libraries(udometer curl)
target_link_libraries(udometer curlpp)

#############
## Install ##
#############


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_acr_udometer.cpp)



