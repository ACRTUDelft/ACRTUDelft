cmake_minimum_required(VERSION 2.8.3)
project(acr_hardware)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
rospy 
std_msgs 
message_generation
sensor_msgs
cv_bridge
image_transport
)

find_package(OpenCV)
   find_package(OpenCV 2 REQUIRED)

   include_directories(${OpenCV_INCLUDE_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

## Allow c++11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")



include_directories(
  ${catkin_INCLUDE_DIRS}

)



add_executable(cameraToAngle src/cameraToAngle/cameraToAngle.cpp)
target_link_libraries(cameraToAngle ${catkin_LIBRARIES})
target_link_libraries(cameraToAngle ${OpenCV_LIBRARIES})

add_executable(flirLepton src/flirLepton/flirLepton.cpp)
target_link_libraries(flirLepton ${catkin_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############
