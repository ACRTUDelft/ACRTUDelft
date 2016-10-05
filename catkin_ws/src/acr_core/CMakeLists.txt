cmake_minimum_required(VERSION 2.8.3)
project(acr_core)

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
  qt_build
)

find_package(OpenCV)
#find_package(OpenCV 2 REQUIRED)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

include(${QT_USE_FILE})

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)

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

add_executable(cameraToAngle src/cameraToAngle/main.cpp)
  target_link_libraries(cameraToAngle ${catkin_LIBRARIES})
  target_link_libraries(cameraToAngle ${OpenCV_LIBRARIES})

add_executable(core src/core/main.cpp)
  target_link_libraries(core ${catkin_LIBRARIES})
 

#############
## Install ##
#############


#############
## Testing ##
#############
