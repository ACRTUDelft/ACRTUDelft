cmake_minimum_required(VERSION 2.8.3)
project(acr_debug)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  diagnostic_msgs
  geometry_msgs
  roscpp
  rospy
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

catkin_python_setup()

###########
## Build ##
###########

catkin_install_python(PROGRAMS src/battery_display/battery_display.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/keyboard_sensors/keyboard_sensors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/module_display/module_display.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/ultrasonic_display/ultrasonic_display.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Allow c++11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtleBridge src/turtleBridge/turtleBridge.cpp)
target_link_libraries(turtleBridge ${catkin_LIBRARIES})

add_executable(WiFiControls src/WiFiControls/WiFiControls.cpp)
target_link_libraries(WiFiControls ${catkin_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############
