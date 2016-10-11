cmake_minimum_required(VERSION 2.8.3)
project(acr_core)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp
	std_msgs 
	message_generation
	sensor_msgs
	cv_bridge
	image_transport
	qt_build
)

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

add_executable(cameraToAngle src/cameraToAngle/node_cameraToAngle.cpp)
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
catkin_add_gtest(sensorData-test test/test_core_sensorData.cpp)
	target_link_libraries(sensorData-test ${catkin_LIBRARIES})

catkin_add_gtest(cameraToAngle-test test/test_cameraToAngle.cpp)
	target_link_libraries(cameraToAngle-test ${catkin_LIBRARIES})
	target_link_libraries(cameraToAngle-test ${OpenCV_LIBRARIES})
	configure_file(test/testLeft.png test/testLeft.png COPYONLY)
	configure_file(test/testRight.png test/testRight.png COPYONLY)
	configure_file(test/testEmpty.png test/testEmpty.png COPYONLY)
