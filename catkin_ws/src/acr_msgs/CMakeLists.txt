cmake_minimum_required(VERSION 2.8.3)
project(acr_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	std_msgs
	sensor_msgs
	message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	ModuleState.msg
	Ultrasonic.msg
)

## Generate services in the 'srv' folder
# add_service_files(
# 	FILES
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	geometry_msgs
	std_msgs
	sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
	CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
	${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############


#############
## Testing ##
#############
