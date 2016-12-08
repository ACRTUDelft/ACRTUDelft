cmake_minimum_required(VERSION 2.8.3)
project(acr_launch)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES acr_launch
#  CATKIN_DEPENDS roslaunch
#  DEPENDS system_lib
)

###########
## Build ##
###########

#############
## Install ##
#############

#############
## Testing ##
#############

# TODO: add a check for the launch files here
