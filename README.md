![Travis build status] (https://api.travis-ci.org/ACRTUDelft/ACRTUDelft.svg?branch=master)
# Autonomous Cleaning Robot | Tu Delft
The aim of this interdisciplinary project on the TU Dleft is to design a robot which is able to collect waste from students at the campus. The project will run for multiple years, with each year a new group of students, until the robot is fully functional.

> ### Instead of you moving to a litter bin, the litter bin is moving to you!

Although the name suggests the robot can also clean, this is actually not true, it is only a driving bin.  
The robot will track people using a combination of a *FlirLepton* infrared camera and OpenCV as image recognition software. Behaviour of the robot is created by using a state machine with states encoding the desired situations and their interaction with the hardware.
The core of the software is the [ROS](https://www.ros.org/) framework. This allows us to create small and simple modules that work together to create the desired behaviour of the robot. The interaction between the nodes is explained in our Wiki alongside additional information about the code.


