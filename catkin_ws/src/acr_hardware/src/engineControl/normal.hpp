#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "engineControl.hpp"

using namespace ros;

#define ENGINE_LEFT ENGINE0
#define ENGINE_RIGHT ENGINE1

/**
 * Callback for received messages on 'sensor_modules'
 * Writes to the pins if 'SENSOR_IDLE' or 'SENSOR_INTERACT' is received
 */
void normalCallback(const geometry_msgs::Twist& msg) {
	if (msg.linear.x > 1 || msg.linear.x < -1) {
		ROS_WARN("Invalid linear.x, must be between -1 and 1 (now is %f)", msg.linear.x);
		return;
	}
	if (msg.angular.z > 90 || msg.linear.z < -90) {
		ROS_WARN("Invalid angular.z, must be between -90 and 90 (now is %f)", msg.angular.z);
		return;
	}
	
	lastMessage = millis();
	
	int speed = (int) (msg.linear.x * 100);
	int angular = (int) (msg.angular.z * 1.1111f);
	
	if (angular == 0) { 
		runEngine(ENGINE_LEFT, speed);
		runEngine(ENGINE_RIGHT, speed);
	} else if (angular > 0) { 
		if (speed == 0) {
			runEngine(ENGINE_RIGHT, angular);
			runEngine(ENGINE_LEFT, -angular);
		} else {
			runEngine(ENGINE_RIGHT, speed / (angular + 1));
			runEngine(ENGINE_LEFT, speed);
		}
	} else {
		if (speed == 0) {
			runEngine(ENGINE_LEFT, angular);
			runEngine(ENGINE_RIGHT, -angular);
		} else {
			runEngine(ENGINE_LEFT, speed / (angular + 1));
			runEngine(ENGINE_RIGHT, speed);
		}
	}
}
