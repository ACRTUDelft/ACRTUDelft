#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "engineControl.hpp"

using namespace ros;

#define ENGINE_LEFT ENGINE0
#define ENGINE_RIGHT ENGINE1

/**
 * Callback for received messages on 'cmd_vel'
 * Calculates the speeds for both engines and writes this to the corresponding pins.
 * 
 * This method is used for vehicles with a left and right engine.
 * When the linear speed is 0, the turns become in place by reversing the other engine.
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
	
	if (angular == 0) { 		// straight
		runEngine(ENGINE_LEFT, speed);
		runEngine(ENGINE_RIGHT, speed);
	} else if (angular > 0) { 	// left
		if (speed == 0) {
			runEngine(ENGINE_RIGHT, angular);
			runEngine(ENGINE_LEFT, -angular);
		} else {
			runEngine(ENGINE_RIGHT, speed / (angular + 1));
			runEngine(ENGINE_LEFT, speed);
		}
	} else {					// right
		if (speed == 0) {
			runEngine(ENGINE_LEFT, angular);
			runEngine(ENGINE_RIGHT, -angular);
		} else {
			runEngine(ENGINE_LEFT, speed / (angular + 1));
			runEngine(ENGINE_RIGHT, speed);
		}
	}
}
