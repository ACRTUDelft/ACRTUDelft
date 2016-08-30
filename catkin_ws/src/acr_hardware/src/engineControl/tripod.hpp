#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "engineControl.hpp"

using namespace ros;

#define ENGINE_LINEAR ENGINE0
#define ENGINE_ANGULAR ENGINE1

void tripodCallback(const geometry_msgs::Twist& msg) {
	if (msg.linear.x > 1 || msg.linear.x < -1) {
		ROS_WARN("Invalid linear.x, must be between -1 and 1 (now is %f)", msg.linear.x);
		return;
	}
	if (msg.angular.z > 90 || msg.linear.z < -90) {
		ROS_WARN("Invalid angular.z, must be between -90 and 90 (now is %f)", msg.angular.z);
		return;
	}
	
	lastMessage = millis();
	
	runEngine(ENGINE_LINEAR, (int) (msg.linear.x * 100));
	runEngine(ENGINE_ANGULAR, (int) (msg.angular.z * 1.1111f));	
}
