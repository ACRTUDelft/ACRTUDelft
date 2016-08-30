#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "../consts.hpp"
#include "../pins.hpp"

#include <wiringPi.h>
#include <softPwm.h>

#define ENGINE_TIMEOUT 1000
#define ENGINE_LINEAR 0
#define ENGINE_ANGULAR 1

const int ENGINE_FORW[] = {ENGINE0_RORW, ENGINE1_FORW};
const int ENGINE_BACK[] = {ENGINE0_BACK, ENGINE1_BACK};

unsigned int lastMessage = millis();

using namespace ros;

/**
 * Set all the pins to input/output and reset the outputs
 */
void setupPins() {
	wiringPiSetupGpio();
	softPwmCreate(ENGINE_FORW[0], 0, 100);
	softPwmCreate(ENGINE_FORW[1], 0, 100);
	
	softPwmCreate(ENGINE_BACK[0], 0, 100);
	softPwmCreate(ENGINE_BACK[1], 0, 100);
}

void stopEngine(int engine) {
	if (engine > 1 || engine < 0) {
		ROS_WARN("Invalid engine in 'stopEngine(%d)'", engine);
		return;
	}
	softPwmWrite(ENGINE_FORW[engine], 0);
	softPwmWrite(ENGINE_BACK[engine], 0);
}

void runEngine(int engine, int power) {
	if (engine > 1 || engine < 0) {
		ROS_WARN("Invalid engine in 'runEngine(%d, %d)'", engine, power);
		return;
	}
	
	if (power > 100 || power < -100) {
		ROS_WARN("Invalid power in 'runEngine(%d, %d)'", engine, power);
		return;
	}
	
	if (power > 0) {
		softPwmWrite(ENGINE_FORW[engine], power);
		softPwmWrite(ENGINE_BACK[engine], 0);
	} else {
		softPwmWrite(ENGINE_BACK[engine], -power);
		softPwmWrite(ENGINE_FORW[engine], 0);
	}
}

/**
 * Callback for received messages on 'sensor_modules'
 * Writes to the pins if 'SENSOR_IDLE' or 'SENSOR_INTERACT' is received
 */
void moduleCallback(const geometry_msgs::Twist& msg) {
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

int main(int argc, char **argv) {
	setupPins();
	init(argc, argv, "tripodEngineControl");
	NodeHandle nh;

	Subscriber sub = nh.subscribe("cmd_vel", 10, moduleCallback);

	Rate loop_rate(10.f);	// 10 Hz
	
	while(ros::ok()) {
		if (millis() - lastMessage > ENGINE_TIMEOUT) {
			stopEngine(ENGINE_LINEAR);
			stopEngine(ENGINE_ANGULAR);
		}
		spinOnce();	
		loop_rate.sleep();
	}			
  return 0;
}
