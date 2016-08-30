#ifndef engineControl_H
#define engineControl_H

#include "ros/ros.h"
#include "../pins.hpp"

#include <wiringPi.h>
#include <softPwm.h>

#define ENGINE0 0
#define ENGINE1 1

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
#endif
