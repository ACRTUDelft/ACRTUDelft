#include "ros/ros.h"
#include "acr_msgs/ModuleState.h"
#include "../settings.hpp"
#include "../pins.hpp"

#include <wiringPi.h>

const int SERVICE[] = {MOD1_S, MOD2_S, MOD3_S};	// Service pins for the modules
const int INTERAC[] = {MOD1_I, MOD2_I, MOD3_I};	// Interact pins for the modules 

using namespace ros;

/**
 * Set all the pins to input/output and reset the outputs
 */
void setupPins() {
	wiringPiSetupGpio();
	for(int i = 0; i < MODULES; i++) {
		pinMode(SERVICE[i], INPUT);
		pinMode(INTERAC[i], OUTPUT);
		digitalWrite(INTERAC[i], LOW);
	}
}

/**
 * Callback for received messages on 'sensor_modules'
 * Writes to the pins if 'MODULE_IDLE' or 'MODULE_INTERACT' is received
 */
void moduleCallback(const acr_msgs::ModuleState& msg) {

	int module = msg.module;	
	if(module >= MODULES) {
		ROS_WARN("module %d not found.", module);
		return;
	}
	
	int status = msg.state;
	if (status == acr_msgs::ModuleState::MODULE_IDLE) {
		digitalWrite(INTERAC[module], LOW);
	} else if (status == acr_msgs::ModuleState::MODULE_INTERACT) {
		digitalWrite(INTERAC[module], HIGH);		
	}
}


int main(int argc, char **argv) {
	setupPins();
	init(argc, argv, "moduleControl");
	NodeHandle nh;

	Publisher pub = nh.advertise<acr_msgs::ModuleState>("sensor_module", 2 * MODULES);
	Subscriber sub = nh.subscribe("sensor_module", 10, moduleCallback);

	Rate loop_rate(1.f);	// 1 Hz
	
	int c = 0;
	while(ros::ok()) {
		for(int i = 0; i < MODULES; i++) {	// for each module
			acr_msgs::ModuleState msg = acr_msgs::ModuleState();
			msg.module = i;
			if (digitalRead(SERVICE[i])) {
				 msg.state = acr_msgs::ModuleState::MODULE_FULL;
			} else {
				msg.state = acr_msgs::ModuleState::MODULE_OK;
			}
			pub.publish(msg);
		}
		spinOnce();	
		loop_rate.sleep();
	}			
  return 0;
}
