#include "ros/ros.h"
#include "diagnostic_msgs/KeyValue.h"
#include "../consts.hpp"

#include <wiringPi.h>

const int SERVICE[] = {1, 2, 3};	// Service pins for the modules
const int INTERAC[] = {5, 6, 7};	// Interact pins for the modules 

using namespace ros;

/**
 * Set all the pins to input/output and reset the outputs
 */
void setupPins() {
	wiringPiSetup();
	for(int i = 0; i < MODULES; i++) {
		pinMode(SERVICE[i], INPUT);
		pinMode(INTERAC[i], OUTPUT);
		digitalWrite(INTERAC[i], LOW);
	}
}

/**
 * Callback for received messages on 'sensor_modules'
 * Writes to the pins if 'SENSOR_IDLE' or 'SENSOR_INTERACT' is received
 */
void moduleCallback(const diagnostic_msgs::KeyValue& msg) {
	char* tmp = strdup(msg.key.c_str());
	 int module = std::stoi(strtok(tmp, ":"));	
	delete tmp;
	
	if(module > MODULES) {
		ROS_WARN("module %d not found.", module);
		return;
	}
	
	int status = std::stoi(msg.value);
	if (status == MODULE_IDLE) {
		digitalWrite(INTERAC[module], LOW);
	} else if (status == MODULE_INTERACT) {
		digitalWrite(INTERAC[module], HIGH);		
	}
}


int main(int argc, char **argv) {
	setupPins();
	init(argc, argv, "moduleControl");
	NodeHandle nh;

	Publisher pub = nh.advertise<diagnostic_msgs::KeyValue>("sensor_module", 2 * MODULES);
	nh.subscribe("sensor_module", 10, moduleCallback);
	Rate loop_rate(10.f);	// 10 Hz
	
	int c = 0;
	while(ros::ok()) {
		if (c == 20) {		// .5 Hz for module state
			for(int i = 0; i < MODULES; i++) {	// for each module
				diagnostic_msgs::KeyValue msg = diagnostic_msgs::KeyValue();
				msg.key = "module:" + i;
				if (digitalRead(SERVICE[i])) {
					 msg.value = std::to_string(MODULE_FULL);
				} else {
					msg.value = std::to_string(MODULE_OK);
				}
				pub.publish(msg);
			}
			c = 0; // reset
		}
		spinOnce();		
		loop_rate.sleep();
		c++;
	}			
  return 0;
}
