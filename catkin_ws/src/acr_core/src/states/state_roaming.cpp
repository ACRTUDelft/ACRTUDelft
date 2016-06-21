#include "state_roaming.hpp"

#include <math.h>

State_Roaming::State_Roaming() { }

State* State_Roaming::update() {
	if(!isnan(SensorData::pointOfInterest())) {
		return new State_Navigating();
	}
	if(rand() % 100 < 5) {
		return new State_Idle();
	} else {
		SensorData::sendTwist(1.f, 0.f);
		return this;
	}
}

void State_Roaming::switchTo() { 
		ROS_INFO("Switched to State_Roaming");
}

void State_Roaming::switchFrom() {
	SensorData::sendTwist(0.f, 0.f); // stop turning!
}
