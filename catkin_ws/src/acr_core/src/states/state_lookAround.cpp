#include "state_lookAround.hpp"

#include <math.h>

State_LookAround::State_LookAround() { }

State* State_LookAround::update() {
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

void State_LookAround::switchTo() { 
		ROS_INFO("Switched to State_LookAround");
}

void State_LookAround::switchFrom() {
	SensorData::sendTwist(0.f, 0.f); // stop turning!
}
