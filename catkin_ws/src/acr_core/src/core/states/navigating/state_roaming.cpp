#include "state_roaming.hpp"

#include "state_approaching.cpp"

#include <math.h>

State_Roaming::State_Roaming() { }

State* State_Roaming::update() {
	if(State_Navigating::backToIdle()) {
		return new State_Idle();
	}
	if(!isnan(SensorData::pointOfInterest())) {
		return new State_Approaching();
	}
	if(rand() % 100 < 5) {
		return new State_Idle();
	} else {
		SensorData::sendTwist(TURN_SPEED, 0.f);
		return this;
	}
}

void State_Roaming::switchTo() { 
	ROS_INFO("Switched to State_Roaming");
	ROS_INFO("Superbehaviour: State_Navigating");
}

void State_Roaming::switchFrom() {
	SensorData::sendTwist(0.f, 0.f); // stop turning!
}
