#include "state_navigating.hpp"

#include <math.h>

#define MOVE_SPEED 1

State_Navigating::State_Navigating() { }

State* State_Navigating::update() {
	float angle = SensorData::pointOfInterest();		
	if(SensorData::isFree(MODULE2) > 0 && !isnan(angle)) {
		SensorData::sendTwist(-angle, MOVE_SPEED);
		return this;
	}
	return new State_Idle();
}

void State_Navigating::switchTo() { 
		ROS_INFO("Switched to State_Navigating");
}

void State_Navigating::switchFrom() {
	SensorData::sendTwist(0.f, 0.f); // stop driving
}
