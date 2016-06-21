#include "state_navigating.hpp"


#include "state_roaming.cpp"

#include <math.h>

#define MOVE_SPEED 1

State_Navigating::State_Navigating() { }

State* State_Navigating::update() {
	return new State_Roaming();
	/**float angle = SensorData::pointOfInterest();
	if(isnan(angle)) return new State_Idle();
	
	if(SensorData::isFree(U_FRONT_TOP)) {
		SensorData::sendTwist(-angle, MOVE_SPEED);
		return this;
	} else {
		return new State_Interact();
	}**/
}

void State_Navigating::switchTo() { 
		ROS_INFO("Switched to State_Navigating");
}

void State_Navigating::switchFrom() {
	SensorData::sendTwist(0.f, 0.f); // stop driving
}
