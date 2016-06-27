#include "state_approaching.hpp"

#include "state_interact.cpp"

#define MOVE_SPEED 1

State_Approaching::State_Approaching() { }

State* State_Approaching::update() {
	if(!isnan(SensorData::pointOfInterest())) {
			if(!SensorData::isFree(U_FRONT_TOP)) {
			return new State_Interact();
		}
		float angle = SensorData::pointOfInterest();
		
		SensorData::sendTwist(-angle, MOVE_SPEED);
		return this;
	}
	else {
		return new State_Idle();
	}
}

void State_Approaching::switchTo() {
	ROS_INFO("Switched to State_Approaching"); 
}

void State_Approaching::switchFrom() { }
