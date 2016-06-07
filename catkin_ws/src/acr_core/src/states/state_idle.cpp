#include "state_idle.hpp" 
#include "state_navigating.cpp"
#include "state_lookAround.cpp"

#include <math.h>

State_Idle::State_Idle() { }

State* State_Idle::update() {
	if(!isnan(SensorData::pointOfInterest())) {
		ROS_INFO("%f", SensorData::pointOfInterest());
		return new State_Navigating();
	}
	if(rand() % 100 < 10) {
		return new State_LookAround();
	} else {
		return this;
	}
}

void State_Idle::switchTo() {
	ROS_INFO("Switched to State_Idle"); 
}

void State_Idle::switchFrom() { }
