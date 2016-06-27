#include "state_navigating.hpp"

#include "state_roaming.cpp"

#include <math.h>

#define MOVE_SPEED 1

State_Navigating::State_Navigating() { }

State* State_Navigating::update() {
	return new State_Roaming();
}

void State_Navigating::switchTo() { 
		ROS_INFO("Switched to State_Navigating");
}

void State_Navigating::switchFrom() {
	SensorData::sendTwist(0.f, 0.f); // stop driving
}
