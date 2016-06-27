#include "state_idle.hpp" 
#include "state_navigating.cpp"
#include <math.h>

State_Idle::State_Idle() { }

State* State_Idle::update() {
	return new State_Navigating();
}

void State_Idle::switchTo() {
	ROS_INFO("Switched to State_Idle"); 
}

void State_Idle::switchFrom() { }
