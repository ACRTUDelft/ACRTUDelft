#include "state_approaching.hpp"


State_Idle::State_Idle() { }

State* State_Approaching::update() {
	//if(!isnan(SensorData::pointOfInterest())) {
		return new State_Navigating();
	/**}
	if(rand() % 100 < 10) {
		return new State_LookAround();
	} else {
		return this;
	} **/
}

void State_Idle::switchTo() {
	ROS_INFO("Switched to State_Approaching"); 
}

void State_Idle::switchFrom() { }
