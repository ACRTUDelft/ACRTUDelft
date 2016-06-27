#include "state_approaching.hpp"


State_Approaching::State_Approaching() { }

State* State_Approaching::update() {
	if(!isnan(SensorData::pointOfInterest())) {
		return this;
	}
	else {
		return new State_Idle();
	}
	/**}
	if(rand() % 100 < 10) {
		return new State_LookAround();
	} else {
		return this;
	} **/
}

void State_Approaching::switchTo() {
	ROS_INFO("Switched to State_Approaching"); 
}

void State_Approaching::switchFrom() { }
