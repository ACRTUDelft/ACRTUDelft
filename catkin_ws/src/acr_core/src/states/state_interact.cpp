#include "state_interact.hpp"

State_Interact::State_Interact() { }

State* State_Interact::update() {
	bool interact = false;
	for (int i = 0; i < 3; i++) {
		if(SensorData::isFree(i)) {
			SensorData::sendModule(i, MODULE_IDLE);
		} else {
			SensorData::sendModule(i, MODULE_INTERACT);
			interact = true;
		}
	}	
	if (interact) {
		return this;
	} else {
		return new State_Idle();
	}
}

void State_Interact::switchTo() { 
		ROS_INFO("Switched to State_Interact");
}

void State_Interact::switchFrom() {
	SensorData::sendModule(MODULE1, MODULE_IDLE);
	SensorData::sendModule(MODULE2, MODULE_IDLE);
	SensorData::sendModule(MODULE3, MODULE_IDLE);
}
