#include "state_concreteInteraction.hpp"

State_ConcreteInteraction::State_ConcreteInteraction() { }

State* State_ConcreteInteraction::update() {
	if(State_Interact::backToIdle()) {
		return new State_Idle();
	}
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

void State_ConcreteInteraction::switchTo() { 
		ROS_INFO("Switched to State_ConcreteInteraction");
		ROS_INFO("Superbehaviour: State_Interact");
}

void State_ConcreteInteraction::switchFrom() {
	SensorData::sendModule(MODULE1, MODULE_IDLE);
	SensorData::sendModule(MODULE2, MODULE_IDLE);
	SensorData::sendModule(MODULE3, MODULE_IDLE);
}
