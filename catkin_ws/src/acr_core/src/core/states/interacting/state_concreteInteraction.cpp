#include "state_concreteInteraction.hpp"

State_ConcreteInteraction::State_ConcreteInteraction() { }

State* State_ConcreteInteraction::update() {
	if(State_Interact::backToIdle()) {
		return new State_Idle();
	}
	bool interact = false;
	for (int i = 0; i < 3; i++) {
		if(SensorData::isFree(i)) {
			SensorData::sendModule(i, acr_msgs::ModuleState::MODULE_IDLE);
		} else {
			SensorData::sendModule(i, acr_msgs::ModuleState::MODULE_INTERACT);
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
	SensorData::sendModule(acr_msgs::ModuleState::MODULE1, acr_msgs::ModuleState::MODULE_IDLE);
	SensorData::sendModule(acr_msgs::ModuleState::MODULE2, acr_msgs::ModuleState::MODULE_IDLE);
	SensorData::sendModule(acr_msgs::ModuleState::MODULE3, acr_msgs::ModuleState::MODULE_IDLE);
}
