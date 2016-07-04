#ifndef State_Interact_H
#define State_Interact_H

#include "../sensorData.hpp"
class State_Idle;

#include "ros/ros.h"

/* Abstract superclass for Interact behaviour.
 * This is useful you want certain condition to hold for all States 
 * within the Interact superbehaviour
 * For example: when you want all subbehaviours to return to the Idle state
 * under given conditions
 */ 
class State_Interact : public State {	
 public:
 
	/* Condition under which to go back to the Idle state
	 */
	bool backToIdle() {
		return false;
	};
	
};
#endif
