#ifndef State_Navigating_H
#define State_Navigating_H

#include "../../sensorData.hpp"
class State_Idle;

#include "../../../consts.hpp"
#include "ros/ros.h"


/* Abstract superclass for Navigating behaviour.
 * This is useful you want certain condition to hold for all States 
 * within the Interact superbehaviour
 * For example: when you want all subbehaviours to return to the Idle state
 * under given conditions
 */ 
class State_Navigating : public State {	

	/* Condition under which to go back to the Idle state
	 */
	public:
		bool backToIdle() {return false;};
	
};

#endif
