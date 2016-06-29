#ifndef State_Navigating_H
#define State_Navigating_H

#include "../sensorData.hpp"
class State_Idle;

#include "../consts.hpp"
#include "ros/ros.h"

/* 
 * The navigating state is the state where the navigates towards a point of interest.
 * When an object is in front of the robot it returns to the idle state or goes to the interact state.
 */
class State_Navigating : public State {	

	public:
		bool backToIdle() {return false;};
	
};

#endif
