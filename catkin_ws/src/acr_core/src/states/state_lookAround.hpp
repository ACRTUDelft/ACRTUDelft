#ifndef State_LookAround_H
#define State_LookAround_H

#include "sensorData.hpp"

class State_Idle;
class State_Navigating;

#include "../consts.hpp"
#include "ros/ros.h"

/* 
 * The lookAround state is the state where the robot looks around in an attempt to find a person.
 * The robot looks around for a random amount of time before it returns to the idle state.
 */
class State_LookAround : public State {	
  public:	
	State_LookAround();
	
	State* update() override;
    
    void switchTo() override;
    
    void switchFrom() override;
};

#endif
