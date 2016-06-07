#ifndef State_Navigating_H
#define State_Navigating_H

#include "sensorData.hpp"
class State_Idle;

#include "../consts.hpp"
#include "ros/ros.h"

/* 
 * The lookAround state is the state where the robot looks around in an attempt to find a person.
 * The robot looks around for a random amount of time before it returns to the idle state.
 */
class State_Navigating : public State {	
  public:	
	State_Navigating();
	
	State* update() override;
    
    void switchTo() override;
    
    void switchFrom() override;
};

#endif
