#ifndef State_Interact_H
#define State_Interact_H

#include "sensorData.hpp"
class State_Idle;

#include "ros/ros.h"

/* State interact is the state where the robot has interaction with a person/
 * When the person is gone, it will return to the idle state.
 */ 
class State_Interact : public State {	
  public:	
	State_Interact();
	
	State* update() override;
    
    void switchTo() override;
    
    void switchFrom() override;
};

#endif
