#ifndef State_ConcreteInteraction_H
#define State_ConcreteInteraction_H

#include "../sensorData.hpp"
class State_Idle;

#include "ros/ros.h"


#include "state_interact.hpp"

/* State interact is the state where the robot has interaction with a person/
 * When the person is gone, it will return to the idle state.
 */ 
class State_ConcreteInteraction : public State_Interact {	
  public:	
	State_ConcreteInteraction();
	
	State* update() override;
    
    void switchTo() override;
    
    void switchFrom() override;
};

#endif
