#ifndef State_ConcreteInteraction_H
#define State_ConcreteInteraction_H

#include "../../sensorData.hpp"
class State_Idle;

#include "ros/ros.h"


#include "state_interact.hpp"

/* This state is part of the Interact superbehaviour
 * This state implements a placeholder routine for how the robot interacts
 */ 
class State_ConcreteInteraction : public State_Interact {	
  public:	
	State_ConcreteInteraction();
	
	State* update() override;
    
    void switchTo() override;
    
    void switchFrom() override;
};

#endif
