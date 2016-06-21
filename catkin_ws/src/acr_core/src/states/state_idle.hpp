#ifndef State_Idle_H
#define State_Idle_H

class State_Navigating;
class State_Roaming;

#include "../sensorData.hpp"
#include "ros/ros.h"

/* Class representing the idle state.
 * This state is the default of the program.
 * 
 * The idle state sometimes decides to start searching for people.
 * When a point of interest is found the navigating state will be invoked.
 */
class State_Idle : public State {	
  public:  
	State_Idle();
	
	State* update() override;
    
    void switchTo() override;
    
    void switchFrom() override;
};
#endif
