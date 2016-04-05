#include "sensorData.hpp"
#include "ros/ros.h"

/* Class representing the idle state.
 * This state is the default of the program
 */
class State_Idle : public State {	
  public:	
	*State_Idle() { } 
	
	State* update() override {
		ROS_INFO("Update called, let me idle..");
		return this;
	}
    
    void switchTo() override {
		ROS_INFO("switched to Idle state");
	}
    
    void switchFrom() override {
		ROS_INFO("switched from Idle state");
	}
};
