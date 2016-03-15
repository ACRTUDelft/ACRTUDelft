#include "sensorData.hpp"
#include "ros/ros.h"

/* Class representing the idle state.
 * This state is the default of the program
 */
class State_Idle : public State {	
  public:	
	*State_Idle() { } 
	
	State* update() override {
		ROS_INFO("Update called");
		if(SensorData::isFree(0) == false) ROS_INFO(" - false");	// example
		return new State_Idle();
	}
    
    void switchTo() override {
		ROS_INFO("switched to idle state");
	}
    
    void switchFrom() override {
		ROS_INFO("switched from idle state");
	}
};
