#include "sensorData.hpp"
#include "ros/ros.h"

/* Class representing the idle state.
 * This state is the default of the program
 */
class State_Navigating : public State {	
  public:	
	*State_Navigating() { } 
	
	State* update() override {
		ROS_INFO("Driving :)");
		SensorData::sendTwist(1.f, 1.f); 
		return this;
	}
    
    void switchTo() override {
		ROS_INFO("switched to Idle state");
	}
    
    void switchFrom() override {
		ROS_INFO("switched from Idle state");
	}
};
