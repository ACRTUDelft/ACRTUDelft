#include "sensorData.hpp"
#include "../consts.hpp"
#include "ros/ros.h"

/* Example class for states
 */
class State_Example : public State {	
  public:	
	*State_Example() { } 
	
	State* update() override {
		ROS_INFO("~ Update called");
		/* when a sensor is not free there must be someone near it! :p*/
		if(!SensorData::isFree(MODULE1)) {						// condition checking
			SensorData::sendModule(MODULE1, MODULE_INTERACT);	// sending messages
		} else {
			SensorData::sendModule(MODULE1, MODULE_IDLE);		// sending other message
		}
		return new State_Example();								// or send 'this' to stay in this state
	}
    
    void switchTo() override {
		ROS_INFO("~ switched to example state");
		SensorData::sendTwist(0.f, 0.f); // stop driving!
	}
    
    void switchFrom() override {
		ROS_INFO("~ switched from example state");
	}
};
