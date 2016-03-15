#include "consts.hpp"
#include "states/sensorData.cpp"
#include "states/state.hpp"
#include "states/state_idle.cpp"

using namespace ros;

int main(int argc, char** argv) {	
	init(argc, argv, "core");
	NodeHandle nh;
	SensorData::init(nh);

	Rate loop_rate(10.f);	// 10 Hz
	State* current = new State_Idle();	// starting state
	current -> switchTo();
	
	while(ok()) {
		/* Update the state */
		State* newState = current -> update();
		if (newState != current) {
			current -> switchFrom();
			newState -> switchTo();
			delete current;
			current = newState;
		}
		spinOnce();
		loop_rate.sleep();
	}
	current->switchFrom();
	return 0;
}
