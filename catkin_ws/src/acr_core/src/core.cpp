#define MOD1 0
#define MOD2 1
#define MOD3 2

#define U_LEFT 0
#define U_FRONT_TOP 1
#define U_FRONT_BOTTOM 2
#define U_RIGHT 3

#include "states/stateSensorData.hpp"
#include "states/state.cpp"
#include "states/state_idle.cpp"

using namespace ros;

int main(int argc, char** argv) {	
	init(argc, argv, "core");
	NodeHandle nh;
	SensorData::init(nh);

	Rate loop_rate(10.f);	// 10 Hz
	State* current = new State_Idle();	// starting state
	
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
