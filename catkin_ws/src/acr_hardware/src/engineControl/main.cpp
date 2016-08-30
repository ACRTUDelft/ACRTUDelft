#include "ros/ros.h"

#include "engineControl.hpp"
#include "tripod.hpp"
#include "normal.hpp"

#define ENGINE_TIMEOUT 1000

using namespace ros;

int main(int argc, char **argv) {
	setupPins();
	init(argc, argv, "engineControl");
	NodeHandle nh;
	bool tripodMode = false;

	Subscriber sub = nh.subscribe("cmd_vel", 10, tripodMode ? tripodCallback : normalCallback);

	Rate loop_rate(10.f);	// 10 Hz
	
	while(ros::ok()) {
		if (millis() - lastMessage > ENGINE_TIMEOUT) {
			ROS_WARN("No messages received for %dms, stopping the engines", ENGINE_TIMEOUT);
			stopEngine(ENGINE0);
			stopEngine(ENGINE1);
		}
		spinOnce();	
		loop_rate.sleep();
	}			
  return 0;
}
