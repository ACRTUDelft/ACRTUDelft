#include "ros/ros.h"
#include "buienradar.cpp"

using namespace ros;
/**
 * ROS node that sends weather information every 5 minutes on 'udometer'.
 * Uses a Buienradar API for gathering of the weather data.
 * Sends a 'geometry_msgs/Vector3' with:
 * 	x: current rain in mm/h
 * 	y: condition change time in minutes from now (5 minute precision)
 * 	z: rain value after the change in mm/h
 */
int main(int argc, char **argv) {
	init(argc, argv, "udometer");
	NodeHandle n;

	Publisher udometer_pub = n.advertise<acr_udometer::Udometer>("udometer", 5);

	Rate loop_rate(1.f/5.f);	// 5 seconds = 1/5 Hz
	int loopCount;
	while (ok()) {
		try{
			udometer_pub.publish(getBuienradarData(52, 4)); //rain data for Delft
			loopCount = 60;	// 5 minutes
		} catch( curlpp::RuntimeError &e ) {
			ROS_WARN("%s - You might not have a working internet connection or the service is down.", e.what());
			loopCount = 1;	// 5 seconds
		}

		spinOnce();
		
		// Sleep loopCount x 5 seconds to allow stopping the program every 5 seconds, instead of once per 5 minutes :P
		for(int i=0; i < loopCount && ok(); i++) {
			loop_rate.sleep();
		}
	}
  return 0;
}
