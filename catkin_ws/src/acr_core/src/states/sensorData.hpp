#ifndef SensorData_H
#define SensorData_H

#include "ros/ros.h"
using namespace ros;

/* Wrapper class for the sensor data.
 * Receives the ROS messages from all sensors.
 * Also sends messages to the engines and modules.
 */
class SensorData {
  public:
	
	/* Returns the distance that the selected ultrasonic sensor receives.
	 * If the sensor is free returns -1, else returns the measured distance.
	 * If the sensor does not exist, the method returns 0.
	 */
	static float isFree(int sensor) {
		switch(sensor) {
			case U_LEFT: 			return -1;
			case U_FRONT_TOP: 		return -1;
			case U_FRONT_BOTTOM:	return -1;
			case U_RIGHT: 			return -1;
		}
		ROS_WARN("Undefined sensor %d!", sensor);
		return 0;
	}

	/* Method for checking the battery status.
	 * Returns true when the battery is full, else otherwise.
	 */ 
	static bool isBatteryFull() {
		return true;
	}

	/* Method to check if the selected module raised its service flag.
	 * Returns true when the flag is raised, false otherwise.
	 * Returns false when the module does not exist.
	 */
	static bool needsService(int module) {
		switch(module) {
			case MOD1: 	return false;
			case MOD2: 	return false;
			case MOD3:	return false;
		}
		ROS_WARN("Undefined module %d!", module);
		return false;
	}

	/* Method to check if the robot has a found a point of interest (e.g. a human).
	 * Returns true when a PoI is found, false otherwise.
	 */
	static bool seesPointOfInterest() {
		return false;
	}

	/* Method that returns the realtive angular position of the PoI.
	 * Negative values are to the left, 0 is in the front and positive numbers indicate to the right.
	 * NOTE: The behavior is undefined when 'seesPointOfInterest() == false'
	 */
	static float pointOfInterest() {
		return 0.f;
	}
	
	/* Method called when the node starts.
	 * starts receiving messages from the sensors.
	 */
	static void init(NodeHandle nh) {
		//Publisher udometer_pub = n.advertise<geometry_msgs::Vector3>("udometer", 5);
		//TODO
	}
};
#endif
