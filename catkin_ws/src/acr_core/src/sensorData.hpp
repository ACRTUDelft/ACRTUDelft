#ifndef SensorData_H
#define SensorData_H

#include "consts.hpp"

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float32.h"
#include "diagnostic_msgs/KeyValue.h"
#include "geometry_msgs/Twist.h"

#include <string>

#define ULTRASONIC_MIN_DIST 2000

using namespace ros;

/* Wrapper class for the sensor data.
 * Receives the ROS messages from all sensors.
 * Also sends messages to the engines and modules.
 */
class SensorData {
	static float batteryCharge;
	
	static float angleOfInterest;
	
	static float uDist[4];
	static float mStat[3];
	
	
	
  public:
	/* Callback for receiving messages about an object of interest.
	 * Stores the horizontal angle towards the object.
	 */
	static void angleofInterestCallback(const std_msgs::Float32& msg) {
		angleOfInterest = msg.data;
	}
	
	/* Callback for receiving battery status updates.
	 * Stores the the received status.
	 * The value is between 0 (empty) and 1 (full).
	 */
	static void batteryCallback(const std_msgs::Float32& msg) {		
		batteryCharge = msg.data;
	}
	
	/* Callback for range measurements.
	 * Only stores the received range.
	 * 'radiation_type' is reused to represent the sensor that measured the range.
	 */
	static void ultrasonicCallback(const sensor_msgs::Range& msg) {		
		uDist[msg.radiation_type] = msg.range < ULTRASONIC_MIN_DIST ? 0 : msg.range;
	}
	
	/* Callback for messages from the modules.
	 * If the value is MODULE_OK or MODULE_FULL, the status is stored.
	 * The key needs to be of the following format: 'module:#'.
	 */
	static void moduleCallback(const diagnostic_msgs::KeyValue& msg) {
		char* tmp = strdup(msg.key.c_str());
		strtok(tmp, ":");
		 int module = std::stoi(strtok(NULL, ":"));	
		delete tmp;
		
		int status = std::stoi(msg.value);
		 if(status > 1) return; // Wrong types
		mStat[module - 1] = status;
	}
	
	/* Returns true if the selected ultrasonic sensor is free.
	 * If the sensor does not exist, the method returns false.
	 */
	static bool isFree(int sensor);

	/* Returns the distance measured by the selected sensor.
	 * -1 is returned when the sensor does not exist.
	 */
	static float getDistance(int sensor);
	
	/* Method for checking the battery status.
	 * Returns true when the battery is full, else otherwise.
	 */ 
	static bool isBatteryFull();

	/* Method to check if the selected module raised its service flag.
	 * Returns true when the flag is raised, false otherwise.
	 * Returns false when the module does not exist.
	 */
	static bool needsService(int module);
	
	/* Method that returns the realtive angular position of a PoI.
	 * Negative values are to the left, 0 is in the front and positive numbers indicate to the right.
	 * NaN is returned when there is no point of interest.
	 */
	static float pointOfInterest();
	
	/* Send a twist message on 'cmd_vel'.
	 * This message has an angular.z of angular and linear.x of linear.
	 */
	static void sendTwist(float angular, float linear);
	
	/* Sends a key-value pair on 'sensor_module'.
	 * This message binds the given moduleState to the given module.
	 */
	static void sendModule(int module, int moduleState);
	
	/* Method called when the node starts.
	 * starts receiving messages from the sensors.
	 */
	static void init(NodeHandle nh);
	
};
#endif
