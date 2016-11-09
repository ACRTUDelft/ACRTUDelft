#ifndef SensorData_H
#define SensorData_H

#include "../settings.hpp"

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "acr_msgs/Ultrasonic.h"
#include "acr_msgs/ModuleState.h"

#include <string>

using namespace ros;

/* Wrapper class for the sensor data.
 * Receives the ROS messages from all sensors.
 * Also sends messages to the engines and modules.
 */
class SensorData {
	static float batteryCharge;
	
	static float angleOfInterest;
	
	static float uDist[ULTRASONIC_SENSORS];
	static float mStat[MODULES];
	
	static float rain_current;
	static int rain_change;
	static float rain_next;
	
	
	
  public:
	/* Callback for receiving messages about an object of interest.
	 * Stores the horizontal angle towards the object.
	 */
	static void angleofInterestCallback(const std_msgs::Float32& msg) {
		angleOfInterest = msg.data;
	}
	
	/* Callback for receiving messages from the udometer.
	 */
	static void udometerCallback(const geometry_msgs::Vector3& msg) {
		rain_current = msg.x;
		rain_change = msg.y;
		rain_next = msg.z;
	}
	
	/* Callback for receiving battery status updates.
	 * Stores the the received status.
	 * The value is between 0 (empty) and 1 (full).
	 */
	static void batteryCallback(const std_msgs::Float32& msg) {	
		float charge = msg.data;
		if(charge > 1.f || charge < 0.f) {
			ROS_WARN("Invalid charge %.2f, the value must be between 0 and 1", charge);
			return;
		}
		batteryCharge = charge;
	}
	
	/* Callback for range measurements.
	 * Only stores the received range.
	 */
	static void ultrasonicCallback(const acr_msgs::Ultrasonic& msg) {	
		int sensor = msg.sensor;
		if (sensor < 0 || sensor >= ULTRASONIC_SENSORS) {
			ROS_WARN("Unknown sensor %d", sensor);
			return;
		}
		uDist[sensor] = (msg.range.range < ULTRASONIC_MIN_DIST) ? 0 : msg.range.range;
	}
	
	/* Callback for messages from the modules.
	 * If the value is MODULE_OK or MODULE_FULL, the status is stored.
	 */
	static void moduleCallback(const acr_msgs::ModuleState& msg) {
		int module = msg.module;
		
		if (module < 0 || module >= MODULES) {
			ROS_WARN("Invalid module id %d", module);
			return;
		}
		
		int status = msg.state;
		if(status != acr_msgs::ModuleState::MODULE_OK && status != acr_msgs::ModuleState::MODULE_FULL) return; // Wrong types
		mStat[module] = status;
	}
	
	/* Returns true when it is raining, false otherwise
	 */
	static bool isRaining();
	
	/* Returns the time in minutes when it starts raining.
	 * This method returns 0 when it is already raining.
	 */
	static int nextRain();
	
	/* Returns the intensity of the next shower in mm/h.
	 * Note that when it is already raining, this method returns the current intensity.
	 */
	static float nextRainIntensity();
	
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
