#ifndef SensorData_C
#define SensorData_C

#include "sensorData.hpp"

using namespace ros;

Publisher twist_pub;
Publisher module_pub;

float SensorData::batteryCharge = 0.f;

float SensorData::angleOfInterest = NAN;

float SensorData::uDist[4] = {0, 0, 0, 0};
float SensorData::mStat[3] = {MODULE_OK, MODULE_OK, MODULE_OK};

float SensorData::rain_current = 0;
int SensorData::rain_change = 0;
float SensorData::rain_next = 0;
	
bool SensorData::isFree(int sensor) {
	if(sensor > 3 || sensor < 0) {
		ROS_WARN("Undefined sensor %d!", sensor);
		return false;
	}
	return uDist[sensor] > 0;
}

float SensorData::getDistance(int sensor) {
	if(sensor > 3 || sensor < 0) {
		ROS_WARN("Undefined sensor %d!", sensor);
		return -1;
	}
	return uDist[sensor];
}

bool SensorData::isBatteryFull() {
	return batteryCharge > 0.2f;
}

bool SensorData::needsService(int module) {
	if(module > 2) {
		ROS_WARN("Undefined module %d!", module);
		return false; 
	}
	return (mStat[module] == MODULE_FULL);
}

float SensorData::pointOfInterest() {
	return angleOfInterest;
}

void SensorData::sendModule(int module, int moduleState) {
	diagnostic_msgs::KeyValue msg = diagnostic_msgs::KeyValue();
	 msg.key = "module:" + std::to_string(module);
	 msg.value = std::to_string(moduleState);
	module_pub.publish(msg);
}

void SensorData::sendTwist(float angular, float linear) {
	geometry_msgs::Twist msg = geometry_msgs::Twist();
	 msg.angular.z = angular;
	 msg.linear.x = linear;	
	twist_pub.publish(msg);	
}

void SensorData::init(NodeHandle nh) {
	twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
	module_pub = nh.advertise<diagnostic_msgs::KeyValue>("sensor_module", 5);
}

bool SensorData::isRaining() {
	return rain_current > 0;
}
	
int SensorData::nextRain() {
	if (isRaining()) return 0;
	return rain_change;
}
	
float SensorData::nextRainIntensity(){
	if (isRaining()) return rain_current;
	return rain_next;
}
#endif
