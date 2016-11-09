#include "ros/ros.h"
#include "acr_msgs/Ultrasonic.h"
#include "../settings.hpp"
#include "../pins.hpp"

#include <wiringPi.h>
#include <sys/time.h>

using namespace ros;

#define TIMEOUT 1000 				// timeout in ms

const int TRIG[] = {ULTRA_TRIG, ULTRA_TRIG, ULTRA_TRIG, ULTRA_TRIG};	// Trigger pins for the sensors
const int ECHO[] = {LU_ECHO, FTU_ECHO, FBU_ECHO, RU_ECHO};				// Echo pins for the sensors

Publisher pub;

/**
 * Check if a timeout happened from a given start time.
 */
bool timeout(long start) {
	return (micros() - start) > TIMEOUT * 1000;
}

/**
 * Measure the distance with the specified sensor.
 * Returns a float containing the measured distance in meters.
 * Returns -1 when a timout happens;
 */
float getDistance(int sensor) {
	digitalWrite(TRIG[sensor], LOW);
	usleep(1);	
	digitalWrite(TRIG[sensor], HIGH);
	usleep(20);
	digitalWrite(TRIG[sensor], LOW);
	
	long start = micros();
	while(digitalRead(ECHO[sensor]) == LOW) {	// wait for the echo pulse to arrive
		if (timeout(start)) return -1;
	}
	
	start = micros();
	while(digitalRead(ECHO[sensor]) == HIGH) {	// measure the echo duration
		if (timeout(start)) return -1;
	}
	
	long time = micros() - start;
	
	return time / 5800.0;	// convert to meters
}

/**
 * Make multiple measurements and calculate the average.
 * return -1, when any measurement fails, returns the average otherwise.
 */
float getAvgDistance(int sensor, int tries) {
	float total = 0;
	int timeout = 0;
	
	for (int i = 0; i < tries; i++)	{
		float tmp = getDistance(sensor);
		if (tmp == -1) {
			return -1;
		}
		total += tmp;
	}
	
	return total / tries;
}

/**
 * Set all the pins to input/output and reset the outputs
 */
void setupPins() {
	wiringPiSetupGpio();
	for(int i = 0; i < ULTRASONIC_SENSORS; i++) {
		pinMode(ECHO[i], INPUT);
		pinMode(TRIG[i], OUTPUT);
		digitalWrite(TRIG[i], LOW);
		usleep(10000);
	}
	
}

/**
 * Check if te range is between the boundries set by the hardware.
 * Returns a corrected distance when it was limited by these boundries, or the old value otherwise.
 */
float correctRange(int sensor,float dist) {
	if (dist == -1) {
		ROS_DEBUG("Sensor %d timed out", sensor);
		return ULTRASONIC_MAX_RANGE;
	}
	if (dist > ULTRASONIC_MAX_RANGE){
		ROS_DEBUG("Sensor %d measured above maximum range", sensor);
		return ULTRASONIC_MAX_RANGE;
	}
	if (dist < ULTRASONIC_MIN_RANGE){
		ROS_DEBUG("Sensor %d measured below minimal range", sensor);
		return ULTRASONIC_MIN_RANGE;
	}
	return dist;
}

/**
 * Send the distance to the ros system.
 */
void sendMessage(int sensor, float dist) {
	acr_msgs::Ultrasonic msg;
	 msg.sensor = sensor;
	 msg.range.min_range = ULTRASONIC_MIN_RANGE;
	 msg.range.max_range = ULTRASONIC_MAX_RANGE;
	 msg.range.field_of_view = ULTRASONIC_FIELD_OF_VIEW;
	 msg.range.range = dist;
	pub.publish(msg);
}

int main(int argc, char **argv) {
	setupPins();
	init(argc, argv, "ultrasonicSensors");
	NodeHandle nh;

	pub = nh.advertise<acr_msgs::Ultrasonic>("sensor_ultrasonic", 2 * ULTRASONIC_SENSORS);
	Rate loop_rate(10.f);	// 10 Hz
	
	while (ros::ok()) {
		for (int i = 0; i < ULTRASONIC_SENSORS; i++) {
			float dist = getAvgDistance(i, 3);
			 dist = correctRange(i, dist);
			sendMessage(i, dist);
		}
		spinOnce();		
		loop_rate.sleep();
	}			
  return 0;
}
