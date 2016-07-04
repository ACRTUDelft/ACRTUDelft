#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#include <wiringPi.h>
#include <sys/time.h>

#define SENSORS 4
#define MIN_RANGE 0.02
#define MAX_RANGE 4.0
#define FIELD_OF_VIEW 0.523598776	// 30 degrees in radians

#define TIMEOUT 100 				// timeout in ms

const int TRIG[] = {1, 2, 3, 4};	// Trigger pins for the sensors
const int ECHO[] = {5, 6, 7, 8};	// Echo pins for the sensors

using namespace ros;

/**
 * Check if a timeout happened from a given start time.
 */
bool timeout(struct timeval start) {
	struct timeval now;
	gettimeofday(&now, NULL);
	return (now.tv_usec - start.tv_usec) > TIMEOUT * 1000;
}

/**
 * Measure the distance with the specified sensor.
 * Returns a float containing the measured distance in meters.
 * Returns -1 when a timout happens;
 */
float getDistance(int sensor) {
	digitalWrite(TRIG[sensor], LOW);
	usleep(5);
	digitalWrite(TRIG[sensor], HIGH);
	usleep(10);
	digitalWrite(TRIG[sensor], LOW);
	
	struct timeval  start, end;
	gettimeofday(&start, NULL);
	 while(digitalRead(ECHO[sensor] == LOW) && !timeout(start))	// wait for response
	gettimeofday(&end, NULL);
	float time = (end.tv_usec - start.tv_usec) / 1000000 + (end.tv_sec - start.tv_sec);
	if (time > TIMEOUT / 1000.f) return -1;
	return time * 171.5; // convert to meters
}

/**
 * Set all the pins to input/output and reset the outputs
 */
void setupPins() {
	wiringPiSetup();
	for(int i = 0; i < SENSORS; i++) {
		pinMode(ECHO[i], INPUT);
		pinMode(TRIG[i], OUTPUT);
		digitalWrite(TRIG[i], LOW);
	}
}

int main(int argc, char **argv) {
	setupPins();
	init(argc, argv, "ultrasonicSensors");
	NodeHandle nh;

	Publisher pub = nh.advertise<sensor_msgs::Range>("sensor_ultrasonic", 2 * SENSORS);
	Rate loop_rate(10.f);	// 10 Hz
	
	while(ros::ok()) {
		for(int i = 0; i < SENSORS; i++) {
			float dist = getDistance(i);
			if(dist == -1) {
				ROS_DEBUG("Sensor %d timed out", i);
				continue;
			}
			sensor_msgs::Range msg;
			msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
			msg.min_range = MIN_RANGE;
			msg.max_range = MAX_RANGE;
			msg.field_of_view = FIELD_OF_VIEW;
			msg.range = dist;
			pub.publish(msg);
		}
		spinOnce();		
		loop_rate.sleep();
	}			
  return 0;
}
