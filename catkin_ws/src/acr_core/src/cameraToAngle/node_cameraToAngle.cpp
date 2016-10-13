#include "cameraToAngle.cpp"

#include <ros/ros.h> 
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Float32.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace ros;

Publisher pub;
Subscriber sub;

/**
 * Callback method for receiving images to process.
 * Calls the detector and publishes the results.
 */
void callback(const sensor_msgs::Image& input) {
	std_msgs::Float32 msg;
	msg.data = imageProcess(input);
	pub.publish(msg);
}

int main(int argc, char **argv) {
	initDetector(checkPreview(argc, argv));
		
	init(argc, argv, "camera_to_angle");
	NodeHandle nh;
	
	pub = nh.advertise<std_msgs::Float32>("sensor_camera", 1);
	sub = nh.subscribe("IR_heatmap", 100, callback);

	spin();
	return 0;
}
