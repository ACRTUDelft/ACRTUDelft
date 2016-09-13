#include <ros/ros.h> 
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Float32.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#define IMG_HEIGHT 60
#define IMG_WIDTH 80
#define FOV 25
#define PREVIEW_SCREEN "cameraToAngle | preview"

using namespace ros;
using namespace cv;
using namespace std;


Publisher pub;
Subscriber sub;
bool showPreview = false;
SimpleBlobDetector detector;

/**
 * Detect blobs in the image.
 * Uses the cv_bridge to convert the images.
 */
void detectBlobs(const sensor_msgs::Image& input, Mat& image, vector<KeyPoint>& keypoints) {
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(input);
	image = cv_ptr->image;

	detector.detect(image, keypoints);
}

/**
 * Get the biggest blob from a list of blobs and calculates its relative angle.
 * Returns NaN if no blob is found.
 */
float getAngle(vector<KeyPoint>& keypoints) {
	float grootsteblob = 0;
	float interest_x = 0;
	float interest_y = 0;
	float angle = NAN;

	for(vector<KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++){
		if(blobIterator->size>grootsteblob){
			grootsteblob=blobIterator->size;
			angle = (blobIterator->pt.x-IMG_WIDTH/2)/IMG_WIDTH*FOV;
		}
	}
	return angle;
}

/**
 * Callback method for receiving images to process.
 * Detects blobs in the received image, selects the best one and returns the relative angle with this blob.
 */
void callback(const sensor_msgs::Image& input) {
	std_msgs::Float32 output;

	Mat image;
	vector<KeyPoint> keypoints;	
	detectBlobs(input, image, keypoints);
	
	if (showPreview) {
		Mat im_with_keypoints;
		drawKeypoints( image, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
		imshow(PREVIEW_SCREEN, im_with_keypoints );
	}
	
	output.data = getAngle(keypoints);
	pub.publish(output);
}

/**
 * Setup the blob detector and its parameters.
 */
void setupDetector() {
	SimpleBlobDetector::Params params;
	 params.maxArea = 20000;
	 params.filterByCircularity = false;
	 params.filterByConvexity = false;
	 params.minArea = 256;
	 params.filterByInertia = false;
	 params.blobColor = 255;

	detector = SimpleBlobDetector(params);
}

int main(int argc, char **argv) {
	if(argc == 2 && strcmp(argv[1], "--preview") == 0) {
		showPreview = true;
		ROS_INFO("Preview window enabled");
	} else {
		ROS_INFO("Preview window disabled");
	}
		
	init(argc, argv, "camera_to_angle");
	NodeHandle nh;
	
	pub = nh.advertise<std_msgs::Float32>("sensor_camera", 1);
	sub = nh.subscribe("IR_heatmap", 100, callback);
	setupDetector();
	
	if (showPreview) {
		cvNamedWindow(PREVIEW_SCREEN);
		startWindowThread ();
	}

	spin();
	return 0;
}
