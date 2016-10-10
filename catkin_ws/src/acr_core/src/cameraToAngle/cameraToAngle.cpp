#include <ros/ros.h> 
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#define IMG_HEIGHT 60
#define IMG_WIDTH 80
#define FOV 25
#define PREVIEW_SCREEN "cameraToAngle | preview"

using namespace cv;
using namespace std;

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
 * Calculate the relative angle from the x position in the image.
 * NOTE: this value should not be interpreted as an angle, but seen as a relative position.
 */
float calculateAngle(float x) {
	return (x - IMG_WIDTH/2) / IMG_WIDTH * FOV;
}

/**
 * Get the biggest blob from a list of blobs and calculates its relative angle.
 * Returns NaN if no blob is found.
 */
float getAngle(vector<KeyPoint>& keypoints) {
	float biggestBlob = 0;
	float interest_x = 0;
	float interest_y = 0;
	float angle = NAN;

	for(vector<KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++){
		if(blobIterator->size > biggestBlob){
			biggestBlob = blobIterator->size;
			angle = calculateAngle(blobIterator->pt.x);
		}
	}
	return angle;
}

/**
 * Callback method for receiving images to process.
 * Detects blobs in the received image, selects the best one and returns the relative angle with this blob.
 */
float imageProcess(const sensor_msgs::Image& input) {
	Mat image;
	vector<KeyPoint> keypoints;	
	detectBlobs(input, image, keypoints);
	
	if (showPreview) {
		Mat im_with_keypoints;
		drawKeypoints( image, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
		imshow(PREVIEW_SCREEN, im_with_keypoints );
	}
	
	return getAngle(keypoints);
}

/**
 * Setup the blob detector and its parameters.
 */
void initDetector(bool preview) {
	showPreview = preview;
	
	SimpleBlobDetector::Params params;
	 params.maxArea = 20000;
	 params.filterByCircularity = false;
	 params.filterByConvexity = false;
	 params.minArea = 256;
	 params.filterByInertia = false;
	 params.blobColor = 255;

	detector = SimpleBlobDetector(params);
	
	if (showPreview) {
#ifndef TESTING
		cvNamedWindow(PREVIEW_SCREEN);
		startWindowThread ();
#endif
		ROS_INFO("Preview window enabled");
	} else {
		ROS_INFO("Preview window disabled");
	}
}

/**
 * Check if the preview must be shown.
 */
bool checkPreview(int argc, char **argv) {
	return argc > 1 && (strcmp(argv[1], "--preview") == 0);
}
