#include <ros/ros.h>
#include <opencv2/opencv.hpp> 
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Float32.h"
#include <cv_bridge/cv_bridge.h>
#define IMG_HEIGHT 60
#define IMG_WIDTH 80
#define FOV 25

using namespace ros;
using namespace cv;
using namespace std;

class cameraToAngle
{
public:
  cameraToAngle()
  {
    //Topic you want to publish
    pub_ = n_.advertise<std_msgs::Float32>("angleOfInterest", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("IR_heatmap", 5, &cameraToAngle::callback, this);
  }

  void callback(const sensor_msgs::Image& input)
  {
    std_msgs::Float32 output;
    float angle = 0;

// Detect blobs.
vector<KeyPoint> keypoints;


cv_bridge::CvImagePtr cv_ptr;
cv_ptr = cv_bridge::toCvCopy(input);
Mat image = cv_ptr->image;


// Setup SimpleBlobDetector parameters.
SimpleBlobDetector::Params params;
 
// Change thresholds
params.minThreshold = 10;
params.maxThreshold = 200;
 
// Filter by Circularity
params.filterByCircularity = true;
params.minCircularity = 0.1;
 
// Filter by Convexity
params.filterByConvexity = true;
params.minConvexity = 0.87;
 
// Filter by Inertia
params.filterByInertia = true;
params.minInertiaRatio = 0.01;

  SimpleBlobDetector detector(params);
detector.detect( image, keypoints);

 float grootsteblob = 0;

for(std::vector<cv::KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++){
if(blobIterator->size>grootsteblob){
   grootsteblob=blobIterator->size;
 angle = (blobIterator->pt.x-IMG_WIDTH/2)/IMG_WIDTH*FOV;
}
} 
output.data = angle;
    pub_.publish(output);
  }

private:
  NodeHandle n_; 
  Publisher pub_;
  Subscriber sub_;

};

int main(int argc, char **argv)
{
  //Initiate ROS
  init(argc, argv, "camera_to_angle");

  //Create an object of class SubscribeAndPublish that will take care of everything
  cameraToAngle SAPObject;

  spin();

  return 0;
}
