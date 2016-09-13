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
    pub_ = n_.advertise<std_msgs::Float32>("sensor_camera", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("IR_heatmap", 100, &cameraToAngle::callback, this);
  }

  void callback(const sensor_msgs::Image& input)
  {
    std_msgs::Float32 output;

// Detect blobs.
vector<KeyPoint> keypoints;


cv_bridge::CvImagePtr cv_ptr;
cv_ptr = cv_bridge::toCvCopy(input);
Mat image = cv_ptr->image;


// Setup SimpleBlobDetector parameters.
SimpleBlobDetector::Params params;
params.maxArea = 20000;
params.filterByCircularity = false;
params.filterByConvexity = false;
params.minArea = 256;
params.filterByInertia = false;
params.blobColor = 255;

 

  SimpleBlobDetector detector(params);
detector.detect( image, keypoints);

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

Mat im_with_keypoints;
drawKeypoints( image, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
 imshow("plaatje", im_with_keypoints );

output.data = angle;
    pub_.publish(output);
ROS_INFO("%f",angle);
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
cvNamedWindow( "plaatje" );
startWindowThread ();
  //Create an object of class SubscribeAndPublish that will take care of everything
  cameraToAngle SAPObject;

  spin();

  return 0;
}
