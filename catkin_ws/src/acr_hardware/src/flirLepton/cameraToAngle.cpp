#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <opencv2/opencv.hpp> 

#define IMG_HEIGHT 60
#define IMG_WIDTH 80
#define FOV 25
#define PACKET_SIZE 164
#define PACKET_SIZE_UINT16 (PACKET_SIZE/2)
#define PACKETS_PER_FRAME 60
#define FRAME_SIZE_UINT16 (PACKET_SIZE_UINT16 * PACKETS_PER_FRAME)

using namespace ros;
using namespace cv;
using namespace std;

class cameraToAngle
{
public:
  cameraToAngle()
  {
    pub_ = nh.advertise<std_msgs::Float32>("sensor_camera", 10);
    sub_ = nh.subscribe("IR_heatmap", 5, &cameraToAngle::callback, this);
  }

  void callback(const sensor_msgs::Image::ConstPtr& input)
  {
SimpleBlobDetector detector;
vector<KeyPoint> keypoints;
detector.detect( input, keypoints);

 Float32 grootsteblob = 0;
 Float32 angle = 0;

for(std::vector<cv::KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++){
if(blobIterator->size>grootsteblob){
   grootsteblob=blobIterator->size;
 angle = (pt.x-IMG_WIDTH/2)/IMG_WIDTH*FOV;
}
} 

    pub_.publish(angle);
  }

private:
  NodeHandle nh; 
  Publisher pub_;
  Subscriber sub_;

};

int main(int argc, char **argv)
{
  //Initiate ROS
  init(argc, argv, "camera_To_Angle");

  //Create an object of class SubscribeAndPublish that will take care of everything
  cameraToAngle SAPObject;

  spin();

  return 0;
}
