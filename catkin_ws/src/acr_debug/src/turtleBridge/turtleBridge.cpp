#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"

using namespace ros;

Publisher pub;

/** 
 * Callback method for received messages.
 * Converts the message to the correct format and publishes it to the node.
 */
void msgCallback(const geometry_msgs::Twist& msg) {
  geometry_msgs::Twist relayMsg;
  relayMsg.linear.x = msg.linear.x;
  relayMsg.angular.z = msg.angular.z / 5.0;
  
  pub.publish(relayMsg);		
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "turtleBridge");
  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe("cmd_vel", 10, msgCallback);
  pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 5);
  ros::spin();

  return 0;
}
