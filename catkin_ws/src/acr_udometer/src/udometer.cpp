#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * ROS node that sends weather information every 60 seconds on 'udometer'.
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "udometer");
	ros::NodeHandle n;

	ros::Publisher udometer_pub = n.advertise<std_msgs::String>("udometer", 5);

	ros::Rate loop_rate(1/60.f);

	while (ros::ok()) {    
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world ";
		msg.data = ss.str();
	
		ROS_INFO("%s", msg.data.c_str());
		udometer_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
  return 0;
}
