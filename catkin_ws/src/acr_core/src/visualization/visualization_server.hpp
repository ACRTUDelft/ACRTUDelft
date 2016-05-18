#ifndef Visualization_Server_H
#define Visualization_Server_H

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>


class Visualization_Server {
  public:
  
	static ros::Publisher vis_pub;
	static visualization_msgs::Marker marker;	
	static void start(ros::NodeHandle nodehandle);	

	static void update(float linear, float angular); 
	static float angular;			
 
};

#endif
