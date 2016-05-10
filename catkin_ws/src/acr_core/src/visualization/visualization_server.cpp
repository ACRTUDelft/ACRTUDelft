
#include "visualization_server.hpp"
#include "../consts.hpp"
#include <std_msgs/String.h>


ros::Publisher Visualization_Server::vis_pub;
visualization_msgs::Marker Visualization_Server::marker;


void Visualization_Server::start(ros::NodeHandle nodehandle){
	ROS_INFO("hallo");
	//nh =nodehandle;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 1;
	marker.pose.position.y = 1;
	marker.pose.position.z = 1;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	//only if using a MESH_RESOURCE marker type:
//	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	ROS_INFO("hallo2");
	vis_pub = nodehandle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	ROS_INFO("hallo3");
	vis_pub.publish(marker);
	ROS_INFO("hallo4");

	
}

void Visualization_Server::update(float x, float y, float z){
	marker.pose.position.x = marker.pose.position.x+ x;
	marker.pose.position.y = marker.pose.position.y + x;
	marker.pose.position.z = marker.pose.position.z + x;
	vis_pub.publish(marker);
}

