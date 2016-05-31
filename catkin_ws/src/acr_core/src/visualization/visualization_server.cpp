
#include "visualization_server.hpp"
#include "../consts.hpp"
#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>


ros::Publisher Visualization_Server::vis_pub;
visualization_msgs::Marker Visualization_Server::marker;int x =0;
visualization_msgs::Marker Visualization_Server::obstacle;

void movementCallBack(const geometry_msgs::Twist& msg) {
		//ROS_INFO("%f, %f", msg.linear.x, msg.angular.z);
		Visualization_Server::update(msg.linear.x, msg.angular.z);
	}

ros::Subscriber sub;
float Visualization_Server::angular =	0;


void Visualization_Server::start(ros::NodeHandle nodehandle){
//	obstacle = createObstacle();
	createMarker();
	createObstacle();	
	ROS_INFO("hallo");
	//nh =nodehandle;

	//only if using a MESH_RESOURCE marker type:
//	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	ROS_INFO("hallo2");
	vis_pub = nodehandle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	ROS_INFO("hallo3");
//	vis_pub.publish(marker);
	vis_pub.publish(obstacle);
	ROS_INFO("hallo4");
	sub = nodehandle.subscribe("cmd_vel", 10, movementCallBack);

	
}


void Visualization_Server::update(float linear, float ang){
	angular += ang;
	ROS_INFO("%f", angular		);
	marker.pose.position.x = marker.pose.position.x+ sin(angular)*linear;
	marker.pose.position.y = marker.pose.position.y + cos(angular)*linear;
	marker.pose.orientation.z = sin(-(angular/2)+45);
	marker.pose.orientation.w = cos(-(angular/2)+45);
	//marker.pose.orientation.y = marker.pose.orientation.y + cos(angular)*linear;
	//marker.pose.position.z = marker.pose.position.z + x;
//	vis_pub.publish(marker);
	vis_pub.publish(obstacle);
}

void Visualization_Server::createMarker() {
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
	marker.scale.x = 0.3;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
}

void Visualization_Server::createObstacle() {
	obstacle.header.frame_id = "base_link";
	obstacle.header.stamp = ros::Time();
	obstacle.ns = "my_namespace";
	obstacle.id = 1;
	obstacle.type = visualization_msgs::Marker::CUBE;
	obstacle.action = visualization_msgs::Marker::ADD;
	obstacle.pose.position.x = 3;
	obstacle.pose.position.y = 1;
	obstacle.pose.position.z = 1;
	obstacle.pose.orientation.x = 0.0;
	obstacle.pose.orientation.y = 0.0;
	obstacle.pose.orientation.z = 0.0;
	obstacle.pose.orientation.w = 1.0;
	obstacle.scale.x = 1;
	obstacle.scale.y = 1;
	obstacle.scale.z = 1;
	obstacle.color.a = 1.0; // Don't forget to set the alpha!
	obstacle.color.r = 0.0;
	obstacle.color.g = 1.0;
	obstacle.color.b = 0.0;	
}




