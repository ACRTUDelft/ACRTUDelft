#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <stdio.h>
#include <stdlib.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string.h>

using namespace ros;

int main(int argc, char **argv) {
	init(argc, argv, "WiFiControls");
	NodeHandle nh;

	Publisher wifiC_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
	
	int sockfd, newsockfd, portno;
	unsigned int clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int  n;
   
	/* setup socket */
	sockfd = socket(AF_INET, SOCK_STREAM, 0);

	if (sockfd < 0) {
	  ROS_WARN("ERROR opening socket");
	  exit(1);
	}

	/* Initialize socket structure */
	bzero((char *) &serv_addr, sizeof(serv_addr));	//clear the server address
	portno = 6000;

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(portno);

	/* Bind the host address */
	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
	  ROS_WARN("ERROR on binding");
	  exit(1);
	}

	listen(sockfd,5);
	clilen = sizeof(cli_addr);
		
	ROS_INFO("Waiting for connection..");

	while (ok()) {   
		
	struct timeval tv;	// Timeout length
	tv.tv_sec = 5;
    tv.tv_usec = 0;	
    		
		fd_set rfds;
		FD_ZERO(&rfds);
		FD_SET(sockfd, &rfds);
    
		int retval = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
	    if (retval == -1)
			ROS_WARN("select()");
		else if (retval == 0) {	// Timeout
			continue;
		}
		newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	
		if (newsockfd < 0) {
		  ROS_WARN("ERROR on accept");
		  exit(1);
		}
		
		ROS_INFO("Socket connected");
		while(ok()) {
			try {
				bzero(buffer,256);
				n = read( newsockfd,buffer,255 );

				if (n < 0) {
				  ROS_WARN("ERROR reading from socket");
				  break;
				}
				
				std::string data = buffer;				
				size_t br = data.find('\n', 0);
				
				/* Create ROS message */
				geometry_msgs::Twist msg;
				 msg.linear.x = std::stof(data.substr(0, br));
				 msg.angular.z = -std::stof(data.substr(br, 256));
				
				wifiC_pub.publish(msg);			
				spinOnce();
			} catch (Exception &e) {
				ROS_WARN("Connection lost");
				break;
			}
		}
	}
  return 0;
}
