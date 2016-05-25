#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "diagnostic_msgs/KeyValue.h"

#include <stdio.h>
#include <stdlib.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string.h>

#define MODULES 3

using namespace ros;

int main(int argc, char **argv) {
	init(argc, argv, "WiFiControls");
	NodeHandle nh;

	Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
	Publisher module_pub = nh.advertise<diagnostic_msgs::KeyValue>("sensor_module", 5);
	
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
				if (n > 0) {
					std::string s = buffer;
									
					/* Convert dat to data array */
					std::string delimiter = "\n";
					std::string data [5];
					size_t pos = 0;
					std::string token;
					int c = 0;
					while ((pos = s.find(delimiter)) != std::string::npos) {
						token = s.substr(0, pos);
						data[c] = token;
						s.erase(0, pos + delimiter.length());
						c++;
						if(c > 4) c = 0;
					}
					
					/* Create twist message */
					geometry_msgs::Twist tmsg;
					 tmsg.linear.x = std::stof(data[0]);
					 tmsg.angular.z = std::stof(data[1]);
					twist_pub.publish(tmsg);
					
					/* Create module messages */
					for(int i = 2; i < 5; i++) {
						diagnostic_msgs::KeyValue msg;
						msg.key = "module:" + std::to_string(i);
						if(data[i].compare("true") == 0) {
							msg.value = std::to_string(3);		// MODULE_INTERACT
						} else {
							msg.value = std::to_string(2);		// MODULE_IDLE
						}
						module_pub.publish(msg);
					}			
					spinOnce();
				} else {
					ROS_WARN("Connection lost, please reconnect.");
					break;
				}							
			} catch (...) {
				ROS_WARN("Connection lost, please reconnect.");
				break;
			}
		}
	}
  return 0;
}
