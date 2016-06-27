#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "diagnostic_msgs/KeyValue.h"

#include <stdio.h>
#include <stdlib.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string.h>
#include <vector>

#include "consts.hpp"

#define MODULES 3
#define PORT 6000

using namespace ros;
using namespace std;

/**
 * Split the input string to an array using a delimiter.
 */
vector<string> split(const string& str, const char& ch) {
	string next;
	vector<string> result;

	for (string::const_iterator it = str.begin(); it != str.end(); it++) {
		if (*it == ch) {
			if (!next.empty()) {
				result.push_back(next);
				next.clear();
			}
		} else {
			next += *it;
		}
	}
	if (!next.empty()) {
		result.push_back(next);
	}
	return result;
}

/**
 * Parse the received data and publish the messages.
 */
void parseMessage(char buffer[], Publisher twist_pub, Publisher module_pub) {
	std::vector<string> data = split(buffer, '\n');						
					
	/* Create twist message */
	geometry_msgs::Twist tmsg;
	 tmsg.linear.x = std::stof(data[0]);
	 tmsg.angular.z = -std::stof(data[1]);
	twist_pub.publish(tmsg);
	
	/* Create module messages */
	for(int i = 2; i < 5; i++) {
		diagnostic_msgs::KeyValue msg;
		msg.key = "module:" + std::to_string(i - 2);
		if(data[i].compare("true") == 0) {
			msg.value = std::to_string(MODULE_INTERACT); 
		} else {
			msg.value = std::to_string(MODULE_IDLE);
		}
		module_pub.publish(msg);
	}			
}

int main(int argc, char **argv) {
	init(argc, argv, "WiFiControls");
	NodeHandle nh;

	Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
	Publisher module_pub = nh.advertise<diagnostic_msgs::KeyValue>("sensor_module", 5);
	
	int sockfd, newsockfd;
	unsigned int clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
   
	/* setup socket */
	sockfd = socket(AF_INET, SOCK_STREAM, 0);

	if (sockfd < 0) {
	  ROS_WARN("ERROR opening socket");
	  exit(1);
	}

	/* Initialize socket structure */
	bzero((char *) &serv_addr, sizeof(serv_addr));	//clear the server address

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(PORT);

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
		tv.tv_sec = 1;
		tv.tv_usec = 0;	
    		
		fd_set rfds;
		FD_ZERO(&rfds);
		FD_SET(sockfd, &rfds);
    
		int retval = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
	    if (retval == -1) {
			ROS_WARN("Connection closed");
			return 1;
		} else if (retval == 0) {	// timeout
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
				int n = read( newsockfd,buffer,255 );

				if (n < 0) {
				  ROS_WARN("ERROR reading from socket");
				  break;
				} else if (n > 0) {
					parseMessage(buffer, twist_pub, module_pub);
					spinOnce();
				} else {
					ROS_WARN("Connection lost, please reconnect.");
					break;
				}							
			} catch (...) {
				ROS_WARN("Error while processing the data, please reconnect.");
				break;
			}
		}
	}
  return 0;
}
