#include "udometer.hpp"
#include <sstream>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Options.hpp>

#define BUIENRADAR_TRESHOLD 3	// value that must be reached before it is registered as 'rain'.

/**
 * ROS node that sends weather information every 5 minutes on 'udometer'.
 * Uses a Buienradar API for gathering of the weather data.
 * Sends a 'geometry_msgs/Vector3' with:
 * 	x: current rain in mm/h
 * 	y: condition change time in minutes from now (5 minute precision)
 * 	z: rain value after the change in mm/h
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "udometer");
	ros::NodeHandle n;

	ros::Publisher udometer_pub = n.advertise<geometry_msgs::Vector3>("udometer", 5);

	ros::Rate loop_rate(1.f/5.f);	// 5 seconds = 1/5 Hz

	while (ros::ok()) {
		try{
			std::ostringstream os;		
			os << curlpp::options::Url("http://gps.buienradar.nl/getrr.php?lat=52&lon=4");	//rain data for Delft
			//ROS_INFO("%s", os.str().c_str());
		
			geometry_msgs::Vector3 msg = parseBuienradarData(os.str());	
			udometer_pub.publish(msg);
		} catch( curlpp::RuntimeError &e ) {
			ROS_WARN("%s - You might not have a working internet connection or the service is down.", e.what());
		}

		ros::spinOnce();
		
		// Sleep 60x 5 seconds to allow stopping the program every 5 seconds, instead of once per 5 minutes :P
		for(int i=0; i < 60 && ros::ok(); i++) {
			loop_rate.sleep();
		}
	}
  return 0;
}


























geometry_msgs::Vector3 parseBuienradarData(std::string str) {
	geometry_msgs::Vector3 msg = geometry_msgs::Vector3();
	msg.x = buienradarCalc(std::stoi(str.substr(0, 3)));
	 ROS_INFO("Current rain: %f mm/h", msg.x);
	
	int lines = str.length() / 11;
	for(int i = 0; i < lines; i++) {	// For each line
		int amount = std::stoi(str.substr(11*i, 3));
		if(msg.x > 0.f) {	// If it is raining
			if(amount == 0) {	// and becoming dry
				msg.y = i * 5.f;
				msg.z = 0;
				 ROS_INFO("\nWeather becomes dry in %i minutes.", (int) msg.y);
				break;
			}
		} else {	// If it is dry
			if(amount > BUIENRADAR_TRESHOLD) {	// and starts raining
				msg.y = i * 5.f;
				msg.z = buienradarCalc(amount);
				 ROS_INFO("\nIt will start raining (%f mm/h) in %i minutes.", msg.z, (int) msg.y);
				break;
			}
		}
	}
	return msg;
}


