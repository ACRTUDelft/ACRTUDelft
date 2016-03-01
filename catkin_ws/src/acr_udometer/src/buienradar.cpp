#include "buienradar.hpp"
#include <sstream>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Options.hpp>

#define BUIENRADAR_TRESHOLD 10	// Value that must be reached before it is registered as 'rain'.

geometry_msgs::Vector3 getBuienradarData(int lat, int lon) {
	std::ostringstream os;		
	os << curlpp::options::Url("http://gps.buienradar.nl/getrr.php?lat=" + std::to_string(lat) + "&lon=" + std::to_string(lon));
	//ROS_INFO("%s", os.str().c_str());
		
	geometry_msgs::Vector3 msg = parseBuienradarData(os.str());	
}


geometry_msgs::Vector3 parseBuienradarData(std::string str) {
	geometry_msgs::Vector3 msg = geometry_msgs::Vector3();
	msg.x = buienradarCalc(std::stoi(str.substr(0, 3)));
	 ROS_INFO("Current rain: %f mm/h", msg.x);
	msg.y = -1; // default
	
	int lines = str.length() / 11;
	for(int i = 1; i < lines; i++) {	// For each line
		int amount = std::stoi(str.substr(11*i, 3));
		if(msg.x > 0.f) {	// If it is raining			
			if(amount < BUIENRADAR_TRESHOLD) {	// and becoming dry
				msg.y = i * 5.f;
				msg.z = 0;
				 ROS_INFO("Weather becomes dry in %i minutes.", (int) msg.y);
				break;
			}
		} else {	// If it is dry
			if(amount > BUIENRADAR_TRESHOLD) {	// and starts raining
				msg.y = i * 5.f;
				msg.z = buienradarCalc(amount);
				 ROS_INFO("It will start raining (%f mm/h) in %i minutes.", msg.z, (int) msg.y);
				break;
			}
		}
	}
	if(msg.y == -1) ROS_INFO("No change in weather for a while..");
	return msg;
}
