#include "buienradar.hpp"
#include <sstream>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Options.hpp>

#define BUIENRADAR_TRESHOLD 10	// Value that must be reached before it is registered as 'rain'.

acr_udometer::Udometer getBuienradarData(int lat, int lon) {
	std::ostringstream os;		
	os << curlpp::options::Url("http://gps.buienradar.nl/getrr.php?lat=" + std::to_string(lat) + "&lon=" + std::to_string(lon));
		
	return parseBuienradarData(os.str());	
}


acr_udometer::Udometer parseBuienradarData(std::string str) {
	acr_udometer::Udometer msg = acr_udometer::Udometer();
	msg.currentRain = buienradarCalc(std::stoi(str.substr(0, 3)));
	 ROS_INFO("Current rain: %f mm/h", msg.currentRain);
	msg.changeTime = -1; // default
	
	int lines = str.length() / 11;
	for(int i = 1; i < lines; i++) {	// For each line
		int amount = std::stoi(str.substr(11*i, 3));
		if(msg.currentRain > 0.f) {	// If it is raining			
			if(amount < BUIENRADAR_TRESHOLD) {	// and becoming dry
				msg.changeTime = i * 5.f;
				msg.newRain = 0;
				 ROS_INFO("Weather becomes dry in %i minutes.", (int) msg.changeTime);
				break;
			}
		} else {	// If it is dry
			if(amount > BUIENRADAR_TRESHOLD) {	// and starts raining
				msg.changeTime = i * 5.f;
				msg.newRain = buienradarCalc(amount);
				 ROS_INFO("It will start raining (%f mm/h) in %i minutes.", msg.newRain, (int) msg.changeTime);
				break;
			}
		}
	}
	if(msg.changeTime == -1) ROS_INFO("No change in weather for a while..");
	return msg;
}
