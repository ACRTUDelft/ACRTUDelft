#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include <string>

/**
 * Parser for the Buienradar API data.
 * Creates a message that can be send by this node.
 * The message is of the following layout:
 *  x: current rain in mm/h
 * 	y: condition change time in minutes from now (5 minute precision) (-1 when no change ahead)
 * 	z: rain value after the change in mm/h
 * 
 * @param A std::string containing the data provided by the API.
 * @return A geometry_msgs/Vector3 message containing parsed weather data.
 */
geometry_msgs::Vector3 parseBuienradarData(std::string str);

/**
 * Get rain data from the Buienradar API.
 * Requests the page and parses it.
 * @return A geometry_msgs/Vector3 message containing parsed weather data.
 */
geometry_msgs::Vector3 getBuienradarData();

/**
 * Calculates the actual rain rate in mm/h from the data format used by the Buienradar API.
 * @param Integer value [0-255] as provided by the API.
 * @return The rain rate in mm/h rounded to two decimals.
 */
float buienradarCalc(int value) {
	float val = (value - 109) / 32.f;
	return roundf(pow(10.f, val) * 1000.f) / 1000.f;
}
