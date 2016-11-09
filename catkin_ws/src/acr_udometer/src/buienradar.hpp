#include "ros/ros.h"
#include "acr_udometer/Udometer.h"
#include <string>

/**
 * Parser for the Buienradar API data.
 * Creates a message that can be send by this node.
 * The message is of the following layout:
 *  currentRain: current rain in mm/h
 * 	changeTime: condition change time in minutes from now (5 minute precision) (-1 when no change ahead)
 * 	newRain: rain value after the change in mm/h
 * 
 * @param A std::string containing the data provided by the API.
 * @return A acr_udometer::Udometer message containing parsed weather data.
 */
acr_udometer::Udometer parseBuienradarData(std::string str);

/**
 * Get rain data from the Buienradar API.
 * Requests the page and parses it.
 * @return A acr_udometer::Udometer message containing parsed weather data.
 */
acr_udometer::Udometer getBuienradarData();

/**
 * Calculates the actual rain rate in mm/h from the data format used by the Buienradar API.
 * @param Integer value [0-255] as provided by the API.
 * @return The rain rate in mm/h rounded to two decimals.
 */
float buienradarCalc(int value) {
	float val = (value - 109) / 32.f;
	return roundf(pow(10.f, val) * 1000.f) / 1000.f;
}
