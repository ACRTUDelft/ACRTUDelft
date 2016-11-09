#ifndef TESTING
	#define TESTING
#endif

#include "std_msgs/Float32.h"
#include "acr_msgs/Ultrasonic.h"
#include "geometry_msgs/Vector3.h"
#include "acr_msgs/ModuleState.h"

#include "../../src/core/sensorData.cpp"
#include <gtest/gtest.h>

/**
 * Run all the test cases.
 */
int main (int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

void setRange(int sensor, float range) {
	acr_msgs::Ultrasonic msg;
	 msg.sensor = sensor;
	 msg.range.range = range;
	
	SensorData::ultrasonicCallback(msg);
}

void setAllRanges(float range) {
	setRange(0, range);
	setRange(1, range);
	setRange(2, range);
	setRange(3, range);
}

void setModuleState(int module, int state) {
	acr_msgs::ModuleState msg;
	 msg.module = module;
	 msg.state = state;
	
	SensorData::moduleCallback(msg);
}

void setAllModuleStates(int state) {
	setModuleState(0, state);
	setModuleState(1, state);
	setModuleState(2, state);
}

void setBattery(float charge) {
	std_msgs::Float32 msg;
	 msg.data = charge;
	 
	SensorData::batteryCallback(msg);
}

void setUdometer(float currentValue, int time, float newValue) {
	geometry_msgs::Vector3 msg;
	 msg.x = currentValue;
	 msg.y = time;
	 msg.z = newValue;
	 
	SensorData::udometerCallback(msg);
}

/*
 * Tests for isFree() method.
 */
  
TEST(testIsFree, testSensorPositiveBound) {
	setAllRanges(ULTRASONIC_MIN_DIST);
	ASSERT_FALSE(SensorData::isFree(4)) << "Should return false when sensor does not exist";
}

TEST(testIsFree, testSensorNegativeBound) {
	setAllRanges(ULTRASONIC_MIN_DIST);
	ASSERT_FALSE(SensorData::isFree(-1)) << "Should return false when sensor does not exist";
}

TEST(testIsFree, testSensorFree) {
	setRange(1, ULTRASONIC_MIN_DIST);
	ASSERT_TRUE(SensorData::isFree(1)) << "Should return true when free";
}

TEST(testIsFree, testSensorNotFree) {
	setRange(1, 0);
	ASSERT_FALSE(SensorData::isFree(1)) << "sensor should not be free when distance is 0";
}

/*
 * Tests for getDistance() method.
 */
 
TEST(testGetDistance, testSensorPositiveBound) {
	setAllRanges(ULTRASONIC_MIN_DIST);
	ASSERT_EQ(-1, SensorData::getDistance(4)) << "Should return -1 when sensor does not exist";
}

TEST(testGetDistance, testSensorNegativeBound) {
	setAllRanges(ULTRASONIC_MIN_DIST);
	ASSERT_EQ(-1, SensorData::getDistance(-1)) << "Should return -1 when sensor does not exist";
}

TEST(testGetDistance, testgetValue) {
	setRange(1, 1234);
	ASSERT_EQ(1234, SensorData::getDistance(1)) << "should return the value stored and not something else";
}

/*
 * Tests for isBatteryFull() method.
 */
 
TEST(testIsBatteryFull, testgetFull) {
	setBattery(1);
	ASSERT_TRUE(SensorData::isBatteryFull()) << "the battery should be full";
}

TEST(testIsBatteryFull, testgetEmpty) {
	setBattery(.2);
	ASSERT_FALSE(SensorData::isBatteryFull()) << "the battery should be empty when 20% charged";
}

TEST(testIsBatteryFull, testOvercharge) {
	setBattery(.2);
	setBattery(100); // overcharge
	ASSERT_FALSE(SensorData::isBatteryFull()) << "Setting batteries to overcharged values should not happen";
}

TEST(testIsBatteryFull, testUndercharge) {
	setBattery(1);
	setBattery(-2); // overcharge
	ASSERT_TRUE(SensorData::isBatteryFull()) << "Setting batteries to undercharged values should not happen";
}

/*
 * Tests for needsService() method.
 */

TEST(testNeedsService, testModulePositiveBound) {
	setAllModuleStates(acr_msgs::ModuleState::MODULE_OK);
	ASSERT_FALSE(SensorData::needsService(4)) << "Should return false when module does not exist";
}

TEST(testNeedsService, testmoduleNegativeBound) {
	setAllModuleStates(acr_msgs::ModuleState::MODULE_OK);
	ASSERT_FALSE(SensorData::needsService(-1)) << "Should return false when module does not exist";
}

TEST(testNeedsService, testModuleService) {
	setModuleState(acr_msgs::ModuleState::MODULE1, acr_msgs::ModuleState::MODULE_FULL);
	ASSERT_TRUE(SensorData::needsService(acr_msgs::ModuleState::MODULE1)) << "Should return true when the module needs service";
}

TEST(testNeedsService, testModuleOK) {
	setModuleState(acr_msgs::ModuleState::MODULE1, acr_msgs::ModuleState::MODULE_OK);
	ASSERT_FALSE(SensorData::needsService(acr_msgs::ModuleState::MODULE1)) << "Sensor should not need service when in OK state";
}

TEST(testNeedsService, testModuleIdle) {
	setModuleState(acr_msgs::ModuleState::MODULE1, acr_msgs::ModuleState::MODULE_FULL);
	setModuleState(acr_msgs::ModuleState::MODULE1, acr_msgs::ModuleState::MODULE_IDLE);
	ASSERT_TRUE(SensorData::needsService(acr_msgs::ModuleState::MODULE1)) << "MODULE_IDLE should not update the service state";
}

TEST(testNeedsService, testModuleNegative) {
	setModuleState(acr_msgs::ModuleState::MODULE1, acr_msgs::ModuleState::MODULE_FULL);
	setModuleState(acr_msgs::ModuleState::MODULE1, -1);
	ASSERT_TRUE(SensorData::needsService(acr_msgs::ModuleState::MODULE1)) << "A negative state should not update the service state";
}

/*
 * Tests for pointOfInterest() method.
 */
 
TEST(testPointOfInterest, testGetPoI) {
	float angle = 100.12;

	std_msgs::Float32 msg;
	 msg.data = angle;
	SensorData::angleofInterestCallback(msg);

	ASSERT_EQ(angle, SensorData::pointOfInterest()) << "the output should be the same as the input";
}

/*
 * Tests for udometer methods.
 */

TEST(testUdometer, testIsRainingFalse) {
	setUdometer(0, 0, 0);
	ASSERT_FALSE(SensorData::isRaining()) << "it is not raining";
}

TEST(testUdometer, testIsRainingTrue) {
	setUdometer(1, 0, 0);
	ASSERT_TRUE(SensorData::isRaining()) << "it is not raining";
}

TEST(testUdometer, testNextRainWhenRaining) {
	setUdometer(1, 0, 0);
	ASSERT_EQ(0, SensorData::nextRain()) << "it is already raining";
}

TEST(testUdometer, testNextRain) {
	setUdometer(0, 20, 10);
	ASSERT_EQ(20, SensorData::nextRain()) << "it starts in 20 mins, not anything else";
}

TEST(testUdometer, testNextRainIntensityWhenRaining) {
	setUdometer(100, 0, 35);
	ASSERT_EQ(100, SensorData::nextRainIntensity()) << "it is already raining";
}

TEST(testUdometer, testNextRainIntensity) {
	setUdometer(0, 20, 10);
	ASSERT_EQ(10, SensorData::nextRainIntensity()) << "it starts in 20 mins, not anything else";
}
