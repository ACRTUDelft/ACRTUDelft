#include "../src/buienradar.cpp"
#include <gtest/gtest.h>

/**
 * Test the 'buienradarCalc()' method with value 77
 */
TEST(UdometerTests, testFormula77) {
	 ASSERT_EQ(0.1f, buienradarCalc(77));
}

/**
 * Test the 'buienradarCalc()' method with value 0
 */
TEST(UdometerTests, testFormula0) {
	 ASSERT_EQ(0.000f, buienradarCalc(0));
}

/**
 * Test the 'buienradarCalc()' method with value 255
 */
TEST(UdometerTests, testFormula255) {
	 ASSERT_EQ(36517.412f, buienradarCalc(255));
}
	
/**
 * Test for a weather situation without a change starting with clear weather.
 */
TEST(UdometerTests, noChange1) {
	geometry_msgs::Vector3 msg = parseBuienradarData("000|12:20\r\n000|12:25\r\n000|12:30\r\n000|12:35");
	EXPECT_EQ(0, msg.x) << "current rain was not 0, but was " << msg.x;
	EXPECT_EQ(-1, msg.y) << "changing time was not -1, but was " << msg.y;
	EXPECT_EQ(0, msg.z) << "new rain value was not 0, but was " << msg.z;
}

/**
 * Test for a weather situation without a change starting with rainy weather.
 */
TEST(UdometerTests, noChange2) {
	geometry_msgs::Vector3 msg = parseBuienradarData("020|12:20\r\n050|12:25\r\n127|12:30\r\n011|12:35");
	EXPECT_EQ(2, roundf(1000.f * msg.x))<< "current rain was not 0.002, but was " << msg.x;
	EXPECT_EQ(-1, msg.y) << "changing time was not -1, but was " << msg.y;
	EXPECT_EQ(0, msg.z) << "new rain value was not 0, but was " << msg.z;
}

/**
 * Test for a dry weather situation that changes to rainy weather.
 */
TEST(UdometerTests, dryToRain) {
	geometry_msgs::Vector3 msg = parseBuienradarData("000|12:20\r\n000|12:25\r\n090|12:30\r\n230|12:35");
	EXPECT_EQ(0, msg.x)<< "current rain was not 0, but was " << msg.x;
	EXPECT_EQ(10, msg.y) << "changing time was not 15, but was " << msg.y;
	EXPECT_EQ(255, roundf(1000.f * msg.z)) << "new rain value was not 0, but was " << msg.z;
}

/**
 * Test for a rainy weather situation that changes to dry weather.
 */
TEST(UdometerTests, rainToDry) {
	geometry_msgs::Vector3 msg = parseBuienradarData("090|12:20\r\n010|12:25\r\n000|12:30\r\n000|12:35");
	EXPECT_EQ(255, roundf(1000.f * msg.x))<< "current rain was not 0, but was " << msg.x;
	EXPECT_EQ(10, msg.y) << "changing time was not 15, but was " << msg.y;
	EXPECT_EQ(0, msg.z) << "new rain value was not 0, but was " << msg.z;
}

/**
 * Run all the test cases.
 */
int main (int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
