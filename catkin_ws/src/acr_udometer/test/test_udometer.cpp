#include "../src/buienradar.cpp"
#include <gtest/gtest.h>

	//ASSERT_EQ(1,2) << "1 is not equal to 2";
	//EXPECT_EQ(1,2) dows not terminate on failure
	
/**
 * Test for a weather situation without a change starting with clear weather.
 */
TEST(UdometerTests, noChange1) {
	geometry_msgs::Vector3 msg = parseBuienradarData("000|12:20\n000|12:25\n000|12:30\n000|12:35");
	EXPECT_EQ(0, msg.x) << "current rain was not 0, but was " << msg.x;
	EXPECT_EQ(-1, msg.y) << "changing time was not -1, but was " << msg.y;
	EXPECT_EQ(0, msg.z) << "new rain value was not 0, but was " << msg.z;
}

/**
 * Test for a weather situation without a change starting with rainy weather.
 */
TEST(UdometerTests, noChange2) {
	geometry_msgs::Vector3 msg = parseBuienradarData("020|12:20\n050|12:25\n127|12:30\n011|12:35");
	EXPECT_EQ(2, roundf(1000.f * msg.x))<< "current rain was not 0.002, but was " << msg.x;
	EXPECT_EQ(-1, msg.y) << "changing time was not -1, but was " << msg.y;
	EXPECT_EQ(0, msg.z) << "new rain value was not 0, but was " << msg.z;
}

/**
 * Test for a dry weather situation that changes to rainy weather.
 */
TEST(UdometerTests, dryToRain) {
	geometry_msgs::Vector3 msg = parseBuienradarData("000|12:20\n000|12:25\n090|12:30\n230|12:35");
	EXPECT_EQ(0, msg.x)<< "current rain was not 0, but was " << msg.x;
	EXPECT_EQ(15, msg.y) << "changing time was not 15, but was " << msg.y;
	EXPECT_EQ(12345, roundf(1000.f * msg.z)) << "new rain value was not 0, but was " << msg.z;
}

/**
 * Test for a rainy weather situation that changes to dry weather.
 */
TEST(UdometerTests, rainToDry) {
	geometry_msgs::Vector3 msg = parseBuienradarData("090|12:20\n010|12:25\n000|12:30\n000|12:35");
	EXPECT_EQ(12345, roundf(1000.f * msg.x))<< "current rain was not 0, but was " << msg.x;
	EXPECT_EQ(15, msg.y) << "changing time was not 15, but was " << msg.y;
	EXPECT_EQ(0, msg.z) << "new rain value was not 0, but was " << msg.z;
}

/**
 * Run all the test cases.
 */
int main (int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
