#include "../src/buienradar.cpp"
#include <gtest/gtest.h>

TEST(UdometerTests, case1) {
	//parseBuienradarData("000|12:23\n");
	//ASSERT_EQ(1,2) << "1 is not equal to 2";
	//EXPECT_EQ(1,2) dows not terminate on failure
}


int main (int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
