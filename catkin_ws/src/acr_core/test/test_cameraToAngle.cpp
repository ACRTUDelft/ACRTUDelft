#ifndef TESTING
	#define TESTING
#endif

#include "../src/cameraToAngle/cameraToAngle.cpp"
#include <gtest/gtest.h>

const sensor_msgs::Image loadImage(std::string imagePath) {
	Mat image = imread(imagePath, CV_LOAD_IMAGE_GRAYSCALE);
	if(image.empty()) {
		perror("Error loading image");
	}
	cv_bridge::CvImage out_msg;
	out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	out_msg.image    = image; // Your cv::Mat

	return *out_msg.toImageMsg();
}

/**
 * Run all the test cases.
 */
int main (int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

TEST(testCheckPreview, testparseArgsPreview) {
	char arg1[] = "Test";
	char arg2[] = "--preview";
	char* argv[] = {arg1, arg2};
	ASSERT_TRUE(checkPreview(2, argv)) << "Preview not enabled with --preview as second argument";
}

TEST(testCheckPreview, testparseArgsMorePreview) {
	char arg1[] = "Test";
	char arg2[] = "--preview";
	char arg3[] = "Test2";
	char* argv[] = {arg1, arg2, arg3};
	ASSERT_TRUE(checkPreview(3, argv)) << "Preview not enabled with --preview as second argument and an additional third one";
}

TEST(testCheckPreview, testparseArgsTypo) {
	char arg1[] = "Test";
	char arg2[] = "--peview";
	char* argv[] = {arg1, arg2};
	ASSERT_FALSE(checkPreview(2, argv)) << "Preview enabled with something different than --preview as argument";
}

TEST(testCheckPreview, testparseArgsProgramName) {
	char arg1[] = "--peview";
	char* argv[] = {arg1};
	ASSERT_FALSE(checkPreview(1, argv)) << "Preview enabled when program is called --preview";
}

TEST(testImageProcess, left) {
	initDetector(false);
	float res = imageProcess(loadImage("test/testLeft.png"));
	ASSERT_GE(0, res) << "The correct angle was not found";
}

TEST(testImageProcess, right) {
	initDetector(false);
	float res = imageProcess(loadImage("test/testRight.png"));
	ASSERT_LE(0, res) << "The correct angle was not found";
}

TEST(testImageProcess, empty) {
	initDetector(false);
	float res = imageProcess(loadImage("test/testEmpty.png"));
	ASSERT_TRUE(std::isnan(res)) << "The correct angle was not found";
}

TEST(testInitDetector, initPreviewTrue) {
	showPreview = false;
	initDetector(true);
	ASSERT_TRUE(showPreview);
}

TEST(testInitDetector, initPreviewFalse) {
	showPreview = true;
	initDetector(false);
	ASSERT_FALSE(showPreview);
}

TEST(testCalculateAngle, left) {
	float res = -FOV / 2.0;
	ASSERT_EQ(res, calculateAngle(0));
}

TEST(testCalculateAngle, right) {
	float res = FOV / 2.0;
	ASSERT_EQ(res, calculateAngle(IMG_WIDTH));
}

TEST(testCalculateAngle, middle) {
	ASSERT_EQ(0, calculateAngle(IMG_WIDTH/2.0));
}
