#ifndef TESTING
	#define TESTING
#endif

#include "../../src/cameraToAngle/cameraToAngle.cpp"
#include <gtest/gtest.h>

#define LEFT_IMAGES  1
#define RIGHT_IMAGES 1
#define EMPTY_IMAGES 1

std:: string getImagePath(std:: string folder, int id) {
	return "test/test_cameraToAngle/images/" + folder + "/" + std::to_string(id) + ".png";
}

const sensor_msgs::Image loadImage(std::string filePath) {
	Mat image = imread(filePath, CV_LOAD_IMAGE_GRAYSCALE);
	if(image.empty()) {
		std::string err = "Error loading image " + filePath;
		perror(err.c_str());
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
	for (int i = 0; i < LEFT_IMAGES; i++) {
		std::string file  = getImagePath("left", i);
		float res = imageProcess(loadImage(file));
		ASSERT_GE(0, res) << "'images/left/" << i << ".png' should return NaN";
	}
}

TEST(testImageProcess, right) {
	initDetector(false);
	for (int i = 0; i < RIGHT_IMAGES; i++) {
		std::string file  = getImagePath("right", i);
		float res = imageProcess(loadImage(file));
		ASSERT_LE(0, res) << "'images/right/" << i << ".png' should return NaN";
	}
}

TEST(testImageProcess, empty) {
	initDetector(false);
	for (int i = 0; i < EMPTY_IMAGES; i++) {
		std::string file  = getImagePath("none", i);
		float res = imageProcess(loadImage(file));
		ASSERT_TRUE(std::isnan(res)) << "'images/none/" << i << ".png' should return NaN";
	}
	
	
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
