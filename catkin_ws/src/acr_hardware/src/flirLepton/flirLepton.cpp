#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include "SPI.cpp"

#define PACKET_SIZE 164
#define PACKET_SIZE_UINT16 (PACKET_SIZE/2)
#define PACKETS_PER_FRAME 60
#define FRAME_SIZE_UINT16 (PACKET_SIZE_UINT16 * PACKETS_PER_FRAME)

#define IMG_HEIGHT 60
#define IMG_WIDTH 80

using namespace ros;

int main( int argc, char **argv ){

	init(argc, argv, "FlirLepton");
	NodeHandle nh;

	Publisher pub = nh.advertise<sensor_msgs::Image>("IR_heatmap", 5);
	
	std::vector<uint8_t> image;
	uint8_t result[PACKET_SIZE * PACKETS_PER_FRAME];
	
	sensor_msgs::Image imageMsg;
	imageMsg.height = IMG_HEIGHT;
	imageMsg.width = IMG_WIDTH;
	imageMsg.encoding = sensor_msgs::image_encodings::MONO8;
	imageMsg.is_bigendian = 0; // false
	imageMsg.step = IMG_WIDTH;

	//open spi port
	SpiOpenPort(0);

	while(ros::ok()) {
		uint16_t *frame;
		
		for(int j=0; j<PACKETS_PER_FRAME; j++) {
			//if it's a drop packet, reset j to 0, set to -1 so he'll be at 0 again loop
			read(spi_cs0_fd, result + sizeof(uint8_t)*PACKET_SIZE*j, sizeof(uint8_t)*PACKET_SIZE);
			int packetNumber = result[j*PACKET_SIZE+1];
			if(packetNumber != j) {
				ROS_WARN("Package lost!");
			}
		}

		frame = (uint16_t *)result;
		uint16_t value;
		uint16_t minValue = 65535;
		uint16_t maxValue = 0;

		
		for(int i = 2; i < FRAME_SIZE_UINT16; i++) {
			//flip the MSB and LSB at the last second
			int temp = result[i*2];
			result[i*2] = result[i*2+1];
			result[i*2+1] = temp;
			
			value = frame[i];
			if(value > maxValue) {
				maxValue = value;
			}
			if(value < minValue) {
				minValue = value;
			}
		}

		float diff = maxValue - minValue;
		float scale = 255/diff;
		
		for(int i = 2; i < FRAME_SIZE_UINT16; i++) {
			value = (frame[i] - minValue) * scale;
			image[i-2] = (uint8_t) value;			
		}
		imageMsg.data = image;
		pub.publish(imageMsg);
	}
	SpiClosePort(0);
}
