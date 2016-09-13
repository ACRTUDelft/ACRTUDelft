#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <vector> 

enum {
	FrameWidth = 80,
	FrameHeight = 60,
	RowPacketWords = FrameWidth + 2,
	RowPacketBytes = 2*RowPacketWords,
	FrameWords = FrameWidth*FrameHeight
};

using namespace ros;

int fd;
struct spi_ioc_transfer _tr;

const char *device = "/dev/spidev0.1"; // Change to 0.0 if necessary!
unsigned char mode = 0, bits = 8;
unsigned int speed = 16000000;
unsigned short delay = 0;
bool scale = false;

std::vector<unsigned char> tx(RowPacketBytes);
std::vector<unsigned char> result(RowPacketBytes*FrameHeight);
std::vector<unsigned short> rawData(FrameWords);
std::vector<unsigned char> procData(FrameWords);
Publisher pub;

/**
 * Initializes the spi interface.
 * Returns false when an error occurs
 */
bool initLepton();

/**
 * Receives one packet from the spi interface.
 */
int getPacket(int iRow, unsigned char *packetData);

/**
 * Constructs one frame from multiple packets.
 */
int getFrame();

/**
 * Converts the received data to the raw image.
 * Also calculates the minimal and maximum vlaues in the data.
 */
void convertData(uint16_t& minValue, uint16_t& maxValue);

/**
 * Creates the ros image object by converting the existing image to MONO8.
 */ 
sensor_msgs::Image createImage(uint16_t& minValue, uint16_t& maxValue);

/**
 * Reads the camera in a loop and sends the recorded image.
 */
void readCamera();

int main( int argc, char **argv );
