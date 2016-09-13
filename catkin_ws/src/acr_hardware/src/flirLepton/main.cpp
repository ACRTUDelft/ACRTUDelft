#include "main.hpp"

bool initLepton() {
    fd = open(device, O_RDWR);
    if (fd < 0)
        ROS_WARN("Can't open device");
    else if (-1 == ioctl(fd, SPI_IOC_WR_MODE, &mode))
        ROS_WARN("Can't set SPI mode");
    else if (-1 == ioctl(fd, SPI_IOC_RD_MODE, &mode))
        ROS_WARN("Can't get SPI mode");
    else if (-1 == ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits))
        ROS_WARN("Can't set bits per word");
    else if (-1 == ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits))
        ROS_WARN("Can't get bits per word");
    else if (-1 == ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed))
       ROS_WARN("Can't set max speed");
    else if (-1 == ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed))
        ROS_WARN("Can't get max speed");
    else
        return true;
    return false;
}

int getPacket(int iRow, unsigned char *packetData) {
    _tr.rx_buf = (unsigned long) packetData;
    return ioctl(fd, SPI_IOC_MESSAGE(1), &_tr);
}

int getFrame() {
	int errors = 0;
    int iRow;
	for (iRow = 0; iRow < FrameHeight; ) {
		unsigned char *packet = &result[iRow*RowPacketBytes];

		if (getPacket(iRow, packet) < 1) {
			ROS_WARN("Error transferring SPI packet");
			return 1	;
		}

		int packetNumber;
		if ((packet[0] & 0xf)==0xf)
			packetNumber = -1;
		else
			packetNumber = packet[1];

		if (packetNumber==-1) {
			usleep(1000);
			if (++errors > 300) break;
			continue;
		}

		if (packetNumber != iRow) {
			usleep(1000);
			break;
		}

		++iRow;
	}
	return iRow;
}

void convertData(uint16_t& minValue, uint16_t& maxValue) {
	unsigned char *in = &result[0];
	unsigned short *out = &rawData[0];
	for (int iRow = 0; iRow < FrameHeight; ++iRow) {
		in += 4;
		for (int iCol = 0; iCol < FrameWidth; ++iCol) {
			unsigned short value = in[0];
			value <<= 8;
			value |= in[1];
			in += 2;
			if (value > maxValue) maxValue = value;
			if (value < minValue) minValue = value;
			*(out++) = value;
		}
	}
}

sensor_msgs::Image createImage(uint16_t& minValue, uint16_t& maxValue) {
	int diff = maxValue - minValue + 1;
	for (int y = 0; y < FrameHeight; ++y) {
		for (int x = 0; x < FrameWidth; ++x) {
			int baseValue = rawData[FrameWidth*y + x]; // take input value in [0, 65536)
			procData[FrameWidth * y + x] = 256 * (baseValue - minValue)/diff;
		}
	}

	sensor_msgs::Image imageMsg;
	imageMsg.height = FrameHeight;
	imageMsg.width = FrameWidth;
	imageMsg.encoding = sensor_msgs::image_encodings::MONO8;
	imageMsg.is_bigendian = 0; // false
	imageMsg.step = FrameWidth;
	
	std::vector<unsigned char> image;
	for (unsigned char i : procData) {
		image.push_back(i);
	}
	imageMsg.data = image;
	return imageMsg;
}

void readCamera() {
	_tr.tx_buf = (unsigned long) &tx[0];
    _tr.len = RowPacketBytes;
    _tr.delay_usecs = delay;
    _tr.speed_hz = speed;
    _tr.bits_per_word = bits;

    int resets = 0; // Number of times we've reset the 0...59 loop for packets
    while (ros::ok()) {
        int iRow = getFrame();

        if (iRow < FrameHeight) {
            if (++resets >= 750) {
                ROS_WARN("Packet reset counter hit 750");
                resets = 0;
                usleep(750000);
            }
            continue;
        }

        resets = 0; 
		uint16_t minValue = 65535;
		uint16_t maxValue = 0;
		
		convertData(minValue, maxValue);
		
		if (!scale) {
			minValue = 7800;
			maxValue = 8300;
		}
		
		sensor_msgs::Image msg = createImage(minValue, maxValue);		
		pub.publish(msg);
	
        ros::spinOnce();
    }	
}

int main( int argc, char **argv ) {
	if(argc == 2 && strcmp(argv[1], "--scale") == 0) {
		scale = true;
		ROS_INFO("Color scaling enabled");
	} else {
		ROS_INFO("Color scaling disabled");
	}
	
	init(argc, argv, "flirLepton");
	NodeHandle nh;
	pub = nh.advertise<sensor_msgs::Image>("IR_heatmap", 5);
	
    if (!initLepton()) return 1;

    usleep(250000);

    readCamera();
}
