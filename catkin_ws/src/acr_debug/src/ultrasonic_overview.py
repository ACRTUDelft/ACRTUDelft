#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Range

import sys, select, termios, tty, os

header = """
Check the range measured by the ultrasonic sensors
-----------------------------------

CTRL-C to quit

"""

sensors = [float('NaN'), float('NaN'), float('NaN'), float('NaN')]

# Display the module states.
def display():
	os.system('clear')
	print(header)
	print("U_LEFT:\t\t" + str(sensors[0]))
	print("U_FRONT_TOP:\t" + str(sensors[1]))
	print("U_FRONT_BOTTOM:\t" + str(sensors[2]))
	print("U_RIGHT:\t" + str(sensors[3]))

# Callback for when a message is recieved.
# Gets the module name and sets its values.
def moduleCallback(msg):
	sensor = msg.radiation_type
	if (sensor > 3) or (sensor < 0):
		rospy.warn("Invalid sensor: " + str(val))
	sensors[sensor] = msg.range
	display()
	
if __name__=="__main__":
	rospy.init_node("ultrasonic_display")
	rospy.Subscriber("sensor_ultrasonic",Range, moduleCallback)
	
	display()
	rospy.spin()
