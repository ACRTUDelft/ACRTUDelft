#!/usr/bin/env python
import rospy

from std_msgs.msg import Float32

import sys, select, termios, tty, os

charge = -1;
header = """
Check the status of the battery
-----------------------------------

CTRL-C to quit
"""

# Display the battery state.
def display(newCharge):
	global charge
	if (newCharge == charge):
		return
	charge = newCharge
	os.system('clear')
	print(header)
	print("Battery charge: " + str(100 * charge) + "%")

# Callback for when a message is recieved.
# Gets the module name and sets its values.
def moduleCallback(value):
	display(value.data)
	
if __name__=="__main__":
	rospy.init_node("battery_display")
	rospy.Subscriber("sensor_battery",Float32, moduleCallback)
	
	display(0.0)
	rospy.spin()
