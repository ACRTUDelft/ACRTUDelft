#!/usr/bin/env python
import rospy

from diagnostic_msgs.msg import KeyValue

import sys, select, termios, tty, os

header = """
Check the status of all the modules
-----------------------------------

CTRL-C to quit

Module\tStatus\tInteraction"""

modules = {
	'MODULE1':("-", "-"),
	'MODULE2':("-", "-"),
	'MODULE3':("-", "-"),
       	}

# Display the module states.
def display():
	os.system('clear')
	print(header)
	for mod in modules:
		print(mod + "\t" + modules[mod][0] + "\t" + modules[mod][1])

# Callback for when a message is recieved.
# Gets the module name and sets its values.
def moduleCallback(kval):
	modName = "MODULE" + str(int(kval.key.split(":")[1]) + 1)
	val = int(kval.value)
	mod = modules[modName]
	if (val == 0):
		modules[modName] = ("FULL", mod[1])
	elif (val == 1):
		modules[modName] = ("OK", mod[1])
	elif (val == 2):
		modules[modName] = (mod[0], "IDLE")
	elif (val == 3):
		modules[modName] = (mod[0], "INTERACT")
	else:
		rospy.warn("Invalid module status: " + str(val))
	display()
	
if __name__=="__main__":
	rospy.init_node("acr_module_display")
	rospy.Subscriber("sensor_module",KeyValue, moduleCallback)
	
	display()
	rospy.spin()
