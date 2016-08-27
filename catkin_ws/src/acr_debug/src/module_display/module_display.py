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
	if (val == 0 and modules[modName][0] != "FULL"):
		modules[modName] = ("FULL", mod[1])
		display()
	elif (val == 1 and modules[modName][0] != "OK"):
		modules[modName] = ("OK", mod[1])
		display()
	elif (val == 2 and modules[modName][1] != "IDLE"):
		modules[modName] = (mod[0], "IDLE")
		display()
	elif (val == 3 and modules[modName][1] != "INTERACT"):
		modules[modName] = (mod[0], "INTERACT")
		display()
	elif (val > 3 or val < 0):
		rospy.warn("Invalid module status: " + str(val))
	
if __name__=="__main__":
	rospy.init_node("module_display")
	rospy.Subscriber("sensor_module",KeyValue, moduleCallback)
	
	display()
	rospy.spin()
