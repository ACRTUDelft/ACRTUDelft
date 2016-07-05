#!/usr/bin/env python
import rospy

from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from diagnostic_msgs.msg import KeyValue

import sys, select, termios, tty, math, os
import numpy as np

header = """
Reading from the keyboard and Publishing to acr messages!
---------------------------
Point of interest:

w : straight
a : to the left
d : to the right
s : none

---------------------------
Trigger the ultrasonic sensors:

1-4 : module 1-4
r : increase range
f : decrease range

---------------------------
Trigger module OK/FULL:

u/j : module 1
i/k : module 2
o/l : module 3

---------------------------
Change battery level

t : increase battery level
g : decrease battery level

CTRL-C to quit
"""

# Key mappings

poiBindings = {
		's':(float('nan')),
		'a':(-1.0),
		'w':(0),
		'd':(1.0),
	       	}

distModifiers = {
		'r':(25),
		'f':(-25),
		}

batModifiers = {
		't':(.05),
		'g':(-.05),
		}

sensor_ultrasonic = {
		'1':(0),
		'2':(1),
		'3':(2),
		'4':(3),
	      	}

sensor_modules={
		'u':(0, 1),
		'i':(1, 1),
		'o':(2, 1),
		'j':(0, 0),
		'k':(1, 0),
		'l':(2, 0),
	      	}

# Display the header, the battery level and the range
def printAll(ult_range, bat_lvl):
	os.system('clear')
	for _ in range(1, 10):
		print
	print(header)
	print("\rBattery: " + str(100 * bat_lvl) + "%")
	print("\rRange: " + str(ult_range) + " cm")

# Send Float32 messages
def sendFloat32(pub, fl):	
	msg = Float32()
	msg.data = fl
	pub.publish(msg)

# Send keyValue messages
def sendKeyVal(pub, key, val):	
	kval = KeyValue()
	kval.key = key
	kval.value = val
	pub.publish(kval)

# Send Range messages
def sendRange(pub, radiation_type, r):
	msg = Range()
	msg.radiation_type = radiation_type
	msg.range = r
	pub.publish(msg)

# get the pressed key from the console
def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub_poi = rospy.Publisher('sensor_camera', Float32, queue_size = 1)
	pub_ult = rospy.Publisher('sensor_ultrasonic', Range, queue_size = 4)
	pub_mod = rospy.Publisher('sensor_module', KeyValue, queue_size = 4)
	pub_bat = rospy.Publisher('sensor_battery', Float32, queue_size = 1)

	rospy.init_node('acr_keyboard')

	try:
		ult_max = 500
		ult_dist = 100
		ult = [ult_max, ult_max, ult_max, ult_max]
		bat_lvl = 1.0
		printAll(ult_dist, bat_lvl)
		while(1):
			key = getKey()
			if (key == '\x03'):
				break

			# Point of Interest
			if key in poiBindings.keys():
				sendFloat32(pub_poi, poiBindings[key])
				continue
			
			# Ultrasonic distance
			if key in distModifiers.keys():
				ult_dist += distModifiers[key]
				ult_dist = max(0, min(ult_dist, ult_max))
				printAll(ult_dist, bat_lvl)
				continue

			# Battery levels
			if key in batModifiers.keys():
				bat_lvl += batModifiers[key]
				bat_lvl = max(0, min(bat_lvl, 1))
				printAll(ult_dist, bat_lvl)			
				sendFloat32(pub_bat, bat_lvl)
				continue

			# Ultrasonic trigger
			if key in sensor_ultrasonic.keys():
				sensor = sensor_ultrasonic[key]
				if (ult[sensor] == ult_dist):
					ult[sensor] = ult_max
				else :
					ult[sensor] = ult_dist
				sendRange(pub_ult, sensor, ult[sensor])
				continue					
				
			# Module service
			if key in sensor_modules.keys():
				op = sensor_modules[key]
				sendKeyVal(pub_mod, "module:" + str(op[0]), str(op[1]))
				continue

	except Exception, e:
		print e 

	finally:
		sendFloat32(pub_poi, float('NaN'))
		sendFloat32(pub_bat, 1.0)
		for i in range(0, 3):
			sendRange(pub_ult, i, ult_max)
		for i in range(0, 2):
			sendKeyVal(pub_mod, "module:" + str(i), '1')
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
