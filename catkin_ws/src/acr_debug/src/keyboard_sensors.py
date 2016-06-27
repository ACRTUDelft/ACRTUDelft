#!/usr/bin/env python
import rospy

from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from diagnostic_msgs.msg import KeyValue

import sys, select, termios, tty, math
import numpy as np

msg = """
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
		'r':(1.1),
		'f':(0.9),
		}

batModifiers = {
		't':(1.1),
		'g':(0.9),
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
		print msg
		poi = float('nan')

		ult_max = 10000
		ult_dist = 1999
		ult = [ult_max, ult_max, ult_max, ult_max]
		mod = [1, 1, 1]
		bat_lvl = 1.0
		while(1):
			key = getKey()
			if (key == '\x03'):
				break

			# Point of Interest
			if key in poiBindings.keys():
				poi = poiBindings[key]
			
			# Ultrasonic distance
			if key in distModifiers.keys():
				ult_dist *= distModifiers[key]
				ult_dist = min(ult_dist, ult_max)
				sys.stdout.write("\033[K")
				print("\rRange: " + str(ult_dist))

			# Battery levels
			if key in batModifiers.keys():
				bat_lvl *= batModifiers[key]
				bat_lvl = min(bat_lvl, 1)
				sys.stdout.write("\033[K")
				print("\rBattery: " + str(100 * bat_lvl) + "%")

			# Ultrasonic trigger
			ult = [ult_max, ult_max, ult_max, ult_max]
			if key in sensor_ultrasonic.keys():
				ult[sensor_ultrasonic[key]] = ult_dist
				
			# Module service
			if key in sensor_modules.keys():
				op = sensor_modules[key]
				mod[op[0]] = op[1]
			
			# Send module messages	
			for i, d in enumerate(mod):
				kval = KeyValue()
				kval.key = "module:" + str(i)
				kval.value = str(mod[i])
				pub_mod.publish(kval)

			# Send ultrasonic distance messages	
			for i, d in enumerate(ult):
				ultRange = Range()
				ultRange.radiation_type = i
				ultRange.range = ult[i]
				pub_ult.publish(ultRange)

			# Send battery levels	
			batFloat = Float32()
			batFloat.data = bat_lvl
			pub_bat.publish(batFloat)	
			
			
			# Send Point of Interest	
			poiFloat = Float32()
			poiFloat.data = poi
			pub_poi.publish(poiFloat)

	except Exception, e:
		print e 

	finally:
		poiFloat = Float32()
		poiFloat.data = float('nan')
		pub_poi.publish(poiFloat)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


