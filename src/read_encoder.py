#!/usr/bin/env python

# This ROS node is meant to be run onboard the plane.
# It subscribes reads in the encoder angle from the arduino and 
# Publishes this info to a ros_topic "/gimbal_angle"

import serial as sr
import numpy as np
import rospy
from std_msgs.msg import Float64
import time

ser = sr.Serial('/dev/ttyUSB0', 115200)
ser.close()
ser.open()
ser.flush()

while True:
	#print ser.writable
	print ser.readline()
	