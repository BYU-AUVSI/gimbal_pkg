#!/usr/bin/env python

# This ROS node is meant to be run onboard the plane.
# It subscribes to /gimbal_cmd (commanded angles in radians)
# and writes them via serial to the arduino.

import serial as sr
import numpy as np
import math as m
import rospy, tf
from std_msgs.msg import Float64, Bool

import time, struct
import sys

# Init global variables (Object Oriented Programming would be better)
pitch = 0.0
tmp = 0.0
bomb = False

# Callback Function for /gimbal_cmd subscriber
def callback(msg):
	global pitch
	pitch = msg.data

# Callback Function for /bomd_drop subscriber
def bomb_callback(msg):
	global bomb
	bomb = msg.data


def main():
	global tmp, bomb
	angle_tmp = Float64()

	# Init ROS node and subscriber
	rospy.init_node('Gimbal_ros_to_arduino', anonymous=True)
	rospy.Subscriber('/gimbal_cmd', Float64, callback)
	rospy.Subscriber('/bomb_drop', Bool, bomb_callback)
	pub = rospy.Publisher('/gimbal_angle', Float64, queue_size=1)

	print "========== gps_gimbal_pointing started =========="
	arduino = sr.Serial('/dev/ttyUSB0')     # dummy serial port to get rid of garbage values left over
	with arduino:
	    arduino.setDTR(False)
	    time.sleep(1)
	    arduino.flushInput()
	    arduino.setDTR(True)

	ser = sr.Serial('/dev/ttyUSB0', 57600) # set up new, clean serial port

	try:
		while not rospy.is_shutdown(): 
			if tmp != pitch or bomb: # check to see if commanded pitch is new
				pitch_cmd = round(pitch, 2)
				pitch_cmd = pitch_cmd*(180.0/np.pi) # convert to degrees
				print "commanded pitch", pitch_cmd
				bin = struct.pack('f', pitch_cmd, bomb) # setup bits to write
				for b in bin:       # float 4 bytes
					ser.write(b) # write command to arduino
				time.sleep(0.2)
				tmp = pitch # save last commanded pitch to check against
			angle = ser.readline()
			angle = float(angle)
			if (angle != angle_tmp):
				angle_cal = angle - 68
				if angle_cal > 180.0:
					angle_cal -= 360
				print 'Encoder Angle', angle_cal
				angle_tmp = angle
				pub.publish(angle_cal)

	except rospy.ROSInterruptException:
		print "exiting...."
		ser.flushInput()
		ser.flushOutput()
		ser.close()
		print "serial port closed"
		return

if __name__ == '__main__':
	main()
