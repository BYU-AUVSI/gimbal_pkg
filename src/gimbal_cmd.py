#!/usr/bin/env python

# This ROS Node creates a slider that publishes the value to /gimbal_cmd topic
# The node is meant to be run on a ground computer to control the pitch of the gimbal.

import numpy as np
import matplotlib.pyplot as plt 
from matplotlib.widgets import Slider
from std_msgs.msg import Float64
import rospy

class Sliders:

	def __init__(self):
		# Creates a figure and axes for the Sliders.
		self.fig,self.ax = plt.subplots() 
		plt.axis("off")               # Suppress the axis from showing.

		# SECTION 1
		###############################################################
		self.num_of_sliders = 1 # The number of Slider objects that
								# will be used. The minimum value is 1
		###############################################################

		''' Down below you will create Slider objects for each slider 
			you need. The format for each object is as follows:
			self.Object1 = Slider(self.num_of_sliders, slider_number, 
				maxV = 1,minV = -1,intV = 0, gain = 1, name = 'Slider')
			self.Object2 = Slider(self.num_of_sliders, slider_number,
				maxV = 1,minV = -1,intV = 0, gain = 1, name = 'Slider')            
				...
			self.ObjectN = Slider(self.num_of_sliders, slider_number, 
				maxV = 1, minV = -1,intV = 0, gain = 1, name = 'Slider')
			For a description of each parameter, see the Slider Class 
			below.'''


		# SECTION 2
		################################################################
		# Instantiate objects here
		self.Spitch = mySlider(self.num_of_sliders,1,80,-10,0,np.pi/180.0,'Pitch_r')
		# self.Syaw = mySlider(self.num_of_sliders,2,30,-30,0,np.pi/180.0,'Yaw_r')
		################################################################

		plt.draw()


	def getInputValues(self):
		''' This function is called to return the sliders' values. 
			The variable values is a list that contains the values of 
			all of the sliders. You will need to insure that this 
			function returns the values in the following format.
			values = [self.object1.getValue(),
				self.object2.getValue(),...,
				self.ObjectN.getValue] '''

		# SECTION 3
		################################################################              
		values = self.Spitch.getValue()
		################################################################
		return values

class mySlider:

	def __init__(self,num_of_sliders=1, slider_number=1, maxV = 1, 
		minV = -1, intV = 0, gain = 1, name = 'Slider'):
		# num_of_sliders - The number of Sliders that you will use
		# slider_number - This number starts at 1 and its max value 
		#                 should be num_of_sliders . This variable 
		#                 determines the location of the Slider in 
		#                 the axes; 1 corresponds to the top Slider 
		#                 in the axes and largest number corresponds
		#                 to the bottom of the axes. Each Slider 
		#                 object needs a unique slider_number.

		self.data = 0            # Current value of Slider
		self.name = name         # Name of Slider
		self.maxValue = maxV     # Max value of the Slider
		self.minValue = minV     # Min value of the Slider
		self.initValue = intV    # Initial value of Slider
		self.gain = gain         # The gain of the Slider can be used for 
								 # conversion purposes. EX, If you want the 
								 # Slider to be in terms of degrees but the
								 # value passed in radians. The max and min 
								 # values will in terms of degrees and the 
								 # gain would be pi/180 to convert
								 # the Slider's value from degrees to radians.
		self.Slider_length = 0.5 # This is the length of the Slider in terms 
								 # of the percentage of the figure's length
		self.Slider_width = 0.03 # This is the width of the Slider in terms 
								 # of the percentage of the figure's width

		
		# Sets the position and color of the Slider

		# The horizontal position of the Slider in the figure.
		hpos = 0.8 - 0.6/(num_of_sliders+1)*slider_number 

		# Specify a subsection of the current axes where the slider will go.
		self.axSlider = plt.axes([0.5-self.Slider_length/2,hpos, self.Slider_length, 
			self.Slider_width], axisbg = 'orange')

		# Instantiate a slider, and create a handle to it
		self.SliderHandle = Slider(self.axSlider, self.name, self.minValue,
			self.maxValue, valinit = self.initValue)
		self.data = self.initValue

		# When a change occurs on the slider, the function self.update
		# will be called.
		self.SliderHandle.on_changed(self.update)

		

	# Updates the value of the Slider when the Slider is changed
	def update(self,val):
		self.data = self.SliderHandle.val

	# Returns the current value of the Slider(s)
	def getValue(self):
		return self.data*self.gain

def angle_publish(event):
	angle = usr_input.getInputValues()
	command_pub.publish(angle)

if __name__ == '__main__':

	usr_input = Sliders()

	# Create the node
	rospy.init_node('Gimbal_angle_command', anonymous=False)

	# Publisher to topic 'gimbal_cmd'
	command_pub = rospy.Publisher('gimbal_cmd',Float64,queue_size=1)

	# Init Timer
	pub_timer = rospy.Timer(rospy.Duration(1.0/1.0), angle_publish)

	try:

		# Keep node alive until ROS is shutdown.
		while not rospy.is_shutdown():
			plt.show(block=True)
				
		# The ROSInterruptException is raised if the program is killed
		# while sleeping with rospy.sleep() or rospy.rate.sleep()
	except rospy.ROSInterruptException:
		pass