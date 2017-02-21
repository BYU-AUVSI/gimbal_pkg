#!/usr/bin/env python

from time import sleep
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200) # Establish the connection on a specific port
counter = 32 # Below 32 everything in ASCII is gibberish
ser.flush()

while True:
	counter +=1
	#ser.write(str(chr(counter))) # Convert the decimal number to ASCII then send it to the Arduino
	if (ser.inWaiting()>0):	
		print ser.readline() # Read the newest output from the Arduino
		#sleep(.1) # Delay for one tenth of a second
	if counter == 255:
		counter = 32
