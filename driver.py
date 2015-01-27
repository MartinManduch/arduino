#http://petrimaki.com/2013/04/28/reading-arduino-serial-ports-in-windows-7/
#python 2.7
import serial
import time
import sys
import struct

def packIntegerAsULong(value):
    """Packs a python 4 byte unsigned integer to an arduino unsigned long"""
    return struct.pack('I', value)    #should check bounds
	
	
ser = serial.Serial('COM18', 9600, timeout=3)

# command number
ser.write([int(sys.argv[1])])

if int(sys.argv[1]) > 9: # commands from 10 and higher require 2 params
	# parameter 1 (motor speed) 0-255
	ser.write([int(sys.argv[2])])

	# parameter 2 (number of impulses)
	ser.write(packIntegerAsULong(int(sys.argv[3])))


while 1:
	try:
		input = ser.readline()
		if (input != None and input != ""):
			input = input.rstrip('\n')
			if input[0] =='i':
				sys.stdout.write("impulses:" +  input[1:] + "\r")			
				#print(input[1:])
			elif input[0]=="m":				
				if input[1:].find("STOP")>-1:					
					print("\n" + input[1:])
					break
			else:
				print input
		else: # readline timeout occured
			break
	except serial.SerialTimeoutException:
		print('Data could not be read')
	time.sleep(0.0005)
