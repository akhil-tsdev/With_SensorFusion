#!/usr/bin/env python
# Copyright (c) 2016 Turingsense Inc
# Script for RF chip FW upgrade
# TuringSense firmware downloader

from __future__ import print_function

import argparse
import sys

import serial, time

# Print version of Turingsense RF
def wait_data_startwith(stream, wait_str):
	while True:
		response = stream.readline()
                version = response
		#if len(response.strip()) > 0: 
	        #	print(response)
		if (response.startswith(wait_str)):
                        print (version)
			break

## defining waiting function
## it is used to read all the data from stream until it reach the data that
## matches with the given key word
## stream: the serial stream connected to the firmware
## wait_str: the wait string from firmware to indicate the end of waiting
def wait_data(stream, wait_str):
	while True:
		response = stream.readline()
		#if len(response) > 0: 
		#	print(response)
		if response.strip() == wait_str:
			break

## defining sending function
## it takes three arguments: stream, wait_str, value
## stream: the serial stream connected to the firmware
## wait_str: the wait string from firmware to indicate the readiness of the firmware
##	     to receive data
## value: the data to be sent to the firmware
def send_data(stream, wait_str, value):
	wait_data(stream, wait_str)
	if value is not None:
		if type(value) is str:
			size = len(value)
#			print("Sending: " + str(size) + "[" + value + "]")
			stream.write(str(size) + "\n")
			for x in value:
				stream.write(x)
		else:
			value_str = str(value)
#			print("Sending: [" + value_str + "]")
			stream.write(value_str + "\n")
		#time.sleep(0.5)
##
## end of send_data()
##

parser = argparse.ArgumentParser(description = 'TuringSense Nordic firmware downloader')
max_port_num = 100

g1 = parser.add_mutually_exclusive_group(required=True)
g1.add_argument('firmware_file', nargs='?', type=argparse.FileType('rb'))
g1.add_argument('--info', action='store_true')
parser.add_argument('--port', type=int, choices=xrange(0,max_port_num), default=-1)

#imagechoice = parser.add_argument_group('image')
#nimagechoice.add_argument('-1', '--image1', help = 'flash image 1')
#imagechoice.add_argument('-2', '--image2', help = 'flash image 2')

args = parser.parse_args()

if args.firmware_file is not None:
	firmware = args.firmware_file.read()
	args.firmware_file.close()
	firmware_length = len(firmware)
else:
	print("File " + args.firmware_file + "not found")
	sys.exit(-1)

port_num = args.port - 1
if port_num < 0:
	port_num = 0
	while port_num < max_port_num:
		try:
			test_stream = serial.Serial(port_num)
			test_stream.close()
			break;
		except Exception as e:
			port_num = port_num + 1
if port_num >= max_port_num:
	print("Can't find TuringSense device")
	sys.exit(0)
try:
	stream = serial.Serial(port_num)  # open first serial port
except Exception as e:
	print("Fail to open port #" + str(port_num + 1))
	sys.exit(-1)

# Print version
wait_data_startwith(stream, "Turingsense")
print("File size: " + str(firmware_length))
lines = firmware.split();
num_lines = len(lines)
print("Num of line: " + str(num_lines))

print("Reading from " + stream.portstr)       # check which port was really used

# wait until we got the start signal from the firmware
wait_data(stream, "STARTTXHEX")

# send the number of lines
send_data(stream, "READY", num_lines)

time.sleep(0.5)

last = ""
count = 0
for x in lines:
	last = x
	send_data(stream, "READY", x)
	count = count + 1
	if count % 10 == 0:
		print(str(count) + " of " + str(num_lines))

	#stream.close()             # close port
	#sys.exit(0)

print("Done sending. Waiting for END signal")
# wait until we got the end signal from the firmware
wait_data(stream, "ENDTXHEX")

stream.close()             # close port
sys.exit(0)

