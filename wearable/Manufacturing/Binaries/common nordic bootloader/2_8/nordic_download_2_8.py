#!/usr/bin/env python
# Copyright (c) 2016 Turingsense Inc
# Script for RF chip FW upgrade
# TuringSense firmware downloader

from __future__ import print_function
from timeit import default_timer as timer

import argparse
import sys
import re

import serial, time
import serial.tools.list_ports

import Tkinter
import tkFileDialog
import tkMessageBox

# Print version of Turingsense RF
def wait_data_startwith(stream, wait_str):
	while True:
		response = stream.readline()
                version = response
		#if len(response.strip()) > 0: 
	        #	print(response)
		if (response.startswith(wait_str)):
                        print (version)
			m1 = re.search('\d\.\d', version)
			ver = re.split('\D+',m1.group(0))
			return (int(ver[0]) * 1000) + int(ver[1])
			#break

## defining waiting function
## it is used to read all the data from stream until it reach the data that
## matches with the given key word
## stream: the serial stream connected to the firmware
## wait_str: the wait string from firmware to indicate the end of waiting
def wait_data(stream, wait_str):
	while True:
		response = stream.readline()
		#if len(response.strip()) > 0: 
		#	print(response)
		if response.strip() == wait_str:
			break

## defining function for converting hex in string to binary
## hex to bin
## input: string of intel hex format: ":XXXXXX"
## output: bytesarray of the same hex in bytes
def str_hex_to_bin(input_str):
	clean_str = input_str.strip()
	if ':' in clean_str:
		return bytearray.fromhex(clean_str[1:len(clean_str)])
	else:
		return bytearray.fromhex(clean_str)

def merge_hex(old_str, add_str):
	clean_str = add_str.strip()
	return old_str + clean_str

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
#			bytes_value = str_hex_to_bin(value)
#			size = len(bytes_value)
#			print("Sending: " + str(size) + "[" + value + "]")
#			stream.write(str(size) + "\n")
#			for x in bytes_value:
#				stream.write(x)

			size = len(value)
			#print("Sending: " + str(size) + "[" + value + "]")
			stream.write(str(size) + "\n")
			for x in value:
				stream.write(x)
		else:
			value_str = str(value)
			#print("Sending: [" + value_str + "]")
			stream.write(value_str + "\n")
		#time.sleep(0.5)
##
## end of send_data()
##



## defining function to upload Nordic binary
## this is the main function that will send the binary data to Nordic
## it takes two arguments: port number and the list of data read from file
## port_num: the port number for transmission
## firmware: the firmware data read from file
def upload_binary(port_num, firmware):
	global progress
	global progress_pct
	global is_gui_mode
	try:
		stream = serial.Serial(port_num)  # open first serial port
	except Exception as e:
		print("Fail to open port #" + str(port_num + 1))
		return

	NUM_HEX_PER_USB_UPLOAD = 8

	start = timer()
	firmware_length = len(firmware)
	# Print version
	version = wait_data_startwith(stream, "Turingsense")
	if version <= 2004:
		# backward compatibility for bootloader V 2.4 or before
		NUM_HEX_PER_USB_UPLOAD = 1
	print("File size: " + str(firmware_length))
	lines = firmware.split();
	num_lines = len(lines)
	print("Num of line: " + str(num_lines))

	print("Reading from " + stream.portstr)       # check which port was really used

	# wait until we got the start signal from the firmware
	wait_data(stream, "STARTTXHEX")

	if is_gui_mode:
		progress.set(progress_pct, text = "Erasing ...")
	# send the number of lines
	send_data(stream, "READY", num_lines)

	#time.sleep(0.5)

	if is_gui_mode:
		progress.set(progress_pct, text = "Uploading ...")
	last = ""
	count = 0
	turn_count = 0
	merge_str = ""
	package_sent = 1
	for x in lines:
		last = x
		count = count + 1
#		print(str(count) + " of " + str(num_lines))
		if count % 10 == 0:
			print(str(count) + " of " + str(num_lines))

		if turn_count == 0:
			merge_str = x.strip()
		else:
			merge_str = merge_str + x.strip()
		package_sent = 0
		turn_count = turn_count + 1
		if turn_count < NUM_HEX_PER_USB_UPLOAD:
			continue
		# else turn_count == NUM_HEX_PER_USB_UPLOAD:
		package_sent = 1
		turn_count = 0
		send_data(stream, "READY", merge_str)
		if is_gui_mode:
			progress_pct = ((count * 100) / num_lines) * 0.01
			progress.set(progress_pct, text = "Uploading ...")

	if package_sent == 0 and len(merge_str) > 0:
		send_data(stream, "READY", merge_str)
		if is_gui_mode:
			progress_pct = ((count * 100) / num_lines) * 0.01
			progress.set(progress_pct, text = "Uploading ...")

	print("Done sending. Waiting for END signal")
	# wait until we got the end signal from the firmware
	wait_data(stream, "ENDTXHEX")

	stream.close()             # close port

	end = timer()
	print("Time: " + str(end - start))
##
## end of upload_binary()
##

## defining function to collect available ports and update the listBox
## return a list of ports
## this function also change the global variable port_list_box
def scan_ports():
	global port_list_box
	if (port_list_box.size() > 0):
		# delete existing entries
		port_list_box.delete(first = 0, last = port_list_box.size() - 1)
	ports = list(serial.tools.list_ports.comports())
	if (len(ports) > 0):
		id = 0
		for port in ports:
			print (port)
			id = id + 1
			if (len(port[1]) > 0):
				port_list_box.insert(id, port[1])
			else:
				port_list_box.insert(id, port[0])
	else:
		port_list_box.insert(1, "No device connected")
	return ports

############ the followings are sets of functions for GUI commands ##########

## functions for QUIT button
def quit_button():
	sys.exit(0)

## function for OPEN file button
## uses file_path global var to store the path to file
def open_file_button():
	global file_path
	file_path.set(tkFileDialog.askopenfilename())
	#print("File: " + file_path )

## function for RESET button
## uses ports global var to store the list of ports
def reset_port_button():
	global ports
	ports = scan_ports()

## function for updating the progress bar
def update_progress_bar():
	global progress_pct
	global progress
	progress.set(progress_pct, text = "Uploading ...")
	progress.after(10, lambda: update_progress_bar())

## function for UPLOAD button
## 
def upload_button():
	global progress
	global progress_pct
	global file_path
	global port_list_box
	global root
	global ports
	port_id = port_list_box.curselection()
	if len(ports) <= 0:
		tkMessageBox.showerror("Error", "There is no connected device found! Please connect TuringSense satelite or hub to USB port.")
		return
	if len(port_id) != 1:
		tkMessageBox.showerror("Error", "Port is not selected! Please select a port to use.")
		return
	if len(file_path.get()) <= 0:
		tkMessageBox.showerror("Error", "Nordic hex file is not selected! Please select a hex file.")
		return
	port_num = int(ports[port_id[0]][0][3:]) - 1

	progress_pct = 0.0
	progress.set(progress_pct, text = "Uploading ...")
	print(file_path.get())
	file_ptr = open(file_path.get(), "rb")
	firmware = file_ptr.read()
	file_ptr.close()

	upload_binary(port_num, firmware)
	progress.set(progress_pct, text = "Completed")

############ END of functions for GUI commands #############

############ Meter class definition for progress bar ###########
'''Michael Lange <klappnase (at) freakmail (dot) de>
The Meter class provides a simple progress bar widget for Tkinter.

INITIALIZATION OPTIONS:
The widget accepts all options of a Tkinter.Frame plus the following:

     fillcolor -- the color that is used to indicate the progress of the
                  corresponding process; default is "orchid1".
     value -- a float value between 0.0 and 1.0 (corresponding to 0% - 100%)
              that represents the current status of the process; values higher
              than 1.0 (lower than 0.0) are automagically set to 1.0 (0.0); default is 0.0 .
     text -- the text that is displayed inside the widget; if set to None the widget
             displays its value as percentage; if you don't want any text, use text="";
             default is None.
     font -- the font to use for the widget's text; the default is system specific.
     textcolor -- the color to use for the widget's text; default is "black".
 
WIDGET METHODS:
All methods of a Tkinter.Frame can be used; additionally there are two widget specific methods:

    get() -- returns a tuple of the form (value, text)
    set(value, text) -- updates the widget's value and the displayed text;
                        if value is omitted it defaults to 0.0 , text defaults to None .
'''
class Meter(Tkinter.Frame):
	def __init__(self, master, width=300, height=20, bg='white', fillcolor='orchid1',\
			value=0.0, text=None, font=None, textcolor='black', *args, **kw):
		Tkinter.Frame.__init__(self, master, bg=bg, width=width, height=height, *args, **kw)
		self._value = value
		self._canv = Tkinter.Canvas(self, bg=self['bg'], width=self['width'], \
				height=self['height'], highlightthickness=0, relief='flat', bd=0)
		self._canv.pack(fill='both', expand=1)
		self._rect = self._canv.create_rectangle(0, 0, 0, self._canv.winfo_reqheight(), \
				fill=fillcolor, width=0)
		self._text = self._canv.create_text(self._canv.winfo_reqwidth()/2, \
				self._canv.winfo_reqheight()/2, text='', fill=textcolor)
		if font:
			self._canv.itemconfigure(self._text, font=font)
		self.set(value, text)
		self.bind('<Configure>', self._update_coords)

	def _update_coords(self, event):
		'''Updates the position of the text and rectangle inside the canvas when the size of
		the widget gets changed.'''
		# looks like we have to call update_idletasks() twice to make sure
		# to get the results we expect
		self._canv.update_idletasks()
		self._canv.coords(self._text, self._canv.winfo_width()/2, self._canv.winfo_height()/2)
		self._canv.coords(self._rect, 0, 0, self._canv.winfo_width()*self._value, self._canv.winfo_height())
		self._canv.update_idletasks()

	def get(self):
		return self._value, self._canv.itemcget(self._text, 'text')

	def set(self, value=0.0, text=None):
		#make the value failsafe:
		if value < 0.0:
			value = 0.0
		elif value > 1.0:
			value = 1.0
		self._value = value
		if text == None:
			#if no text is specified use the default percentage string:
			text = str(int(round(100 * value))) + ' %'
		self._canv.coords(self._rect, 0, 0, self._canv.winfo_width()*value, self._canv.winfo_height())
		self._canv.itemconfigure(self._text, text=text)
		self._canv.update_idletasks()

############ END Meter class definition for progress bar ###########

parser = argparse.ArgumentParser(description = 'TuringSense Nordic firmware downloader')
max_port_num = 100

parser.add_argument('firmware_file', nargs='?', type=argparse.FileType('rb'))
parser.add_argument('--gui', help='Run in GUI mode', action='store_true')
parser.add_argument('--port', type=int, choices=xrange(0,max_port_num), default=-1)

args = parser.parse_args()

progress_pct = 0.0
if args.gui:
######## Start of GUI #########
	root = Tkinter.Tk()
	file_path = Tkinter.StringVar(root, value = "")
	#if args.firmware_file is not None:
	#	file_path.set(str(args.firmware_file))

	# USB selection frame
	port_frame = Tkinter.LabelFrame(root, text = "Select USB port")
	port_frame.pack(side = Tkinter.LEFT, expand = "no")
	port_list_frame = Tkinter.Frame(port_frame)
	port_list_frame.pack(fill = "both", expand = "yes")
	port_scrollbar = Tkinter.Scrollbar(port_list_frame)
	port_scrollbar.pack(side = Tkinter.RIGHT, fill = Tkinter.Y)
	port_list_box = Tkinter.Listbox(port_list_frame, selectmode = Tkinter.SINGLE,
			yscrollcommand = port_scrollbar.set, width = 30, height = 5)
	reset_button = Tkinter.Button(port_frame, text = "Reset device list",
			command = reset_port_button)
	ports = scan_ports()
	port_list_box.pack()
	port_scrollbar.config(command = port_list_box.yview)
	reset_button.pack()

	right_frame = Tkinter.Frame(root)
	right_frame.pack(side = Tkinter.RIGHT, fill = "both", expand = "yes")
	# file path frame
	file_frame = Tkinter.LabelFrame(right_frame, text = "Nordic binary")
	file_frame.pack(side = Tkinter.TOP, fill = "both", expand = "yes")
	file_entry = Tkinter.Entry(file_frame, bd = 5, width = 60,
				textvariable = file_path)
	file_entry.pack(side = Tkinter.LEFT)
	browse_button = Tkinter.Button(file_frame, text = "Browse",
				command = open_file_button)
	browse_button.pack(side = Tkinter.LEFT)

	# Button frame
	button_frame = Tkinter.Frame(right_frame)
	button_frame.pack(side = Tkinter.BOTTOM, expand = "yes")
	load_widget_button = Tkinter.Button(button_frame, text = "Upload", command = upload_button)
	load_widget_button.pack(side = Tkinter.LEFT)
	quit_widget_button = Tkinter.Button(button_frame, text = "Quit", command = quit_button)
	quit_widget_button.pack(side = Tkinter.RIGHT)

	# Progress bar
	progress_frame = Tkinter.LabelFrame(right_frame, text = "Progress")
	#progress_canvas = Tkinter.Canvas(progress_frame, bd = 0, height = 25, width = 400)
	#progress_canvas.create_rectangle(5, 5, 400, 25)
	progress_frame.pack(side = Tkinter.TOP, fill = "both", expand = "yes")
	#progress_canvas.pack(fill = "both", expand = "yes")
	progress = Meter(progress_frame, relief = "ridge", bd = 5)
	progress.pack(fill = "x")
	progress.set(0.0, "Progress ...")
	#progress.after(10, lambda: progress.set(progress_pct))

	is_gui_mode = True
	root.mainloop()
	sys.exit(0)
######## End of GUI #########

is_gui_mode = False

if args.firmware_file is not None:
	firmware = args.firmware_file.read()
	args.firmware_file.close()
	firmware_length = len(firmware)
else:
	print("File " + args.firmware_file + "not found")
	sys.exit(-1)

#debug
#lines = firmware.split();
#for x in lines:
#	bbb = str_hex_to_bin(x)
#	print("HEX: "+ str(len(bbb)) + ":" + str(bbb))
#sys.exit(0)
#end
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

upload_binary(port_num, firmware)
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

