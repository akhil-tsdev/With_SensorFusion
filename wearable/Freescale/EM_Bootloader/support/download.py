#!/usr/bin/env python

# TuringSense firmware downloader

from __future__ import print_function

import argparse
import sys

import kboot

# special bootloader properties for TuringSense device
# MaxImageLength is the maximum length in bytes for a normal firmware image
TuringSenseMaxImageLength = 0x40

# MaxLongImageLength is the maximum length in bytes for an oversize image 1
# which will occupy both image 1 and image 2 flash space
TuringSenseMaxLongImageLength = 0x41

# The BaseAddr attributes are the flash memory base addresses of the image
# areas
TuringSenseImage1BaseAddr = 0x42
TuringSenseImage2BaseAddr = 0x43

def auto_int (str):
    return int (str, 0)

parser = argparse.ArgumentParser(description = 'TuringSense firmware downloader')

parser.add_argument('firmware_file', nargs='?', type=argparse.FileType('rb'))
parser.add_argument('--info', action='store_true')
parser.add_argument('--image', type=int, choices=xrange(1,3), default=2)
parser.add_argument('-r', '--reset', action='store_true')

FREESCALE_VID = 0x15a2
FREESCALE_PID = 0x0073
parser.add_argument('--vid', type=auto_int, help='USB vendor ID', default=FREESCALE_VID)
parser.add_argument('--pid', type=auto_int, help='USB product ID', default=FREESCALE_PID)

args = parser.parse_args()
#print(args)

if args.firmware_file is not None:
    firmware = args.firmware_file.read()
    args.firmware_file.close()
    firmware_length = len(firmware)

kb = kboot.KBoot()

try:
    devs = kb.scan_usb_devs(kboot_vid = args.vid, kboot_pid = args.pid)
    if len(devs) == 0:
        print("TuringSense device not found")
        sys.exit(2)
    elif len(devs) > 1:
        print("more than one TuringSense device found")
        sys.exit(3)
    kb.connect_usb(devs[0])
    print("TuringSense device connected")

#    data = kb.read_memory (start_address = 0, length = 256)
#    print(data)

    flash_sector_size = kb.get_property(kboot.Property.FlashSectorSize)['raw_value']

    try:
        max_image_length = kb.get_property(TuringSenseMaxImageLength)['raw_value']
        max_long_image_length = kb.get_property(TuringSenseMaxLongImageLength)['raw_value']
        image_1_addr = kb.get_property(TuringSenseImage1BaseAddr)['raw_value']
        image_2_addr = kb.get_property(TuringSenseImage2BaseAddr)['raw_value']
    except Exception as e:
        print("Can't read TuringSense device properties")
        if False: # for testing if properties are not available
            image_1_addr = 0x8000
            image_2_addr = 0x24000
            max_image_length = 0x1c000
            max_long_image_length = 0x38000
        else:
            sys.exit(4)

    if args.info:
        print("Flash sector size: %d" % flash_sector_size)
        print("Max image length %d" % max_image_length)
        print("Max long image length %d" % max_long_image_length)
        print("image 1 addr %08x" % image_1_addr)
        print("image 2 addr %08x" % image_2_addr)
        sys.exit(0)  # everything is OK

    if args.firmware_file:

        if args.image == 1:
            base = image_1_addr
            max_len = max_long_image_length
        else:
            base = image_2_addr
            max_len = max_image_length

        if firmware_length > max_len:
            print("firmware image too large")
            sys.exit(5)

        if firmware_length % flash_sector_size != 0:
            #print("padding firmware, original length %d" % firmware_length)
            pad_len = flash_sector_size - (firmware_length % flash_sector_size)
            firmware = firmware + '\0' * pad_len
            firmware_length = len(firmware)
            #print("padded length %d" % firmware_length)
        
        try:
            kb.flash_erase_region(base, firmware_length)
        except Exception as e:
            print("error erasing flash")
            print(str(e))
            sys.exit(6)

        try:
            kb.write_memory(base, firmware)
        except Exception as e:
            print("error erasing flash")
            print(str(e))
            sys.exit(7)

        try:
            verify_data = kb.read_memory(base, firmware_length)
        except Exception as e:
            print("error reading back flash")
            print(str(e))
            sys.exit(8)

        if firmware != verify_data:
            print("firmware read back verification failed")
            print(str(e))
            sys.exit(9)

        print("firmware downloaded")

    if args.reset:
        magic_base = 0x1fffc000
        kb.write_memory(magic_base, [0])
        print("resetting target")
        kb.reset()

    kb.disconnect()
    sys.exit(0)
    

except Exception as e:
    print(str(e))
    sys.exit(1)

