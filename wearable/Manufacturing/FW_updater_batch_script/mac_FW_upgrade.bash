#! /bin/bash
# Copyright 2017 (c) Turingsense, Inc
#
# FW upgrader bash file for MAC
#

MAC_MAJOR_VERSION=0
MAC_MINOR_VERSION=1

echo "Turingsense Firmware Updater script for MAC version $MAC_MAJOR_VERSION.$MAC_MINOR_VERSION"

source mac_files.txt

blhost -u 0x15a2,0x0073 -- read-memory 0xFFF00 0x100  | grep "Error: UsbHidPeripheral()" 

if [ $? -eq 0 ] ; then
    echo ERROR: No device is connected!  Check if sat/hub is powered on and has white LED when you run this script!
	return 1
fi

blhost -u 0x15a2,0x0073 -- read-memory 0xFFF00 0x100 | grep "kStatusMemoryRangeInvalid" 

if [ $? -eq 0 ] ; then
    #SAT
    HW=SAT
    FLASH_START=0XA000
    FLASH_END=0x35800
    TUNNELING_BIN=$SAT_TUNNELING_BIN
    NORDIC_BIN=$SAT_NORDIC_BIN
    KINETIS_BIN=$SAT_KINETIS_BIN
else
    HW=HUB
    FLASH_START=0XF000
    FLASH_END=0xF0000
    TUNNELING_BIN=$HUB_TUNNELING_BIN
    NORDIC_BIN=$HUB_NORDIC_BIN
    KINETIS_BIN=$HUB_KINETIS_BIN
fi

echo
echo "Programming $HW"
echo

echo "STEP 1) Programming tunneling code... Please wait..."
# Erase and write tunneling code
blhost -u 0x15a2,0x0073 -- flash-erase-region $FLASH_START $FLASH_END
blhost -u 0x15a2,0x0073 -- write-memory $FLASH_START "$TUNNELING_BIN"
blhost -u 0x15a2,0x0073 -- reset
echo Programmed tunneling code successfully!

# Wait for it to come out of bootloader and start tunneling code
# LED should be blue.
echo
echo Waiting to program Nordic... Press any key ONLY AFTER your hardware turns BLUE!
sleep 10

# Update Nordic
echo
echo "STEP 2) Programming Nordic... Please wait..."
python "$PYTHON_SCRIPT" $NORDIC_BIN | grep -i "done sending"
if [ $? -eq 0 ] ; then
    echo Programming Nordic finished successfully!
	/bin/rm tmpFile
else
    echo ERROR: Fail!! Please check your USB connection and see if Device Manager recognizes the Virtual COM Port!
	/bin/rm tmpFile
    exit 1
fi

# Wait for it to go to 22F Bootloader
echo Waiting for a reset...Press any key ONLY AFTER your hardware turns WHITE!
sleep 5

echo
echo "STEP 3) Programming 22F... Please wait..."
blhost -u 0x15a2,0x0073 -- flash-erase-region $FLASH_START $FLASH_END
blhost -u 0x15a2,0x0073 -- write-memory $FLASH_START $KINETIS_BIN
blhost -u 0x15a2,0x0073 -- reset

echo Programming 22F successfully!

sleep 5

echo
echo
echo Bye bye!!
echo
echo

