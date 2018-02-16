:: Copyright 2017 (c) Turingsense, Inc
::
:: FW upgrader batch file
::
:: Modify files.txt
:: 1) tunneling sw
:: 2) nordic file
:: 3) 22f file
:: 

@ECHO OFF
SET MAJOR_VERSION=2
SET MINOR_VERSION=9

SET blhost=C:\turing\Github\trunk\wearable\Freescale\NXP_Kinetis_Bootloader_2_0_0\bin\Tools\blhost\win\blhost.exe
ECHO Turingsense Firmware Updater Batch file version %MAJOR_VERSION%.%MINOR_VERSION%
ECHO.

:: Set files
for /f "delims== tokens=1,2" %%G in (files.txt) do set %%G=%%H

%blhost% -u 0x15a2,0x0073 -- read-memory 0xFFF00 0x100 > tmpFile

:: See if no device is connected
find "Error: UsbHidPeripheral()" tmpFile > nul
IF %errorlevel% equ 0 (
    ::No device connected
    echo ERROR: No device is connected!  Check if sat/hub is powered on and has white LED when you run this script!
    goto done
)

find "kStatusMemoryRangeInvalid" tmpFile > nul
IF %errorlevel% equ 0 (
    ::SAT
    set HW=SAT
    set FLASH_START=0XA000
    set FLASH_END=0x35800
    set TUNNELING_BIN=%SAT_TUNNELING_BIN%
    set NORDIC_BIN=%SAT_NORDIC_BIN%
    set KINETIS_BIN=%SAT_KINETIS_BIN%
) ELSE (
    set HW=HUB
    set FLASH_START=0XF000
    set FLASH_END=0xF0000
    set TUNNELING_BIN=%HUB_TUNNELING_BIN%
    set NORDIC_BIN=%HUB_NORDIC_BIN%
    set KINETIS_BIN=%HUB_KINETIS_BIN%
)
del tmpFile

ECHO.
ECHO Programming %HW%
ECHO.
::ECHO 22f tunneling bin:
::ECHO %TUNNELING_BIN%
::ECHO.
::ECHO Nordic bin:
::ECHO %NORDIC_BIN%
::ECHO.
::ECHO Kinetis bin:
::ECHO %KINETIS_BIN%
::ECHO.

ECHO OFF
ECHO STEP 1) Programming tunneling code... Please wait...
:: Erase and write tunneling code
%blhost% -u 0x15a2,0x0073 -- flash-erase-region %FLASH_START% %FLASH_END% > nul
%blhost% -u 0x15a2,0x0073 -- write-memory %FLASH_START% %TUNNELING_BIN%  > nul
%blhost% -u 0x15a2,0x0073 -- reset > nul
ECHO Programmed tunneling code successfully!

:: Wait for it to come out of bootloader and start tunneling code
:: LED should be blue.
ECHO.
ECHO Waiting to program Nordic... Press any key ONLY AFTER your hardware turns BLUE!
TIMEOUT 10

:: Update Nordic
ECHO.
ECHO STEP 2) Programming Nordic... Please wait...
python %PYTHON_SCRIPT% %NORDIC_BIN% | find /i "done sending" > nul
if %errorlevel% equ 0 (
    ECHO Programming Nordic finished successfully!
) ELSE (
    ECHO ERROR: Fail!! Please check your USB connection and see if Device Manager recognizes the Virtual COM Port!
    goto done
)

:: Wait for it to go to 22F Bootloader
@ECHO OFF
ECHO Waiting for a reset...Press any key ONLY AFTER your hardware turns WHITE!
TIMEOUT 5

ECHO.
ECHO STEP 3) Programming 22F... Please wait...
%blhost% -u 0x15a2,0x0073 -- flash-erase-region %FLASH_START% %FLASH_END% > nul
%blhost% -u 0x15a2,0x0073 -- write-memory %FLASH_START% %KINETIS_BIN% | find /i "Error" > nul
if %errorlevel% equ 0 (
    ECHO ERROR with file '%KINETIS_BIN%'
) 
%blhost% -u 0x15a2,0x0073 -- reset > nul

ECHO Programming 22F successfully!

TIMEOUT 5

:done
ECHO.
ECHO.
ECHO Bye bye!!
ECHO.
ECHO.

