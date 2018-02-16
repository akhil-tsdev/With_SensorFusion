Turingsense 2015
cwati 03/01/15
 
This project is for the EVAL board of the sensor module (22F) to talk to
Invensense.  

At the end of the main function, there is a while loop that reads all 128
registers within the Invensense.  You can
use Kinetis Design Studio debugger view.  View "(x)= Variables" and
monitor the content of stm_buffer[].  
See how the sensor registers, somewhere around
register 60 (0x3C) --> check MPU9250_SPI.h, vary with the movement of the
sensor module.


To build this project you will need ksdk_platform_lib_K22F51212 that's included
in Freescale standard Kinetis SDK.
C:\Freescale\KSDK_1.0.0\lib\ksdk_platform_lib\kds\K22F51212
