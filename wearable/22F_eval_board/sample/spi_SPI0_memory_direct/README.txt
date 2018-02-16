05/01/2015

Read ../README_IAR.txt for more info on building the project.

This project aims to talk to Invensense from K22F_51212.
MQX SPI driver is buggy for MK22F 51212.
Successfully talked to Invensense on SPI0 CS1 by porting KSDK driver.

Limitation to the porting of the SPI driver from KSDK 1.0.0 to MQX 4.1
1. eDMA is disabled
2. slave is disabled

Warning: Setting breakpoints at certain location for tracing during debugging might cause SPI transcation to fail.

Before you compile, 
1) copy the following files:
C:\Users\cwati\Documents\GitHub\trunk\wearable\Freescale_MQX_4_1\config\frdmk22f120m\user_onfig.SPI0enabled.h
"C:\Freescale\Freescale_MQX_4_1\mqx\source\bsp\frdmk22f120m\frdmk22f120m.SPI0enabled.h" 
"C:\Freescale\Freescale_MQX_4_1\mqx\source\bsp\frdmk22f120m\init_spi.SPI0enabled.c" 

to the following path (notice the renaming of the file names):
"C:\Freescale\Freescale_MQX_4_1\config\frdmk22f120m\user_config.h"
"C:\Freescale\Freescale_MQX_4_1\mqx\source\bsp\frdmk22f120m\frdmk22f120m.h" 
"C:\Freescale\Freescale_MQX_4_1\mqx\source\bsp\frdmk22f120m\init_spi.c" 

Then rebuild BSP and PSP libraries
C:\Freescale\Freescale_MQX_4_1\mqx\build\iar\bsp_frdmk22f120m
C:\Freescale\Freescale_MQX_4_1\mqx\build\iar\psp_frdmk22f120m

2) Port the SPI library 
copy the whole directory "spi_ksdk" to:
C:\Freescale\Freescale_MQX_4_1\mqx\source\io\

3) You might need to include libraries
C/C++ Build-> Settings-> IAR Linker for ARM-> Library
"C:/Freescale/Freescale_MQX_4_1/lib/frdmk22f120m.iar/debug/bsp/bsp.a"
"C:/Freescale/Freescale_MQX_4_1/lib/frdmk22f120m.iar/debug/psp/psp.a"


