This IAR projects were built using IAR on Eclipse.
http://mcuoneclipse.com/2013/11/03/tutorial-replacing-iar-ew-with-eclipse-ide/

IAR might have been installed here:
C:\Program Files (x86)\IAR Systems\Embedded Workbench 7.2


How to Build for IAR on Eclipse:
1.Add  C/C++ Build ->Environment -> add environment variable
GITHUB
C:\Users\cwati\Documents\GitHub

MQX_4_1_PATH
C:\Freescale\Freescale_MQX_4_1
How to Build for IAR on Eclipse:

2. Copy over the following
$GITHUB\trunk\wearable\Freescale_MQX_4_1\mqx\examples\<IAR project name folder>

To your:  
C:\Freescale\Freescale_MQX_4_1\mqx\examples\

3. Update your SPI driver with the following (if you haven't).
$GitHub\trunk\wearable\Freescale_MQX_4_1\mqx\source\io\spi_ksdk

copy the whole directory "spi_ksdk" to:
C:\Freescale\Freescale_MQX_4_1\mqx\source\io\

Also update your OSSF driver with the following (if you haven't).
$GitHub\trunk\wearable\Freescale_MQX_4_1\mqx\source\io\ossf

copy the whole directory "ossf" to:
C:\Freescale\Freescale_MQX_4_1\mqx\source\io\

4. Import the GITHUB project as existing project.


5. Build the libraries
1) For enabling SPI0 
copy the following files:
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

You might need to include libraries in your project
C/C++ Build-> Settings-> IAR Linker for ARM-> Library
"C:/Freescale/Freescale_MQX_4_1/lib/frdmk22f120m.iar/debug/bsp/bsp.a"
"C:/Freescale/Freescale_MQX_4_1/lib/frdmk22f120m.iar/debug/psp/psp.a"

