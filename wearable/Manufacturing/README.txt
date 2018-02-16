Sample steps to program Manufacturing board for ONBOARD DIAG code.

This is the blhost path.
C:\turing\github\trunk\wearable\Freescale\NXP_Kinetis_Bootloader_2_0_0\bin\Tools\blhost\
It's a lot easier if you put the path to blhost to your System Variables.


1) Make sure you program the Nordic Bootloader with this:
C:\turing\github\trunk\wearable\Manufacturing\Binaries\common nordic bootloader\nordic_bootloader_2_2.hex

WARNING: When programming Nordic Bootloader using NrfGo, make sure you select
"Program Bootloader"



==========================SAT NORDIC==========================
blhost -u 0x15a2,0x0073 -- flash-erase-region 0xa000 0x35800
blhost -u 0x15a2,0x0073 -- write-memory 0xa000 "C:\turing\Github\trunk\wearable\Manufacturing\Binaries\22fNordic tunneling\sat\22f_sat_nordic_bl_bootloader_2_2.bin"


blhost -u 0x15a2,0x0073 -- reset
python C:\turing\Github\trunk\wearable\common_code\bootloader\nordic_download.py --port 3 C:\turing\github\trunk\wearable\Manufacturing\Binaries\SYS-DIAG\sat\nordic\SYS_NRD_SAT_0_14_5.3.4.2_forboot.hex"
=============================SAT==============================

power cycle satellite

**** Change 22F binary
blhost -u 0x15a2,0x0073 -- flash-erase-region 0xa000 0x35800

blhost -u 0x15a2,0x0073 -- write-memory 0xa000 "C:\turing\github\trunk\wearable\Manufacturing\Binaries\SYS-DIAG\sat\22F\SYS_22F_SAT_0_14.5.3.4.2_forboot.bin"

**** Check content of Flash
blhost -u 0x15a2,0x0073 -- read-memory 0x3f800 100
**** Clear TS region
blhost -u 0x15a2,0x0073 -- flash-erase-region 0x3f800 0x800

**** Write sat ID, example 0x13b (315 decimal)  
blhost -u 0x15a2,0x0073 -- write-memory 0x3f800 {{95010000}}

**** Write hub ID
blhost -u 0x15a2,0x0073 -- write-memory 0x3f834 {{08000000}}

**** Write LCP
blhost -u 0x15a2,0x0073 -- write-memory 0x3f844 {{08000000}}

blhost -u 0x15a2,0x0073 -- reset
==========================HUB NORDIC==========================
cd "C:\turing\Github\trunk\wearable\Manufacturing\Binaries\22fNordic tunneling\hub"

blhost -u 0x15a2,0x0073 -- flash-erase-region 0xF000 0xF0000
blhost -u 0x15a2,0x0073 -- write-memory 0xF000 "C:\turing\Github\trunk\wearable\Manufacturing\Binaries\22fNordic tunneling\hub\22f_hub_nordic_bl_bootloader_2_2.bin"
blhost -u 0x15a2,0x0073 -- reset

python C:\turing\Github\trunk\wearable\common_code\bootloader\nordic_download.py --port 3 "C:\turing\github\trunk\wearable\Manufacturing\Binaries\SYS-DIAG\hub\nordic\SYS_NRD_HUB_0_14_5.3.4.2_forboot.hex"


=============================HUB 22F==============================
Power cycle hub.

blhost -u 0x15a2,0x0073 -- flash-erase-region 0xF000 0xF0000
blhost -u 0x15a2,0x0073 -- write-memory 0xF000 "C:\turing\github\trunk\wearable\Manufacturing\Binaries\SYS-DIAG\hub\22F\SYS_22F_HUB_0_14_5.3.4.2_ch10_forboot.bin"
blhost -u 0x15a2,0x0073 -- reset


// Clear TS region.  Sector size for hub is 0x1000.  TS_region for flash starts at 0xff000 (That is: 0x10_0000 - 0x1000).
blhost -u 0x15a2,0x0073 -- read-memory 0xFFF00 0x100
blhost -u 0x15a2,0x0073 -- flash-erase-region 0xFF000 0x1000

******** Write hub ID
blhost -u 0x15a2,0x0073 -- write-memory 0xFFFF0 {{ffffffffffffffffffffffff08000000}}
blhost -u 0x15a2,0x0073 -- read-memory 0xFFFFC 4
blhost -u 0x15a2,0x0073 -- reset



