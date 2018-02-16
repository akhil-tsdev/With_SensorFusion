REM ---------------------------------------------------------------
rem -- START PARAMETERS ------------------------------------------|
REM ---------------------------------------------------------------

set FW_VERSION=0_18_20.12.13.12_eGarment


set COMPILE_SAT_NORDIC=1
set COMPILE_HUB_NORDIC=1
set COMPILE_SAT_22F=1
set COMPILE_HUB_22F=1

set PRIMARY_PATH=C:\turing\github\trunk\wearable
set PRIMARY_BIB_PATH=%PRIMARY_PATH%\Manufacturing\Binaries

set LOG_FILE=copy_bin_to_manufactory_path.log

REM ---------------------------------------------------------------
rem -- END PARAMETERS   ------------------------------------------|
REM ---------------------------------------------------------------

echo START COPY OF BINARIES > %LOG_FILE%
rem del /S /Q %PRIMARY_BIB_PATH%\ONBOARD-DIAG\ >> %LOG_FILE%
rem del /S /Q %PRIMARY_BIB_PATH%\SYS-DIAG\ >> %LOG_FILE%
 
rem pause

rem SYSTEM
if %COMPILE_HUB_NORDIC% == 0 goto continueHubNrd;
 del /S /Q %PRIMARY_BIB_PATH%\SYS-DIAG\hub\nordic\ >> %LOG_FILE%
 set HUB_NORDIC_BUILD_FILENAME=%PRIMARY_PATH%\nordic_module\hub\_build\SYS_NRD_HUB
 set HUB_NORDIC_DESTINATION_FILENAME=%PRIMARY_BIB_PATH%\SYS-DIAG\FW_%FW_VERSION%
 echo f|xcopy /Q /R /S /Y %HUB_NORDIC_BUILD_FILENAME%.hex %HUB_NORDIC_DESTINATION_FILENAME%\SYS_NRD_HUB_%FW_VERSION%.hex >> %LOG_FILE%
 echo f|xcopy /Q /R /S /Y %HUB_NORDIC_BUILD_FILENAME%.axf %HUB_NORDIC_DESTINATION_FILENAME%\OTA\SYS_NRD_HUB_%FW_VERSION%.axf >> %LOG_FILE%
:continueHubNrd

if %COMPILE_SAT_NORDIC% == 0 goto continueSatNrd;
 del /S /Q %PRIMARY_BIB_PATH%\SYS-DIAG\sat\nordic\ >> %LOG_FILE%
 set SAT_NORDIC_BUILD_FILENAME=%PRIMARY_PATH%\nordic_module\satellite\_build\SYS_NRD_SAT
 set SAT_NORDIC_DESTINATION_FILENAME=%PRIMARY_BIB_PATH%\SYS-DIAG\FW_%FW_VERSION%
 echo f|xcopy /Q /R /S /Y %SAT_NORDIC_BUILD_FILENAME%.hex %SAT_NORDIC_DESTINATION_FILENAME%\SYS_NRD_SAT_%FW_VERSION%.hex >> %LOG_FILE%
 echo f|xcopy /Q /R /S /Y %SAT_NORDIC_BUILD_FILENAME%.axf %SAT_NORDIC_DESTINATION_FILENAME%\OTA\SYS_NRD_SAT_%FW_VERSION%.axf >> %LOG_FILE%
:continueSatNrd

if %COMPILE_HUB_22F% == 0 goto continueHub22f;
 del /S /Q %PRIMARY_BIB_PATH%\SYS-DIAG\hub\22F\ >> %LOG_FILE%
 set HUB_22F_BUILD_FILENAME="%PRIMARY_PATH%\Freescale\Freescale_MQX_4_1\demo\hub_prod_2_tasks\iar\twr21f120m\Int Flash Debug\Exe\SYS_22F_HUB.bin"
 set HUB_22F_DESTINATION_FILENAME=%PRIMARY_BIB_PATH%\SYS-DIAG\FW_%FW_VERSION%\SYS_22F_HUB_%FW_VERSION%.bin
 echo f|xcopy /Q /R /S /Y %HUB_22F_BUILD_FILENAME% %HUB_22F_DESTINATION_FILENAME% >> %LOG_FILE%
 set HUB_22F_BUILD_FILENAME="%PRIMARY_PATH%\Freescale\Freescale_MQX_4_1\demo\hub_prod_2_tasks\iar\twr21f120m\Int Flash Debug\Exe\SYS_22F_HUB.out"
 set HUB_22F_DESTINATION_FILENAME=%PRIMARY_BIB_PATH%\SYS-DIAG\FW_%FW_VERSION%\SYS_22F_HUB_%FW_VERSION%.out
 echo f|xcopy /Q /R /S /Y %HUB_22F_BUILD_FILENAME% %HUB_22F_DESTINATION_FILENAME% >> %LOG_FILE%
:continueHub22f

if %COMPILE_SAT_22F% == 0 goto continueSat22f;
 del /S /Q %PRIMARY_BIB_PATH%\SYS-DIAG\sat\22F\ >> %LOG_FILE%
 set SAT_22F_BUILD_FILENAME=%PRIMARY_PATH%\22F_sensor_module\production\22f_sat_production2\Bootloader\SYS_22F_SAT.bin
 set SAT_22F_DESTINATION_FILENAME=%PRIMARY_BIB_PATH%\SYS-DIAG\FW_%FW_VERSION%\SYS_22F_SAT_%FW_VERSION%.bin
 echo f|xcopy /Q /R /S /Y %SAT_22F_BUILD_FILENAME% %SAT_22F_DESTINATION_FILENAME% >> %LOG_FILE%
 set SAT_22F_BUILD_FILENAME=%PRIMARY_PATH%\22F_sensor_module\production\22f_sat_production2\Bootloader\SYS_22F_SAT.elf
 set SAT_22F_DESTINATION_FILENAME=%PRIMARY_BIB_PATH%\SYS-DIAG\FW_%FW_VERSION%\SYS_22F_SAT_%FW_VERSION%.elf
 echo f|xcopy /Q /R /S /Y %SAT_22F_BUILD_FILENAME% %SAT_22F_DESTINATION_FILENAME% >> %LOG_FILE%
:continueSat22f
