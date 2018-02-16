To use:
1) Modify files.txt
2) Open "cmd" in Windows (Command Prompt)
3) Connect hardware to PC then power on.  It should be recognized as a COM
port on your PC.  While its LED is white (about 5 seconds), run:
    
    C:\turing\Github\trunk\wearable\Manufacturing\FW_updater_batch_script>FW_upgrade.bat



--------------Sample output------------------------------
C:\Users\cwati\Desktop\SAVE\FW_upgrade_script>FW_upgrade.bat

Programming HUB

Checking binaries...
Everything looks ok...Starting to program HUB...

STEP 1) Programming tunneling code... Please wait...
Programmed tunneling code successfully!

Waiting to program Nordic... Press any key ONLY AFTER your hardware turns BLUE!

Waiting for  3 seconds, press a key to continue ...

STEP 2) Programming Nordic... Please wait...
Programming Nordic finished successfully!
Waiting for a reset...Press any key ONLY AFTER your hardware turns WHITE!

Waiting for 3 seconds, press a key to continue ...

STEP 3) Programming 22F... Please wait...
Programming 22F successfully!

Waiting for 0 seconds, press a key to continue ...


Bye bye!!


C:\Users\cwati\Desktop\SAVE\FW_upgrade_script>
