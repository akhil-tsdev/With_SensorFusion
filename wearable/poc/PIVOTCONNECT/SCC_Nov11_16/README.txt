HUB NAME IP ADDRESS     SAT IDS BT(optional)   CHANNEL HUB ID
P1	192.168.1.101	11-12-13	       	2	10	
P2	192.168.1.102	21-22-23		5	20	
P3	192.168.1.103	31-32-33		8	30	
P4	192.168.1.104	41-42-43		11	40	
P5	192.168.1.105	51-52-53		13	50	


0) Must use the new Nordic bootloader.
hub 22f, hub nordic, sat 22f, sat nordic binaries are to be used with
bootloader (starting address is NOT 0x0).

1) All satellite 22F uses the same binary.  Then overwrite the ID using the
flash.

// Read TS region
blhost -u 0x15a2,0x0073 -- read-memory 0x3f800 100
// Clear TS region
blhost -u 0x15a2,0x0073 -- flash-erase-region 0x3f800 0x800

**** Write sat ID, example 0x13b (315 decimal)  
blhost -u 0x15a2,0x0073 -- write-memory 0x3f800 {{01000000}}

blhost -u 0x15a2,0x0073 -- reset

2) Tested with this UnityApp-Connect commit:
https://github.com/TuringSense/UnityApp-Connect/commit/b001b8a4a7352b9d0884958881938327fd4e9635
