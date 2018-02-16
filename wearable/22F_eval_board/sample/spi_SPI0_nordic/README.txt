05/06/2015 cwati

Read ../README_IAR.txt for more info on building the project.

This code is for K22F512 12 eval board.
This will talk to Nordic via SPI0 CS1.
Nordic will be the slave.

It seems like when acting as a master, Freescale 22F
fails to get the first byte sent by the slave.
The Beagle says the data is received, but the first pop on
22F doesn't store it.

I didn't spend time debugging the 22F, instead I added a buffer
when Nordic slave sends data.  A buffer of 4 bytes.

When 22F receives it, it discards 3 bytes (the first 1 is already
lost).

This project was built using IAR on Eclipse.
http://mcuoneclipse.com/2013/11/03/tutorial-replacing-iar-ew-with-eclipse-ide/


