/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_device_registers.h"
#include "fsl_os_abstraction.h"
#include "fsl_dspi_master_driver.h"
#include "board.h"

#define RED 	0x52
#define GREEN 	0x47
#define BLUE 	0x42

bool pushflag = false;
uint8_t spiSourceBuffer = RED;
uint8_t spiSinkBuffer;

uint8_t RGBdataOut[3] = {RED, GREEN, BLUE};
uint8_t RGBcount = 0;
int main(void)
{
    /* Write your code here */
	hardware_init();
	dbg_uart_init();
	OSA_Init();

	GPIO_DRV_Init(switchPins, ledPins);

	printf("SPI master example, press SW2 to send 0x52(RED), 0x47 (GREEN) or 0x42(BLUE) \n\r");


	/*HW_SPI1 -- ALT 7*/
	/* PTD_0  SPI0_PCS0*/
	/* PTD_3  SPI0_SIN*/
	/* PTD_1  SPI0_SCK*/
	/* PTD_2  SPI0_SOUT*/ //signal out
	configure_spi_pins(HW_SPI0);

	dspi_master_state_t dspiMasterState; // simply allocate memory for this
	// configure the members of the user config //
	dspi_master_user_config_t userConfig;
	userConfig.isChipSelectContinuous = false;
	userConfig.isSckContinuous = false;
	userConfig.pcsPolarity = kDspiPcs_ActiveLow;
	userConfig.whichCtar = kDspiCtar0;
	userConfig.whichPcs = kDspiPcs0;  //Selects the Chip select

	// init the DSPI module //
	DSPI_DRV_MasterInit(HW_SPI0, &dspiMasterState, &userConfig);

	// Define bus configuration.
	uint32_t calculatedBaudRate;
	dspi_device_t spiDevice;
	spiDevice.dataBusConfig.bitsPerFrame = 8;
	spiDevice.dataBusConfig.clkPhase = kDspiClockPhase_SecondEdge;
	spiDevice.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
	spiDevice.dataBusConfig.direction = kDspiMsbFirst;
	spiDevice.bitsPerSec = 50000;
	// configure the SPI bus //
	DSPI_DRV_MasterConfigureBus(HW_SPI0, &spiDevice, &calculatedBaudRate);

    for (;;) {

    	if(GPIO_DRV_ReadPinInput(kGpioSW1) == 0)    //is switch pressed?
    	{
    		pushflag = true;
    	}

    	OSA_TimeDelay(250);

    	if(pushflag)
    	{
    		pushflag = false;
    		GPIO_DRV_TogglePinOutput(BOARD_GPIO_LED_BLUE);
    		printf ("Sending 0x%x \n\r",spiSourceBuffer & 0xFF);
    		//SEND DATA
            dspi_status_t Error = DSPI_DRV_MasterTransferBlocking(HW_SPI0, 			//spi instance
    															  NULL, 			//device information structure, is already configured in DSPI_DRV_MasterConfigureBus
    															  &spiSourceBuffer, //send buffer address
    															  &spiSinkBuffer,   //received buffer address
    															  1,				//number of bytes to send and receive
    															  1000);            //timeout in miliseconds
    		//////////////////////////////

            if (Error == kStatus_DSPI_Success)
            {
            	printf ("Transmission succeed  \n\r");
            	printf ("Received data 0x%x \n\n\r",spiSinkBuffer & 0xFF);

            	RGBcount++;
            	if(RGBcount > 2 )
            	{
            		RGBcount = 0;
            	}
            	spiSourceBuffer = RGBdataOut[RGBcount];
            }

    	}

    }
    /* Never leave main */
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
