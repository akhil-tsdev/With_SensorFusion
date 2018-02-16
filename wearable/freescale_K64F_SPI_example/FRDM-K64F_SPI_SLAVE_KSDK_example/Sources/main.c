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
#include "fsl_dspi_slave_driver.h"
#include "fsl_dspi_hal.h"
#include "board.h"

/*RGB values*/
#define RED 	0x52
#define GREEN 	0x47
#define BLUE 	0x42

#define RED_LED			0
#define GREEN_LED		1
#define BLUE_LED		2
#define ALL_ON			3
#define ALL_OFF			4

void Turn_Led (uint8_t Led);

uint8_t spiSourceBuffer = 0xAA;
uint8_t spiSinkBuffer;

int main(void)
{

    /* Write your code here */
	// declare which module instance you want to use
	hardware_init();
	dbg_uart_init();
	OSA_Init();

	GPIO_DRV_Init(switchPins, ledPins);

	Turn_Led (ALL_ON);
	OSA_TimeDelay(1000);
	Turn_Led (ALL_OFF);

	/*HW_SPI1 -- ALT 7*/
	/* PTD_0  SPI0_PCS0*/
	/* PTD_3  SPI0_SIN*/
	/* PTD_1  SPI0_SCK*/
	/* PTD_2  SPI0_SOUT*/ //signal out
	configure_spi_pins(HW_SPI0);

	// Interrupt driven
	dspi_slave_state_t dspiSlaveState;

	// update configs
	dspi_slave_user_config_t slaveUserConfig;
	slaveUserConfig.dataConfig.clkPhase = kDspiClockPhase_SecondEdge;
	slaveUserConfig.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
	slaveUserConfig.dataConfig.bitsPerFrame = 8;
	slaveUserConfig.dataConfig.direction = kDspiMsbFirst;
	slaveUserConfig.dummyPattern = DSPI_DEFAULT_DUMMY_PATTERN;

	// init the slave (interrupt driven)
	DSPI_DRV_SlaveInit(HW_SPI0, &dspiSlaveState, &slaveUserConfig);


    for (;;) {

    	dspi_status_t Error = DSPI_DRV_SlaveTransferBlocking(HW_SPI0,				//spi instance
    														 &spiSourceBuffer,		//send buffer address
    														 &spiSinkBuffer,		//received buffer address
    														 1,						//number of bytes to send and receive
    														 OSA_WAIT_FOREVER);		//timeout OSA_WAIT_FOREVER = blocking

    	if (Error == kStatus_DSPI_Success)
		{

    		switch(spiSinkBuffer)
    		{
    					case((uint8_t)RED):
    						Turn_Led (RED_LED);
    						spiSourceBuffer = 0xA1;
    						break;
    					case((uint8_t)GREEN):
    						Turn_Led (GREEN_LED);
    						spiSourceBuffer = 0xB1;
    						break;
    					case((uint8_t)BLUE):
    						Turn_Led (BLUE_LED);
    						spiSourceBuffer = 0xC1;
    						break;
    					case((uint8_t)0xFF):
    						Turn_Led (ALL_ON);
    						break;
    		}
		}

    }
    /* Never leave main */
    return 0;
}


void Turn_Led (uint8_t Led)
{
	switch(Led){
		case 0:
			GPIO_DRV_ClearPinOutput(BOARD_GPIO_LED_RED);
			GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_GREEN);
			GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_BLUE);
			break;
		case 1:
			GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_RED);
			GPIO_DRV_ClearPinOutput(BOARD_GPIO_LED_GREEN);
			GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_BLUE);
			break;
		case 2:
			GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_RED);
			GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_GREEN);
			GPIO_DRV_ClearPinOutput(BOARD_GPIO_LED_BLUE);
			break;
		case 3:
			GPIO_DRV_ClearPinOutput(BOARD_GPIO_LED_RED);
			GPIO_DRV_ClearPinOutput(BOARD_GPIO_LED_GREEN);
			GPIO_DRV_ClearPinOutput(BOARD_GPIO_LED_BLUE);
			break;
		case 4:
			GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_RED);
			GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_GREEN);
			GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_BLUE);
			break;
	}

}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
