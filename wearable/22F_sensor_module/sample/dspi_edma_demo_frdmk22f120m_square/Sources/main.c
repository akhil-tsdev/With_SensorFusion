/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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
 *
 */
/*******************************************************************************
 * Standard C Included Files
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*******************************************************************************
 * SDK Included Files
 ******************************************************************************/
#include "board.h"
#include "fsl_uart_driver.h"
#include "fsl_edma_driver.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_os_abstraction.h"
/*******************************************************************************
 * Application Included Files
 ******************************************************************************/
#include "data_file.h"
#include "terminal.h"
#include "dspi.h"
#include "edma.h"
/*******************************************************************************
 * CWATI Included Files
 ******************************************************************************/
//#include "MPU9250_SPI.h"
#include "MK22F51212.h"
#include "MK22F51212_spi.h"
#define BASE_ADDR0			0x4002C000u
#define USE_GPIO_PTB19		1

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SPI0_TXCMD (SPI_PUSHR_PCS(0x01) | SPI_PUSHR_CTAS(0x00))
/*******************************************************************************
 * Global Variables
 ******************************************************************************/
/* Semaphore for checking eDMA channel completion. */
semaphore_t g_statusSem;

/* DMA Buffers */
uint32_t g_masterRxBuffer[64];
uint32_t g_slaveRxBuffer[64];
volatile uint16_t g_slaveTxCount;
volatile uint32_t g_errorFlag;

/* Check buffers */
extern const uint32_t g_dataBuffer[];
extern const uint32_t g_expectedReturn[];

/* For EDMA init */
//static edma_state_t g_edmaState;
//static edma_user_config_t g_edmaUserConfig;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void pin_setup(void);

/*******************************************************************************
 * Cwati
 ******************************************************************************/
uint32_t cwati_spi_send(uint8_t writeAddr);

#define CWATI(s, ...) printf("\r\n%s:%d cwati" s, __FUNCTION__,__LINE__, __VA_ARGS__)

//#define printf DONTDOANYTHING;
//int DONTDOANYTHING() {}

/*******************************************************************************
 * Code
 ******************************************************************************/
void pin_setup(void) {
//	configure_spi_pins(kSensorInstance);
	configure_spi_pins(kRFInstance);
}
/******************************************************************************/
void printFIFO(void) {
	for (int i = 0; i < 4; i++) {
		printf("\r\nTX FIFO[%d]=0x%x ", i,
				DSPI_HAL_GetFifoData(SPI0_BASE, kDspiTxFifo, i));
	}
	printf("\r\n");
	for (int i = 0; i < 4; i++) {
		printf("\r\nRX FIFO[%d]=0x%x ", i,
				DSPI_HAL_GetFifoData(SPI0_BASE, kDspiRxFifo, i));
	}
	printf("\r\n\n");
}
void printRegs(int i) {/*
	printf("\r\n printRegs %d", i);
	printf("\r\n HW_SPI_PUSHR=0x%x",HW_SPI_PUSHR_RD(SPI0_BASE));
	printf("\r\n BR_SPI_PUSHR_PCS=0x%x",	BR_SPI_PUSHR_PCS(SPI0_BASE));
	printf("\r\n MCR Module Config Reg=0x%x", HW_SPI_MCR_RD(BASE_ADDR0));
	printf("\r\n SR Status Reg=0x%x", HW_SPI_SR_RD(SPI0_BASE));
	//static inline uint32_t DSPI_HAL_GetTransferCount(uint32_t baseAddr)
	printf("\r\nTCR transfer count: 0x%x", BR_SPI_TCR_SPI_TCNT(SPI0_BASE));
	printFIFO();*/
}

uint32_t test_mpu9250WriteReg(uint8_t WriteAddr, uint8_t WriteData) {
	uint32_t data = 0;

	//mpu9250_spi_select();
	//BW_SPI_PUSHR_PCS(SPI0_BASE, 0x0); //Binary 00000 - deassert
//	printf("\r\n after deassert HW_SPI_PUSHR=0x%x",HW_SPI_PUSHR_RD(SPI0_BASE));
	//printf("\r\ncwati Send1:");
	cwati_spi_send(WriteAddr);
	//printf("\r\ncwati Send2:");
	data = cwati_spi_send(WriteData);
	//mpu9250_spi_deselect();
//	printf("\r\n after send HW_SPI_PUSHR=0x%x",HW_SPI_PUSHR_RD(SPI0_BASE));
	//BW_SPI_PUSHR_PCS(SPI0_BASE, 0x2); //Binary 00000 - deassert
//	printf("\r\n after reassert HW_SPI_PUSHR=0x%x",HW_SPI_PUSHR_RD(SPI0_BASE));
	OSA_TimeDelay(0.050); /* in ms */

	return data;
}
/*
uint32_t test_mpu9250ReadReg(uint8_t ReadAddr) {
	return test_mpu9250WriteReg( ReadAddr | MPU9250_READ_FLAG, 0);
}
*/
int main(void) {
	uint32_t userFreq = 100000; /* 50,000 Hz for now */
	dspi_master_state_t *dspiMasterSensorState =
			(dspi_master_state_t *) OSA_MemAlloc(sizeof(dspi_master_state_t));
	OSA_Init();

	hardware_init();
	dbg_uart_init();
	pin_setup();

#ifdef USE_GPIO_PTB19
	/* Make PTB19 as "SPI1_CS */
    GPIO_DRV_SetPinDir(GPIO_MAKE_PIN(HW_GPIOB, 19), kGpioDigitalOutput); \
    GPIO_DRV_SetPinOutput(GPIO_MAKE_PIN(HW_GPIOB, 19));
    GPIO_DRV_WritePinOutput(GPIO_MAKE_PIN(HW_GPIOB, 19), 1);
#endif

	/* Print welcome message & demo configuration to terminal. */
	print_welcome();
	//cwati(); //calling libraries

	/* DSPI Master Configuration */
	/* Init master (HAL_Init, HAL_Enable, HAL_IsMaster)
	 * Configure bus
	 * Set data format
	 * Set Polarity Mode
	 * Enable fifo operation (regardless of FIFO depth), TX FIFO and RX FIFO */
	//	cwati_dspi_edma_master_setup(dspiMasterSensorState, kSensorInstance, userFreq, 16);
	cwati_dspi_edma_master_setup(dspiMasterSensorState, kRFInstance, userFreq, 16);
	OSA_TimeDelay(100.0 /*ms*/);

	unsigned char mts_buffer[17] = "iiiiiiiiiiiiiiii";
	unsigned char stm_buffer[17] = "Hello TuringSens";
	//unsigned char stm_buffer[17] = "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
	//unsigned char stm_buffer[17] = "----------------";
	//printf("\r\nClock setting is %x\r\n",(SIM_SCGC5[PORTD]));

	while(1) {

		/* JOE: Communicate to Invensense MPU9250 */
//		mpu9250_spi_read_acc();
//		mpu9250_spi_calib_acc();

#ifdef USE_GPIO_PTB19
	    GPIO_DRV_WritePinOutput(GPIO_MAKE_PIN(HW_GPIOB, 19), 0);
#endif
//		DSPI_DRV_MasterTransferData(kSensorInstance, &masterSpiDevice,
	    DSPI_DRV_MasterTransferData(kRFInstance, &masterSpiDevice,
					&mts_buffer[0], &stm_buffer[0], 16);
#ifdef USE_GPIO_PTB19
//		OSA_TimeDelay(20.0 /*ms*/);
#endif
		printf("\r\nReceived buffer <");
		for(int q=0;q<16;++q) {
			printf("[%02x]",stm_buffer[q]);
		}
		printf(">\r\n");
#ifdef USE_GPIO_PTB19
	    GPIO_DRV_WritePinOutput(GPIO_MAKE_PIN(HW_GPIOB, 19), 1);
#endif
		OSA_TimeDelay(1000.0 /*ms*/);

	}
	//int result = mpu9250_spi_init(1,BITS_DLPF_CFG_188HZ);

	//printf("Initialization_result %d",result);
	//printRegs(1);
	//uint8_t response[6];

	//printf("\r\n\nWHOAMI=0x%2x Is it 104??\n", mpu9250_whoami());

//	for (int i = 0x49; i < 0x60; i++) {
//		printf("\r\nMPU9250 0x%x=0x%x",i,test_mpu9250ReadReg(i));
//	}
}

uint32_t cwati_spi_send(uint8_t writeAddr) {
	static int counter = 100;
	cwatiSend[0] = writeAddr;
	printRegs(counter);
	DSPI_DRV_MasterTransferData(kRFInstance, &masterSpiDevice,
//	DSPI_DRV_MasterTransferData(kSensorInstance, &masterSpiDevice,
			&cwatiSend[0], &cwatiRecv[0], 1);
	printRegs(counter++);
	return (uint32_t) cwatiRecv[0];
}

//cwati comment below out for now

//int main (void)
//{
////    uint32_t index;
//    uint8_t msg;
//    uint32_t var;
//    uint32_t userFreq = 3000;
////    osa_status_t syncStatus;
//    dspi_status_t retval = kStatus_OSA_Success;
//
//    volatile uint16_t count;
//
//    dspi_master_state_t *dspiMasterState = (dspi_master_state_t *)OSA_MemAlloc(sizeof(dspi_master_state_t));
//    dspi_master_state_t *dspiMasterSensorState = (dspi_master_state_t *)OSA_MemAlloc(sizeof(dspi_master_state_t));
//
//
////    dspi_slave_state_t *dspiSlaveState = (dspi_slave_state_t *)OSA_MemAlloc(sizeof(dspi_slave_state_t));
////
////    dspi_slave_user_config_t *slaveUserConfig = (dspi_slave_user_config_t *)OSA_MemAlloc(sizeof(dspi_slave_user_config_t));
//
//    OSA_Init();
//
//    /* Create a semephore to check for completed eDMA transfers. */
//    OSA_SemaCreate(&g_statusSem, 0);
//
//    hardware_init();
//    dbg_uart_init();
//
//    #if defined(__GNUC__)
//    setvbuf(stdin, NULL, _IONBF, 0);
//    #endif
//
//    pin_setup();
//
////    /* Generate SPI transmit buffer, with SPI TX Commands. */
////    uint32_t *spiPush = OSA_MemAlloc(sizeof(g_dataBuffer));
////    memset(spiPush, 0, sizeof(g_dataBuffer));
////
////    for(index = 0; index < (sizeof(g_dataBuffer) / sizeof(uint32_t)); index++)
////    {
////        spiPush[index] = SPI0_TXCMD | g_dataBuffer[index];
////    }
//
//    /* Print welcome message & demo configuration to terminal. */
//    print_welcome();
//
//    print_configuration();
//
//    print_pin_setup();
//
//    cwati(); //calling libraries
//
//    /*
//     *
//     * Get SPI CLK frequency from user. Freq must be between 3 kHz & 2 MHz for demo purposes.
//     * Software can disable eDMA channels before transfer is complete for lower SPI CLK frequencies.
//     * For higher frequencies the software cannot keep up when generating the interrupt responses from the slave.
//     *
//     */
//    userFreq = get_spi_freq();
//
////    /* Initialize eDMA & DMAMUX */
////    g_edmaUserConfig.chnArbitration = kEDMAChnArbitrationRoundrobin;
////    g_edmaUserConfig.notHaltOnError = false;
////    EDMA_DRV_Init(&g_edmaState, &g_edmaUserConfig);
////
////    /* eDMA Channel 0 Configuration for transmitting data from DSPI0. */
////    edma_chn_state_t dmaCh0;
////
////    edma_loop_setup_t *dmaLoop0 = (edma_loop_setup_t *)OSA_MemAlloc(sizeof(edma_loop_setup_t));
////
////    dmaLoop0->dmaChanNum = DMA_CH0;
////    dmaLoop0->dmaCh = &dmaCh0;
////    dmaLoop0->type = kEDMAMemoryToPeripheral;
////    dmaLoop0->chSource = kDmaRequestMux0SPI0Tx;
////    dmaLoop0->srcAddr = (uint32_t)&spiPush[0];
////    dmaLoop0->destAddr = (uint32_t)&SPI0->PUSHR;
////    dmaLoop0->length = 0x100U;
////    dmaLoop0->size = 0x04U;
////    dmaLoop0->watermark = 0x04U;
////    dmaLoop0->period = 0x01U;
////    dmaLoop0->dmaCallBack = stop_edma_loop;
////
////    setup_edma_loop(dmaLoop0);
////
////    /* eDMA Channel 1 Configuration for receiving data on DSPI0. */
////    edma_chn_state_t dmaCh1;
////
////    edma_loop_setup_t *dmaLoop1 = (edma_loop_setup_t *)OSA_MemAlloc(sizeof(edma_loop_setup_t));
////
////    dmaLoop1->dmaChanNum = DMA_CH1;
////    dmaLoop1->dmaCh = &dmaCh1;
////    dmaLoop1->type = kEDMAPeripheralToMemory;
////    dmaLoop1->chSource = kDmaRequestMux0SPI0Rx;
////    dmaLoop1->srcAddr = (uint32_t)&SPI0->POPR;
////    dmaLoop1->destAddr = (uint32_t)&g_masterRxBuffer;
////    dmaLoop1->length = 0x100U;
////    dmaLoop1->size = 0x04U;
////    dmaLoop1->watermark = 0x04U;
////    dmaLoop1->period = 0x01U;
////    dmaLoop1->dmaCallBack = stop_edma_loop;
////
////    setup_edma_loop(dmaLoop1);
////
////    /* eDMA Channel 2 Configuration for receiving data on DSPI1. */
////    edma_chn_state_t dmaCh2;
////
////    edma_loop_setup_t *dmaLoop2 = (edma_loop_setup_t *)OSA_MemAlloc(sizeof(edma_loop_setup_t));
////
////    dmaLoop2->dmaChanNum = DMA_CH2;
////    dmaLoop2->dmaCh = &dmaCh2;
////    dmaLoop2->type = kEDMAPeripheralToMemory;
////    dmaLoop2->chSource = kDmaRequestMux0SPI1;
////    dmaLoop2->srcAddr = (uint32_t)&SPI1->POPR;
////    dmaLoop2->destAddr = (uint32_t)&g_slaveRxBuffer;
////    dmaLoop2->length = 0x100U;
////    dmaLoop2->size = 0x04U;
////    dmaLoop2->watermark = 0x04U;
////    dmaLoop2->period = 0x01U;
////    dmaLoop2->dmaCallBack = stop_edma_loop;
////
////    setup_edma_loop(dmaLoop2);
//
//    /* DSPI Master Configuration */
//    dspi_edma_master_setup(dspiMasterState, kRFInstance, userFreq, 16);
//	printf("\r\n\r\ncwatiSend\r\n\r\n");
//	for(count = 0; count < sizeof(cwatiSend); count++)
//	{
//		var = cwatiSend[count] & 0x0000FFFF;
//		printf("\r\n%08X\t", (unsigned int)var);
//		if((count + 1) % 4 == 0)
//		{
//			printf("\r\n\r\n");
//		}
//	}
//	/* Print out original contents of cwatiRecv. */
//	printf("\r\n\r\n\r\ncwatiRecv BEFORE\r\n\r\n");
//	for(count = 0; count < sizeof(cwatiRecv); count++)
//	{
//		printf("\r\n%08X\t", (unsigned int)cwatiRecv[count]);
//		if((count + 1) % 4 == 0)
//		{
//			printf("\r\n\r\n");
//		}
//	}
////    /* Print out spiPush. */
////    printf("\r\n\r\nspiPush\r\n\r\n");
////    for(count = 0; count < 64; count++)
////    {
////        var = spiPush[count] & 0x0000FFFF;
////        printf("\r\n%08X\t", (unsigned int)var);
////        if((count + 1) % 4 == 0)
////        {
////            printf("\r\n");
////        }
////    }
////
////    /* Print out starting contents of g_slaveRxBuffer. */
////    printf("\r\ng_slaveRxBuffer\r\n\r\n");
////    for(count = 0; count < 64; count++)
////    {
////        var = g_slaveRxBuffer[count];
////        printf("\r\n%08X\t", (unsigned int)var);
////        if((count + 1) % 4 == 0)
////        {
////            printf("\r\n");
////        }
////    }
////
////    /* Print out starting contents of g_masterRxBuffer. */
////    printf("\r\n\r\nmasterRxBuffer\r\n\r\n");
////    for(count = 0; count < 64; count++)
////    {
////        printf("%08X\t", (unsigned int)g_masterRxBuffer[count]);
////        if((count + 1) % 4 == 0)
////        {
////            printf("\r\n");
////        }
////    }
//
//    printf("\r\n\r\n\r\nPress space bar to begin.\r\n");
//    msg = 'A';
//    while(msg != ' ')
//    {
//
//        msg = getchar();
//
//    }
////    /* DSPI Slave Configuration */
////    dspi_edma_slave_setup(dspiSlaveState, slaveUserConfig, kSlaveInstance, 16);
//
//    printf("\r\n%c\r\n", msg);
//
//    printf("\r\nDemo started...\r\n");
//
//	/* Print out cwatiSend. */
//
//	printf("CWATI Starting to send...\n");
//	retval = DSPI_DRV_MasterTransferDataBlocking(kRFInstance, NULL, cwatiSend, cwatiRecv, sizeof(cwatiSend), 5000);
//	if (retval == kStatus_DSPI_Success) {
//	    CWATI("Return %s", "success!!");
//	} else {
//		CWATI("FAILED! Return val is %d",retval);
//	}
//
//	printf("\r\ncwatiSend AFTER\r\n\r\n");
//	for(count = 0; count < sizeof(cwatiSend); count++)
//	{
//		var = cwatiSend[count] & 0x0000FFFF;
//		printf("%08X\t", (unsigned int)var);
//		if((count + 1) % 4 == 0)
//		{
//			printf("\r\n");
//		}
//	}
//	/* Print out new contents of cwatiRecv. */
//	printf("\r\n\r\ncwatiRecv AFTER\r\n\r\n");
//	for(count = 0; count < sizeof(cwatiRecv); count++)
//	{
//		printf("%08X\t", (unsigned int)cwatiRecv[count]);
//		if((count + 1) % 4 == 0)
//		{
//			printf("\r\n");
//		}
//	}
////    /* Enable eDMA channels requests to initiate DSPI transfers. */
////    EDMA_DRV_StartChannel(dmaLoop2->dmaCh);
////    EDMA_DRV_StartChannel(dmaLoop1->dmaCh);
////    EDMA_DRV_StartChannel(dmaLoop0->dmaCh);
//
////    while(1)
////    {
////        if(g_errorFlag == 0x00DEAD00)
////        {
////            /* Disable eDMA loops. */
////            disable_edma_loop(dmaLoop0);
////            disable_edma_loop(dmaLoop1);
////            disable_edma_loop(dmaLoop2);
////            printf("\r\nDemo Complete.\r\n\r\n%d loops completed.\r\n", g_slaveTxCount);
////
////            /* Print out new contents of g_slaveRxBuffer. */
////            printf("\r\ng_slaveRxBuffer\r\n\r\n");
////            for(count = 0; count < 64; count++)
////            {
////                var = g_slaveRxBuffer[count];
////                printf("%08X\t", (unsigned int)var);
////                if((count + 1) % 4 == 0)
////                {
////                    printf("\r\n");
////                }
////            }
////
////            /* Compare g_slaveRxBuffer to data side of g_dataBuffer */
////            if(memcmp(g_dataBuffer, g_slaveRxBuffer, sizeof(g_dataBuffer)))
////            {
////                printf("\r\nFail... the slave did not receive all the bytes from the master corectly. Check connections.\r\n");
////            }
////            else
////            {
////                 printf("\r\nSuccess! Slave received all the bytes from the Master.\r\n");
////            }
////
////            /* Print out new contents of g_masterRxBuffer. */
////            printf("\r\n\r\nmasterRxBuffer\r\n\r\n");
////            for(count = 0; count < 64; count++)
////            {
////                printf("%08X\t", (unsigned int)g_masterRxBuffer[count]);
////                if((count + 1) % 4 == 0)
////                {
////                    printf("\r\n");
////                }
////            }
////
////            printf("\r\nFinal value of g_slaveTxCount sent:  0x%X\r\n", (unsigned int)(g_slaveTxCount - 1));
////
////            /* Check g_masterRxBuffer with g_expectedReturn */
////            if(memcmp(g_expectedReturn, g_masterRxBuffer, sizeof(g_expectedReturn)))
////            {
////                printf("\r\nFail... the Master did not receive all the bytes from the slave correctly. Check connections.\r\n");
////            }
////            else
////            {
////                 printf("\r\nSuccess! The Master received all the bytes from the slave correctly.\r\n");
////            }
////
////            OSA_MemFree(spiPush);
////
////            g_errorFlag = 0;
////
////            dspi_edma_deinit(dspiMasterState, dspiSlaveState, slaveUserConfig);
////
////            OSA_SemaDestroy(&g_statusSem);
////
////            printf("\r\nEnd of demo.\r\n");
////
////            while(1)
////            {
////                __asm("NOP");
////            }/* Wait here forever. */
////        }
////        else
////        {
////            /* Check to see if eDMA channels are competing their transfers. */
////            do
////            {
////                syncStatus = OSA_SemaWait(&g_statusSem, 5000);
////            }while(syncStatus == kStatus_OSA_Idle);
////
////            /* Set the flag to terminate the demo if an eDMA channel has taken too long to transfer data. */
////            if(syncStatus == kStatus_OSA_Timeout)
////            {
////                g_errorFlag = 0x00DEAD00;
////            }
////        }
////    }
//}

/******************************************************************************
 * EOF
 ******************************************************************************/
