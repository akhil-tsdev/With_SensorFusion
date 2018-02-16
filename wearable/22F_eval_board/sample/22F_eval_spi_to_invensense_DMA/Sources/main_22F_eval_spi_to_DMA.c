/*
 * Copyright (c) 2015, Turingsense.
 * All rights reserved.
 *
 * cwati
 *
 * This project for the EVAL board of the sensor module (22F) to talk to
 * Invensense.  NOTE: for the sensor module itself, it requires a different
 * project.
 *
 * Trying to use DMA...not working yet 3/1/15
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
#include "MPU9250_SPI.h"
#include "MK22F51212.h"
#include "MK22F51212_spi.h"
#define MPU9250_READ_FLAG   	    0x80

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SPI0_TXCMD (SPI_PUSHR_PCS(0x01) | SPI_PUSHR_CTAS(0x00))
/*******************************************************************************
 * Global Variables
 ******************************************************************************/
/* Semaphore for checking eDMA channel completion. */
semaphore_t g_statusSem;
/* GPIO pins */
gpio_output_pin_user_config_t ledPinGreen;

/* DMA Buffers */
uint32_t g_masterRxBuffer[64];
uint32_t g_slaveRxBuffer[64];
volatile uint16_t g_slaveTxCount;
volatile uint32_t g_errorFlag;

/* Check buffers */
extern const uint32_t g_dataBuffer[];
extern const uint32_t g_expectedReturn[];

/* For EDMA init */
static edma_state_t g_edmaState;
static edma_user_config_t g_edmaUserConfig;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * cwati
 ******************************************************************************/
uint32_t cwati_spi_send(uint8_t writeAddr);
#define CWATI(s, ...) printf("\r\n%s:%d cwati" s, __FUNCTION__,__LINE__, __VA_ARGS__)

//#define printf DONTDOANYTHING;
//int DONTDOANYTHING() {}

/*******************************************************************************
 * Code
 ******************************************************************************/

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
void printRegs(int i) {
//	printf("\r\n printRegs %d", i);
//	printf("\r\n HW_SPI_PUSHR=0x%x",HW_SPI_PUSHR_RD(SPI0_BASE));
//	printf("\r\n BR_SPI_PUSHR_PCS=0x%x",	BR_SPI_PUSHR_PCS(SPI0_BASE));
//	printf("\r\n MCR Module Config Reg=0x%x", HW_SPI_MCR_RD(BASE_ADDR0));
//	printf("\r\n SR Status Reg=0x%x", HW_SPI_SR_RD(SPI0_BASE));
//	//static inline uint32_t DSPI_HAL_GetTransferCount(uint32_t baseAddr)
//	printf("\r\nTCR transfer count: 0x%x", BR_SPI_TCR_SPI_TCNT(SPI0_BASE));
//	printFIFO();
}

void config_scratchRegPins(void) {
	int tmp;

	/* GREEN */
	ledPinGreen.config.outputLogic = 0;
	ledPinGreen.config.slewRate = kPortSlowSlewRate;
	ledPinGreen.config.driveStrength = kPortHighDriveStrength;
	ledPinGreen.pinName = GPIO_MAKE_PIN(HW_GPIOA, 2);

	GPIO_DRV_OutputPinInit(&ledPinGreen);

	/* LEDs active low, will turn on */
    GPIO_DRV_ClearPinOutput(ledPinGreen.pinName);
}

int main(void) {
	uint32_t userFreq = 50000, tmp = 0; /* 50 kHz for now */
	uint32_t index;
	uint32_t instance = kSensorInstance;
    osa_status_t syncStatus;
#define mtsBUFSIZE 128

    volatile uint16_t count;

	dspi_master_state_t *dspiMasterSensorState =
			(dspi_master_state_t *) OSA_MemAlloc(sizeof(dspi_master_state_t));
	OSA_Init();

    /* Create a semephore to check for completed eDMA transfers. */
    OSA_SemaCreate(&g_statusSem, 0);

	hardware_init();
	dbg_uart_init();

	#if defined(__GNUC__)
	setvbuf(stdin, NULL, _IONBF, 0);
	#endif

	configure_spi_pins(instance);

	config_scratchRegPins();  /* Turn on green LED */

	uint32_t mts_buffer[mtsBUFSIZE];	/* Sending buffer */
	uint32_t stm_buffer[mtsBUFSIZE]; 	/* Receiving buffer */
	for (uint32_t i = 0; i < mtsBUFSIZE; i++) {
		mts_buffer[i] = i | MPU9250_READ_FLAG;
		stm_buffer[i] = 0x0;
	}

//    /* Generate SPI transmit buffer, with SPI TX Commands. */
//    uint32_t spiPush[128];
//    for(index = 0; index < 128; index++)
//    {
//        spiPush[index] = SPI0_TXCMD | g_dataBuffer[index];
//    }

    /* Initialize eDMA & DMAMUX */
    g_edmaUserConfig.chnArbitration = kEDMAChnArbitrationRoundrobin;
    g_edmaUserConfig.notHaltOnError = false;
    EDMA_DRV_Init(&g_edmaState, &g_edmaUserConfig);

    /* eDMA Channel 0 Configuration for transmitting data from DSPI0. */
    edma_chn_state_t dmaCh0;
    edma_loop_setup_t *dmaLoop0 = (edma_loop_setup_t *)OSA_MemAlloc(sizeof(edma_loop_setup_t));

    dmaLoop0->dmaChanNum = DMA_CH0;
    dmaLoop0->dmaCh = &dmaCh0;
    dmaLoop0->type = kEDMAMemoryToPeripheral;
    dmaLoop0->chSource = kDmaRequestMux0SPI0Tx;
    dmaLoop0->srcAddr = (uint32_t)&mts_buffer[0];
    dmaLoop0->destAddr = (uint32_t)&SPI0->PUSHR;
    dmaLoop0->length = 0x100U;
    dmaLoop0->size = 0x04U;
    dmaLoop0->length = mtsBUFSIZE;
    dmaLoop0->size = 2;
    dmaLoop0->watermark = 0x04U;
    dmaLoop0->period = 0x01U;
    dmaLoop0->dmaCallBack = stop_edma_loop;

    setup_edma_loop(dmaLoop0);

    /* eDMA Channel 1 Configuration for receiving data on DSPI0. */
    edma_chn_state_t dmaCh1;

    edma_loop_setup_t *dmaLoop1 = (edma_loop_setup_t *)OSA_MemAlloc(sizeof(edma_loop_setup_t));

    dmaLoop1->dmaChanNum = DMA_CH1;
    dmaLoop1->dmaCh = &dmaCh1;
    dmaLoop1->type = kEDMAPeripheralToMemory;
    dmaLoop1->chSource = kDmaRequestMux0SPI0Rx;
    dmaLoop1->srcAddr = (uint32_t)&SPI0->POPR;
    dmaLoop1->destAddr = (uint32_t)&stm_buffer;
    dmaLoop1->length = mtsBUFSIZE;
    dmaLoop1->size = 2;
    dmaLoop1->length = 0x100U;
    dmaLoop1->size = 0x04U;
    dmaLoop1->watermark = 0x04U;
    dmaLoop1->period = 0x01U;
    dmaLoop1->dmaCallBack = stop_edma_loop;

    setup_edma_loop(dmaLoop1);

	cwati_dspi_edma_master_setup(dspiMasterSensorState, instance,
			userFreq, 16);

	edma_status_t edma_status;
    /* Enable eDMA channels requests to initiate DSPI transfers. */
	edma_status = EDMA_DRV_StartChannel(dmaLoop1->dmaCh);
	edma_status = EDMA_DRV_StartChannel(dmaLoop0->dmaCh);

    while(1)
    {
            if(g_errorFlag == 0x00DEAD00)
            {
                /* Disable eDMA loops. */
                disable_edma_loop(dmaLoop0);
                disable_edma_loop(dmaLoop1);
                printf("\r\nDemo Complete.\r\n\r\n");

                /* Print out new contents of g_masterRxBuffer. */
                printf("\r\n\r\nMaster's receiving stm_buffer\r\n\r\n");
                for(count = 0; count < mtsBUFSIZE; count++)
                {
                    printf("%08X\t", (unsigned int)stm_buffer[count]);
                    if((count + 1) % 4 == 0)
                    {
                        printf("\r\n");
                    }
                }

//                OSA_MemFree(spiPush);

                g_errorFlag = 0;

                master_dspi_edma_deinit(dspiMasterSensorState);

                OSA_SemaDestroy(&g_statusSem);

                printf("\r\nEnd of demo.\r\n");

                while(1)
                {
                    __asm("NOP");
                }/* Wait here forever. */
            }
            else
            {
                /* Check to see if eDMA channels are competing their transfers. */
                do
                {
                    syncStatus = OSA_SemaWait(&g_statusSem, 5000);
                } while(syncStatus == kStatus_OSA_Idle);

                /* Set the flag to terminate the demo if an eDMA channel has taken too long to transfer data. */
                if(syncStatus == kStatus_OSA_Timeout)
                {
                    g_errorFlag = 0x00DEAD00;
                }
            }
        }





//	/* Bulk read, SPI blocking read */
//
//	dspi_status_t Error;
//
//	while(1) {
//		Error = DSPI_DRV_MasterTransferDataBlocking(instance, &masterSpiDevice,
//					&mts_buffer[0], &stm_buffer[0], 128, 1000);
//
//		OSA_TimeDelay(20.0 /*ms*/);
//	}

}

/******************************************************************************
 * EOF
 ******************************************************************************/
