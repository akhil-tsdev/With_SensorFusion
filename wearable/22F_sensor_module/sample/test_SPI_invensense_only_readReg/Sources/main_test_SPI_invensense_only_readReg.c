/*
 * TuringSense 2015
 * cwati
 *
 * This project aims to read PCR registers for SPI settings.
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
#include "MK22F12810.h"			/* For SPI1_BASE and SPI0_BASE */
#include "MPU9250_SPI.h"		/* MPU 9250 */
#include "fsl_clock_manager.h"	/* For checking port clock */
#include "fsl_gpio_driver.h"	/* For gpio driver */
#define MPU9250_READ_FLAG   	    0x80

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define SPI0_TXCMD (SPI_PUSHR_PCS(0x01) | SPI_PUSHR_CTAS(0x00))
/*******************************************************************************
 * Global Variables
 ******************************************************************************/
/* GPIO pins */
gpio_output_pin_user_config_t ledPinGreen;
gpio_output_pin_user_config_t ledPinRed;
gpio_output_pin_user_config_t ledPinBlue;
gpio_output_pin_user_config_t spi1_pcs0_pin;
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

/*******************************************************************************
 * Code
 ******************************************************************************/
void pin_setup(void) {
	configure_spi_pins(kSensorInstance);
}
/******************************************************************************/

void select_SPI1_pcs0(void) {
	/* active low */
    GPIO_DRV_ClearPinOutput(spi1_pcs0_pin.pinName);
}

void deselect_SPI1_pcs0(void) {
    GPIO_DRV_SetPinOutput(spi1_pcs0_pin.pinName);
}

void config_scratchRegPins(void) {
	int tmp;

	/* GREEN */
	ledPinGreen.config.outputLogic = 0;
	ledPinGreen.config.slewRate = kPortSlowSlewRate;
	ledPinGreen.config.driveStrength = kPortHighDriveStrength;
	ledPinGreen.pinName = GPIO_MAKE_PIN(HW_GPIOA, 5);

	GPIO_DRV_OutputPinInit(&ledPinGreen);

	/* LEDs active low, will turn on */
    GPIO_DRV_ClearPinOutput(ledPinGreen.pinName);

	/* Sensor is SPI1 PCS0 */
	spi1_pcs0_pin.config.outputLogic = 0;
	spi1_pcs0_pin.config.slewRate = kPortSlowSlewRate;
	spi1_pcs0_pin.config.driveStrength = kPortHighDriveStrength;
	spi1_pcs0_pin.pinName = GPIO_MAKE_PIN(HW_GPIOD, 4);

	GPIO_DRV_OutputPinInit(&spi1_pcs0_pin);
    GPIO_DRV_ClearPinOutput(spi1_pcs0_pin.pinName);	/* Set pin low */

    tmp = GPIO_DRV_ReadPortOutput(ledPinGreen.pinName);
    tmp = GPIO_DRV_ReadPortOutput(spi1_pcs0_pin.pinName);	/* cwati this should give 0x00 for PTD4 set low */
}


uint32_t test_mpu9250WriteReg(uint8_t WriteAddr, uint8_t WriteData) {
	uint32_t data = 0, tmp;

//	select_SPI1_pcs0();
    tmp = GPIO_DRV_ReadPortOutput(spi1_pcs0_pin.pinName);
	cwati_spi_send(WriteAddr);
	data = cwati_spi_send(WriteData);
//	deselect_SPI1_pcs0();
    tmp = GPIO_DRV_ReadPortOutput(spi1_pcs0_pin.pinName);
	OSA_TimeDelay(0.050); /* in ms */
	return data;
}

uint32_t test_mpu9250ReadReg(uint8_t ReadAddr) {
	uint32_t val = test_mpu9250WriteReg( ReadAddr | MPU9250_READ_FLAG, 0);
	return val;
}

int main(void) {
	uint32_t userFreq = 100000, tmp = 0; /* 10 kHz for now */
	uint32_t PT[5][32];
	uint32_t PTafter[5][32];

	dspi_master_state_t *dspiMasterSensorState =
			(dspi_master_state_t *) OSA_MemAlloc(sizeof(dspi_master_state_t));
	OSA_Init();

	hardware_init();
	pin_setup();

	cwati_dspi_edma_master_setup(dspiMasterSensorState, kSensorInstance,
			userFreq, 8);

	config_scratchRegPins();

	/* Send reading before SPI to Mike */
	for (unsigned int pt = 0; pt < 4; pt++) {
		for (unsigned int reg = 0; reg < 33; reg++) {
			if (pt == 0) {
				PT[pt][reg] = HW_PORT_PCRn_RD(PORTA_BASE, reg);
			} else if (pt == 1) {
				PT[pt][reg] = HW_PORT_PCRn_RD(PORTB_BASE, reg);
			} else if (pt == 2) {
				PT[pt][reg] = HW_PORT_PCRn_RD(PORTC_BASE, reg);
			} else if (pt == 3) {
				PT[pt][reg] = HW_PORT_PCRn_RD(PORTD_BASE, reg);
			} else if (pt == 4) {
				PT[pt][reg] = HW_PORT_PCRn_RD(PORTE_BASE, reg);
			}
		}
	}
    mpu9250_spi_init(1,BITS_DLPF_CFG_188HZ);
	tmp = mpu9250_whoami();
	for (int i = 0x49; i < 0x60; i++) {
		tmp = test_mpu9250ReadReg(i);
	}

	/* Send reading after SPI to Mike */
	for (unsigned int pt = 0; pt < 4; pt++) {
		for (unsigned int reg = 0; reg < 33; reg++) {
			if (pt == 0) {
				PTafter[pt][reg] = HW_PORT_PCRn_RD(PORTA_BASE, reg);
			} else if (pt == 1) {
				PTafter[pt][reg] = HW_PORT_PCRn_RD(PORTB_BASE, reg);
			} else if (pt == 2) {
				PTafter[pt][reg] = HW_PORT_PCRn_RD(PORTC_BASE, reg);
			} else if (pt == 3) {
				PTafter[pt][reg] = HW_PORT_PCRn_RD(PORTD_BASE, reg);
			} else if (pt == 4) {
				PTafter[pt][reg] = HW_PORT_PCRn_RD(PORTE_BASE, reg);
			}
		}
	}
	for(;;) {
		//stops here
	}

}

uint32_t cwati_spi_send(uint8_t writeAddr) {
	static int counter = 100;
	uint8_t send = 0, recv = 0;

	send = writeAddr;
	DSPI_DRV_MasterTransferData(kSensorInstance, &masterSpiDevice,
			&send, &recv, 1);
	return (uint32_t) recv;
}

/**************************************** Debug prints **************************************/
/* Debug print for SPI1 == sensor module */
void printFIFO_spi1(void) {
	for (int i = 0; i < 4; i++) {
		printf("\r\nTX FIFO[%d]=0x%x ", i,
				DSPI_HAL_GetFifoData(SPI1_BASE, kDspiTxFifo, i));
	}
	printf("\r\n");
	for (int i = 0; i < 4; i++) {
		printf("\r\nRX FIFO[%d]=0x%x ", i,
				DSPI_HAL_GetFifoData(SPI1_BASE, kDspiRxFifo, i));
	}
	printf("\r\n\n");
}
void printRegs_spi1(int i) {
	unsigned int tmp = 0;

	tmp = HW_SPI_PUSHR_RD(SPI1_BASE);
	tmp = BR_SPI_PUSHR_PCS(SPI1_BASE);
	tmp = HW_SPI_MCR_RD(SPI1_BASE);

	printf("\r\n printRegs_spi1 %d", i);
	printf("\r\n HW_SPI_PUSHR=0x%x",HW_SPI_PUSHR_RD(SPI1_BASE));
	printf("\r\n BR_SPI_PUSHR_PCS=0x%x",	BR_SPI_PUSHR_PCS(SPI1_BASE));
	printf("\r\n MCR Module Config Reg=0x%x", HW_SPI_MCR_RD(SPI1_BASE));
	printf("\r\n SR Status Reg=0x%x", HW_SPI_SR_RD(SPI1_BASE));
	//static inline uint32_t DSPI_HAL_GetTransferCount(uint32_t baseAddr)
	printf("\r\nTCR transfer count: 0x%x", BR_SPI_TCR_SPI_TCNT(SPI1_BASE));
//	printFIFO_spi1();
}

/******************************************************************************
 * EOF
 ******************************************************************************/
