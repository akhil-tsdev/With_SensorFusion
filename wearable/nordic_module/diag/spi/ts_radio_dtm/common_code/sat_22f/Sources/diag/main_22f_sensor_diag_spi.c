/*
 * Copyright (c) 2015 TuringSense
 *
 * main_22f_sensor.c
 *
 * TuringSense K22F Sensor Module.
 * Read from sensor is using SPI.  Initializing Magnetometer is using I2C.
 *
 * This is the main file for satellite 22F code.
 * Different HW revision will use the same set of files, only different
 * defines as follows:
 *
 * For sensor module: PROTO_BOARD
 * For 1st production: PRODUCTION1
 */
/*******************************************************************************
 * Standard C Include Files
 ******************************************************************************/
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * SDK Include Files
 ******************************************************************************/
#include "fsl_device_registers.h"
#include "fsl_os_abstraction.h"
#include "fsl_dspi_master_driver.h"
#include "fsl_dspi_slave_driver.h"
#include "board.h"
#include "mpu9250_firmware.h"
#include "mpu9250_freq.h"
#include "ts_rtc.h"
#include "ts_printf.h"

///* USB */
//void VirtualCom_Deinit(); //virtual_com.h
//#include "fsl_debug_console.h"
//#include "ts_printf.h"

/*******************************************************************************
 * Freescale Open Source Sensor Fusion (OSSF) Include Files
 ******************************************************************************/
#include "build.h"
#include "tasks.h"
#include "magnetic.h"
/*******************************************************************************
 * TuringSense Include Files
 ******************************************************************************/
#include "common_types.h"
#include "common_err.h"
#include "sat_module.h"
#include "ts_fusion.h"
#include "cbuf.h"
#include "crc16.h"
#include "main_22f_sensor.h"

#define NORDIC_CALLBACK_TODO 0

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SAT_ID			234 /* TODO later will read from memory region */

/*******************************************************************************
 * BIAS DETECTION AND BIAS SETTING *** NOTE! VALID FOR SINGOL MEMS
 ******************************************************************************/
#define SAT_BIAS_SENDING 	false
#define SAT_MAGBIAS_X 	 0
#define SAT_MAGBIAS_Y    0
#define SAT_MAGBIAS_Z    0

#define SAT_BIAS_ALWAYSACTIVECALIBRATION 	false //Even if set the fixed values of BIAS (with SAT_BIAS_SENDING=false), calibration of the magnetometer remains active. In this case the calibration values will be recalculated

#define USE_HWE_DEBUG_CONFIGURATION 1
static bool read_sensors_launch_fusion (sensor_data_t *raw_data, sensor_record_t* data);

#if PACE_SENSOR_OUTPUT
static bool alternateSending = true;
#endif


hub_err_t hub_err_status = hub_no_err;

uint32_t reset_rtc_diff = 0; /* Elapsed time before resetting RTC happens.*/

static volatile bool hw_init_done = false;

/* Heartbeat check for Nordic communication */
uint32_t last_nordic_comm;
const uint32_t max_nordic_quiet = 2500; /* in ms */

#if PROTO_BOARD
#define RED_LED_GPIO	4   /* K22F's GPIOA pin number for red LED */
#define GREEN_LED_GPIO	5   /* K22F's GPIOA pin number for green LED */
#define BLUE_LED_GPIO	12  /* K22F's GPIOA pin number for blue LED */
#else
#define RED_LED_GPIO	12   /* K22F's GPIOA pin number for red LED */
#define GREEN_LED_GPIO	13   /* K22F's GPIOA pin number for green LED */
#define BLUE_LED_GPIO	14  /* K22F's GPIOA pin number for blue LED */
#define NORDIC_SWDIO_NRST	19	/* K22F's PTB 19 is for Nordic soft reset */
#endif

#define MAX(a,b) ((a>b)?(a):(b))
#define SPI_COMM_LENGTH MAX(sizeof(sat_mosi_packet_t),sizeof(sat_miso_packet_t))
/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/* SPI related */
typedef union {
	sat_mosi_packet_t packet;
	uint8_t bytes[/*sizeof(sat_mosi_packet_t)*/SPI_COMM_LENGTH];
} receivedCommand_t;
typedef union dataToSend_ {
	sat_miso_packet_t data;
	uint8_t bytes[sizeof(sat_miso_packet_t)];
} dataToSend_t;

dataToSend_t dataToSend;
receivedCommand_t spiReceiveBuffer;

extern uint32_t g_dspiBaseAddr[];
static dspi_slave_state_t nrf51822_dspiSlaveState;


static cbuf_t sensorRecordQ = { 0 }; // Queue of sensor data records (see sat_module.h and cbuf.h)
// Power up defaults: don't collect sensor data, but if told to start
// collecting, send it right away.
static volatile bool sensor_active = false;
static volatile bool sensor_sending = false;

//static uint32_t timeAtBootInMillis = 0;
static int currSourceByte = 0;  // SPI data byte position for SPI slave
static gpio_output_pin_user_config_t ledRedPin;    // Red LED pin
static gpio_output_pin_user_config_t ledGreenPin;  // Green LED pin
static gpio_output_pin_user_config_t ledBluePin;   // Blue LED pin
#if PRODUCTION1
static gpio_output_pin_user_config_t nordicSwdioNrst;
#endif
dspi_status_t nrf51822_init();
#if ENABLE_RTC
uint32_t k22f_get_rtc();
#endif
static void sensorFusionInit();

/* EM USB related stuff */
enum _mainState
{
	mainUSBstart,
	mainUSBstartWait,
	mainSensorInitialization,
	mainLoop
};
static enum _mainState mainState = mainUSBstart;

extern void VC_main();
extern void APP_init();
extern void Task_Start(void *arg);
void Main_Start(void *arg);

void Test_task(void);

/*******************************************************************************
 * Turn on LED of requested color
 ******************************************************************************/
void turnOnLED(sat_led_t color) {
	// LED is active low, clear pin output will turn on
	switch (color) {
	case offLed:
		GPIO_DRV_SetPinOutput(ledGreenPin.pinName);
		GPIO_DRV_SetPinOutput(ledBluePin.pinName);
		GPIO_DRV_SetPinOutput(ledRedPin.pinName);
		break;
	case redLed:
		GPIO_DRV_SetPinOutput(ledGreenPin.pinName);
		GPIO_DRV_SetPinOutput(ledBluePin.pinName);
		GPIO_DRV_ClearPinOutput(ledRedPin.pinName);
		break;
	case greenLed:
		GPIO_DRV_ClearPinOutput(ledGreenPin.pinName);
		GPIO_DRV_SetPinOutput(ledBluePin.pinName);
		GPIO_DRV_SetPinOutput(ledRedPin.pinName);
		break;
	case blueLed:
		GPIO_DRV_SetPinOutput(ledGreenPin.pinName);
		GPIO_DRV_ClearPinOutput(ledBluePin.pinName);
		GPIO_DRV_SetPinOutput(ledRedPin.pinName);
		break;
	case yellowLed:
		GPIO_DRV_ClearPinOutput(ledGreenPin.pinName);
		GPIO_DRV_SetPinOutput(ledBluePin.pinName);
		GPIO_DRV_ClearPinOutput(ledRedPin.pinName);
		break;
	case purpleLed:
		GPIO_DRV_SetPinOutput(ledGreenPin.pinName);
		GPIO_DRV_ClearPinOutput(ledBluePin.pinName);
		GPIO_DRV_ClearPinOutput(ledRedPin.pinName);
		break;
	case turqoiseLed:
	default:
		GPIO_DRV_ClearPinOutput(ledGreenPin.pinName);
		GPIO_DRV_ClearPinOutput(ledBluePin.pinName);
		GPIO_DRV_SetPinOutput(ledRedPin.pinName);
		break;
	}
}

/*******************************************************************************
 * Nordic soft reset
 * Using SWDIO_nRST_K22F_nRF51
 * Minimum 100ms-150ms delay in between.
 ******************************************************************************/
void nrf51822_swdio_reset() {
	uint32_t green, blue, red;
	green = GPIO_DRV_ReadPinInput(ledGreenPin.pinName);
	blue = GPIO_DRV_ReadPinInput(ledBluePin.pinName);
	red = GPIO_DRV_ReadPinInput(ledRedPin.pinName);

	GPIO_DRV_ClearPinOutput(nordicSwdioNrst.pinName);

	turnOnLED(offLed);
	OSA_TimeDelay(200); /* 200ms */

	GPIO_DRV_SetPinOutput(nordicSwdioNrst.pinName);

	/* Turning LEDs back on to its original state.
	 * 0 means LED is on. */
	if (!green) {
		GPIO_DRV_ClearPinOutput(ledGreenPin.pinName);
	}
	if (!blue) {
		GPIO_DRV_ClearPinOutput(ledBluePin.pinName);
	}
	if (!red) {
		GPIO_DRV_ClearPinOutput(ledRedPin.pinName);
	}
}

/*
 * Calling this will turn off and turn back on to whatever initial color was.
 */
void blinkLED() {
	uint32_t green, blue, red;

	green = GPIO_DRV_ReadPinInput(ledGreenPin.pinName);
	blue = GPIO_DRV_ReadPinInput(ledBluePin.pinName);
	red = GPIO_DRV_ReadPinInput(ledRedPin.pinName);

	turnOnLED(offLed);

	/* 0 means LED is on */
	if (!green) {
		GPIO_DRV_ClearPinOutput(ledGreenPin.pinName);
	}
	if (!blue) {
		GPIO_DRV_ClearPinOutput(ledBluePin.pinName);
	}
	if (!red) {
		GPIO_DRV_ClearPinOutput(ledRedPin.pinName);
	}
}
/* Upon unrecoverable error, we call this function and
 * it will blink the LED.
 *
 * WARNING: This function does NOT return!!!
 * It's an infinite loop!!
 */
void blinkLEDerror(sat_led_t color, uint32_t multiplier) {
	 debug_printf("Unrecoverable error happened!");

	while (1) {
		turnOnLED(color);
		OSA_TimeDelay(300 * multiplier);
		turnOnLED(offLed);
		OSA_TimeDelay(300 * multiplier);
	}
}

void onlyTest(bool isCommand){
	OSA_TimeDelay(500);
	if(isCommand)
		turnOnLED(redLed);
	else
		turnOnLED(yellowLed);

	OSA_TimeDelay(500);
	turnOnLED(offLed);
}

void testCommand0(sat_mosi_packet_t packet){ //use command=0
	turnOnLED(turqoiseLed);
	OSA_TimeDelay(500);
	onlyTest(packet.sat_command[SAT_COMMSTRUCT_VALX1] == 1);
	onlyTest(packet.sat_command[SAT_COMMSTRUCT_VALY2] == 2);
	onlyTest(packet.sat_command[SAT_COMMSTRUCT_VALZ3] == 3);
}

void testCommand1(sat_mosi_packet_t packet){ //use command=1
	turnOnLED(purpleLed);
	OSA_TimeDelay(500);
	onlyTest(packet.sat_command[SAT_COMMSTRUCT_VALX1] == 1);
	onlyTest(packet.sat_command[SAT_COMMSTRUCT_VALY2] == 1);
	onlyTest(packet.sat_command[SAT_COMMSTRUCT_VALZ3] == 1);
}

int WAgiveIntoFromUint(uint32_t value){ //use command=1
	int result;
	uint32_t negativeCK=1000000000;

	if(value >= negativeCK)
		result = -((int)((value-negativeCK)));
	else
		result = (int)value;

	return result;
}


/*******************************************************************************
 * Process Hub command coming from Nordic's SPI
 ******************************************************************************/
void processCommand(sat_mosi_packet_t packet) {
//	uint16_t recv_crc16, crc16;
//	static uint32_t failed_crc16 = 0; /* counting failed crc */
//	recv_crc16 = packet.crc16;
//	packet.crc16 = 0;
//	crc16 = crc16_compute((uint8_t*) &packet, sizeof(packet), 0);
//	if (crc16 != recv_crc16) {
//		failed_crc16++;
//		return;
//	}

	uint32_t cmd = packet.sat_command[SAT_COMMSTRUCT_SATID];

	if (cmd & SAT_SPI_SET_RTC) {
		if (packet.sat_command[SAT_COMMSTRUCT_VALX1] == 0) {
			/* We're starting a new recording, let's purge our cbuf.
			 * Resetting head, tail, etc. */
			cbufInit(&sensorRecordQ);
		}
		/* By default we're setting it to 0 */
		//	OSA_TimeInit();
		k22f_set_rtc(packet.sat_command[SAT_COMMSTRUCT_VALX1]);
	}

	if (cmd & SAT_SPI_START) {
		sensor_active = true;
	} else {
		if(sensor_active)
			simpleReset(false,thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2]); //CF
		sensor_active = false;
	}

	if (cmd & SAT_SPI_WAIT) {
		sensor_sending = false;
	} else {
		sensor_sending = true;
	}

	if (cmd & SAT_SPI_SET_COM) {
		int cmd_sat = packet.sat_command[SAT_COMMSTRUCT_COMMA];
		//from < 0 to 9 RESERVED COMMANDS
		if(cmd_sat==SAT_COMM_TESTVALUE0)//test command 0
		{
			testCommand0(packet);
		}else if(cmd_sat==SAT_COMM_TESTVALUE1){//test command 1
			testCommand1(packet);
		}else if(cmd_sat==SAT_COMM_MBIASSENDG){//command 10 - MAG BIAS SENDING MODE
			onlyTest(false);
			simpleReset(true,0,0,0);
			globals.disable_magCalibrationAlways = false;
		}else if(cmd_sat==SAT_COMM_NORMLSENDG){//command 11 - NORMAL SENDING MODE
			onlyTest(false);
			simpleReset(false,thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2]);
		}else if(cmd_sat==SAT_COMM_SETMAGBIAS){//command 12 - NORMAL SENDING MODE SET BIAS
			onlyTest(false);
			simpleReset(false,
					((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALX1]))/1000.0F,
					((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALY2]))/1000.0F,
					((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALZ3]))/1000.0F); //CF

			float valX=((float)packet.sat_command[SAT_COMMSTRUCT_VALX1])/1000.0F;
			onlyTest( valX == 12.345F); //TEST TEST TEST

		}else if(cmd_sat==SAT_COMM_MSTBIASENDG){//command 13 - MAG BIAS SETTED SENDING MODE
			onlyTest(false);
			simpleReset(true,thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2]);
			globals.disable_magCalibrationAlways = true;
		}else
			onlyTest(true); //red led means NOT IMPLEMENTED YET
		}


	if (sensor_active == false) {
		turnOnLED(greenLed);
	} else {
		if (sensor_sending == true) {
			turnOnLED(turqoiseLed);
		} else {
			turnOnLED(redLed);
		}
	}

}

///*******************************************************************************
// * SPI slave callback function to handle incoming Hub command from Nordic
// ******************************************************************************/
//dspi_status_t from_nordic_data_sink(uint8_t sinkWord, uint32_t instance) {
//	/* Sometimes Nordic awakens faster than 22F, and it already tries to
//	 * set sensor active while 22F hasn't completely finished HW init.
//	 * Must wait.
//	 */
//	if (!hw_init_done) {
//		return kStatus_DSPI_Success;
//	}
//	/* In the case of unrecoverable error, do nothing */
//	if (hub_err_status != hub_no_err) {
//		return kStatus_DSPI_Success;
//	}
//
//	static int currSinkByte = 0;
//	static unsigned int pktsyncByte = 0xF9;
//	if (currSinkByte < sizeof(receivedCommand.bytes)) {
//		receivedCommand.bytes[currSinkByte] = sinkWord;
//	}
//	// Check if we're detecting packet-sync-byte sequence [FAFBFCFDFEFF]
//	if (sinkWord >= 0xFA && sinkWord == pktsyncByte + 1) {
//		pktsyncByte = sinkWord;
//	} else {
//		pktsyncByte = 0xF9;
//	}
//	currSinkByte++;
//	if (currSinkByte == SPI_COMM_LENGTH) {
//		if ((pktsyncByte == 0xF9) || (pktsyncByte == 0xFF)) {
//			// Proper packet, not in the middle of packet-sync-byte sequence
//			processCommand(receivedCommand.packet);
//			pktsyncByte = 0xF9;
//		}
//		currSinkByte = 0;
//	} else if (pktsyncByte == 0xFF) {
//		// Master only sends [FAFBFCFDFEFF] at the end of a packet. Must have gotten out of sync.
//		debug_printf("Resync from %d\r\n", currSinkByte);
//		pktsyncByte = 0xF9;
//		currSinkByte = 0;
//		currSourceByte = SPI_COMM_LENGTH;
//
//		/* Power cycle Nordic */
//		nrf51822_swdio_reset();
//
//		if (nrf51822_init() != kStatus_DSPI_Success)
//	    {
//			debug_printf("\r\n ERROR: Can not initialize DSPI slave driver \r\n");
//			blinkLEDerror(redLed, 4);
//	    };
//	}
//	return kStatus_DSPI_Success;
//}
//
///*******************************************************************************
// * SPI slave callback function to send SPI data upon receiving msg from Nordic
// *******************************************************************************/
//dspi_status_t to_nordic_data_source(uint8_t *sourceWord, uint32_t instance) {
//	sat_cbuf_packet_t cbuf_data;
//	err_t errCode = E_NO_MEM;
//
//	/* Sometimes Nordic awakens faster than 22F, and it already tries to
//	 * set sensor active while 22F hasn't completely finished HW init.
//	 * Must wait.
//	 */
//	/* Also, in the case of unrecoverable error, do nothing */
//	if ((!hw_init_done) || (hub_err_status != hub_no_err)) {
//		dataToSend_t invalidDataToSend;
//
//		invalidDataToSend.data.timestamp = INVALID_TIMESTAMP;
//		invalidDataToSend.data.sat_id = SAT_ID;
//		(*sourceWord) = invalidDataToSend.bytes[0];
//
//		return kStatus_DSPI_Success;
//	}
//
//	if (currSourceByte == SPI_COMM_LENGTH) {
//		errCode = E_NO_MEM;
//		errCode = cbufPop(&sensorRecordQ, &cbuf_data);
//		if (errCode != E_OK) {
//			; //empty cbuf
//		} else {
//			dataToSend.data.data = cbuf_data.data;
//			dataToSend.data.timestamp = cbuf_data.timestamp;
//		}
//
//		// If no record could be popped, or if we've been told to not send
//		// anything, send previous sensor data with invalid timestamp.
//		if (errCode != E_OK) {
//			// sensorRecordQ is empty, send invalid timestamp
//			dataToSend.data.timestamp = INVALID_TIMESTAMP;
//		}
//		currSourceByte = 0;
//#if DEBUG
//		debug_printf("sending record @%d (x%X) [%f[%X] / %f[%X],%f[%X],%f[%X]]\r\n",
//				dataToSend.data.timestamp,
//				dataToSend.data.timestamp,
//				dataToSend.data.data.quat_w,
//				*(uint32_t*)(&dataToSend.data.data.quat_w),
//				dataToSend.data.data.quat_x,
//				*(uint32_t*)(&dataToSend.data.data.quat_x),
//				dataToSend.data.data.quat_y,
//				*(uint32_t*)(&dataToSend.data.data.quat_y),
//				dataToSend.data.data.quat_z,
//				*(uint32_t*)(&dataToSend.data.data.quat_z));
//#endif
//	}
//	if (currSourceByte < sizeof(dataToSend.bytes)) {
//		(*sourceWord) = dataToSend.bytes[currSourceByte];
//	} else {
//		(*sourceWord) = 0;
//	}
//	currSourceByte++;
//
//	return kStatus_DSPI_Success;
//}
//
///*******************************************************************************
// * SPI slave callback function to handle error
// *******************************************************************************/
//void on_error(dspi_status_t error, uint32_t instance) {
//	/* Perform error handling in here. */
//	debug_printf("on_error(): error in SPI slave comm\r\n");
//}

dspi_status_t talk_to_nordic() {
	dspi_status_t dspiResult;
	sat_cbuf_packet_t cbuf_data;
	err_t errCode;

	/* Prepare buffer to send */
	errCode = cbufPop(&sensorRecordQ, &cbuf_data);
	if (errCode != E_OK) {
		// sensorRecordQ is empty, send invalid timestamp
		dataToSend.data.timestamp = INVALID_TIMESTAMP;
	} else {
		dataToSend.data.data = cbuf_data.data;
		dataToSend.data.timestamp = cbuf_data.timestamp;
	}

	/* Setting CRC16 */
	dataToSend.data.crc16 = 0;
	dataToSend.data.crc16 = crc16_compute((uint8_t*)&dataToSend.data, sizeof (dataToSend.data), 0);

	/* Send SPI Slave comm for the driver to initiate.  Note that only Master can really
	 * initiate communication.  So we'll wait until it finishes.
	 */
    dspiResult = DSPI_DRV_SlaveTransfer(knRF51822SpiInstance,
                                        &dataToSend.bytes[0],
										&spiReceiveBuffer.bytes[0],
										SPI_COMM_LENGTH);
    return dspiResult;
}

typedef struct spi_tx_ {
	uint32_t numtx[2];
	uint16_t crc16;
} spi_tx_t;
typedef struct spi_rx_ {
	uint32_t numrx[2];
	uint16_t crc16;
} spi_rx_t;

spi_tx_t spi_tx;
spi_rx_t spi_rx;

dspi_status_t recv_from_nordic_diag() {
	dspi_status_t dspiResult;

	/* Send SPI Slave comm for the driver to initiate.  Note that only Master can really
	 * initiate communication.  So we'll wait until it finishes.
	 */
    dspiResult = DSPI_DRV_SlaveTransfer(knRF51822SpiInstance,
                                        NULL,
										(uint8_t*)&spi_rx,
										sizeof(spi_rx));
    return dspiResult;
}
dspi_status_t send_to_nordic_diag() {
	dspi_status_t dspiResult;
//	sat_cbuf_packet_t cbuf_data;
//	err_t errCode;
	uint16_t crc16;
	static uint32_t cnt = 0;

	spi_tx.numtx[0] = cnt;
	spi_tx.numtx[1] = cnt++;
	spi_tx.crc16 = 0;
	crc16 = crc16_compute ((uint8_t*)&spi_tx, sizeof(spi_tx), 0);
	spi_tx.crc16 = crc16;

	/* Send SPI Slave comm for the driver to initiate.  Note that only Master can really
	 * initiate communication.  So we'll wait until it finishes.
	 */
    dspiResult = DSPI_DRV_SlaveTransfer(knRF51822SpiInstance,
                                        (uint8_t*)&spi_tx,
										NULL,
										sizeof(spi_rx));
    return dspiResult;
}
static uint32_t spi_bad_rx = 0, spi_bad_crc = 0, spi_good_crc = 0;
static uint32_t spi_rx_cnt = 0;
void processCommand_diag() {
	uint16_t recv_crc16, crc16;

	if (spi_rx.numrx != spi_rx_cnt) {
		spi_bad_rx++;
		spi_rx_cnt = spi_rx.numrx[0];
	}
	spi_rx_cnt++;

	recv_crc16 = spi_rx.crc16;
	spi_rx.crc16 = 0;
	crc16 = crc16_compute((uint8_t*) &spi_rx, sizeof(spi_rx),0);
	if (crc16 != recv_crc16) {
		spi_bad_crc++;
	} else {
		spi_good_crc++;
	}
}
/*******************************************************************************
 * Initialize Nordic nRF51822
 ******************************************************************************/
dspi_status_t nrf51822_init()
{
	dspi_status_t dspiResult;

	// Setting up K22F as SPI Slave of Nordic nRF51822
	static const dspi_slave_user_config_t nrf51822_dspiSlaveUserConfig =
	{
		.dataConfig.clkPhase = kDspiClockPhase_FirstEdge,
		.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh,
		.dataConfig.bitsPerFrame = 8,
		.dummyPattern = 0,
	};
//
//	configure_spi_pins(knRF51822SpiInstance);

	dspiResult = DSPI_DRV_SlaveDeinit(knRF51822SpiInstance);
	dspiResult = DSPI_DRV_SlaveInit(knRF51822SpiInstance, &nrf51822_dspiSlaveState,
			&nrf51822_dspiSlaveUserConfig);

	return dspiResult;
}


#if OUTPUT_3D_CUBE
/*******************************************************************************
 * Output float number in char
 ******************************************************************************/
static void put_float(float f)
{
	int32_t q;

	q = (int32_t) (f * (1<<30));  // Python demo uses fixed point +-1.30 bits

	putchar((q >> 24) & 0xff);
	putchar((q >> 16) & 0xff);
	putchar((q >> 8)  & 0xff);
	putchar(q & 0xff);
}

/*******************************************************************************
 * Display Quarternion numbers
 ******************************************************************************/
static void output_quaternion_packet(void)
{
#if 1
	printf("$1,%f,%f,%f,%f;\r\n", thisSV_9DOF_GBY_KALMAN.fqPl.q0,
			thisSV_9DOF_GBY_KALMAN.fqPl.q1,
			thisSV_9DOF_GBY_KALMAN.fqPl.q2,
			thisSV_9DOF_GBY_KALMAN.fqPl.q3);
#else
	putchar('$');
	putchar(0x02);
	putchar(0x00);
	put_float(thisSV_9DOF_GBY_KALMAN.fqPl.q0);
	put_float(thisSV_9DOF_GBY_KALMAN.fqPl.q1);
	put_float(thisSV_9DOF_GBY_KALMAN.fqPl.q2);
	put_float(thisSV_9DOF_GBY_KALMAN.fqPl.q3);
	putchar(0x00);
	putchar(0x00);
	putchar('\r');
	putchar('\n');
#endif /* 1 */
}

void main_loop_output_3d_cube(void)
{
	sensor_data_t 		raw_data;				// Raw output of MPU9250 (accel, gyro, mag)
	sat_cbuf_packet_t 	sensor_record;	// Timestamp + sensor data (raw + sensor-fusion output)

	while (true)
	{
		if (! get_raw_data_sample(&raw_data, &sensor_record.data))
			continue;
		output_quaternion_packet();
	}
}
#endif /* OUTPUT_3D_CUBE */
/*******************************************************************************
 * Read MPU9250 raw output (Accelerometer, Gyrometer, Magnetometer, Temperature)
 ******************************************************************************/
static bool get_raw_data_sample_ts(sensor_data_t *raw_data, sensor_record_t *data, uint32_t *timestamp) {
	bool ret;
	;

	while (!mpu9250_data_avail())
		;
	mpu9250_clear_data_avail();
	ret = mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP,
			raw_data);

	if (ret) {
		process_sensor_fusion(raw_data, data);
		*timestamp = raw_data->timestamp;
	}

	return ret;
}

/*******************************************************************************
 * Read MPU9250 raw output (Accelerometer, Gyrometer, Magnetometer, Temperature)
 ******************************************************************************/
static bool get_raw_data_sample(sensor_data_t *raw_data, sensor_record_t *data) {
	bool ret;
	;

	while (!mpu9250_data_avail())
		;
	mpu9250_clear_data_avail();
	ret = mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP,
			raw_data);

	if (ret) {
		process_sensor_fusion(raw_data, data);
	}

	return ret;
}

//#endif

static void sensorFusionInit() {
	/* Initializing Turing Sense sensor fusion */
	ts_fusion_init();
	mpu9250_start();

}

static void simpleReset(bool magBiasSending, float magBiasX, float magBiasY, float magBiasZ) {
	sensorFusionInit();
	globals.enable_MagBiasSending = magBiasSending;
	globals.enable_MagBiasCalibrationAlways = SAT_BIAS_ALWAYSACTIVECALIBRATION;

	thisMagCal.fV[0] = magBiasX; //(float)SAT_MAGBIAS_X;
	thisMagCal.fV[1] = magBiasY; //(float)SAT_MAGBIAS_Y;
	thisMagCal.fV[2] = magBiasZ; //(float)SAT_MAGBIAS_Z;
}

extern uint32_t g_xtal0ClkFreq;

static bool cwati_tmp_check() {
	for (uint8_t qq = 0; qq < SPI_COMM_LENGTH; qq++ ) {
		if (spiReceiveBuffer.bytes[qq] != qq) {
			return false;
		}
	}
	return true;
}

static uint32_t period_spi_nordic[200] = {0};
static uint32_t cnt_spi_nordic = 0, ave_spi_nordic = 0, total_spi_nordic = 0, reinit_nordic = 0;
static uint32_t cwati_spi_fail = 0;

uint32_t tmp_nordic;
/*******************************************************************************
 * Main routine
 ******************************************************************************/
int main(void) {
	sensor_data_t raw_data;			// Raw output of MPU9250 (accel, gyro, mag)
	sat_cbuf_packet_t sensor_record;// Timestamp + sensor data (raw + sensor-fusion output)
	uint32_t current_ts10, time_now;
	bool is_sleeping = false, retbool;
    dspi_status_t dspiResult;
    uint32_t wordsTransfer = 0;

#if DO_SELFTEST
	mpu9250_self_test_results_t factory_st;
	mpu9250_average_data_t avg_normal, avg_selftest;
#endif /* DO_SELFTEST */

	g_xtal0ClkFreq = 8000000U;

	VirtualCom_Deinit();

	// Initialize K22F
	hardware_init();
	if (OSA_Init() != kStatus_OSA_Success) {
		debug_printf("error initializing OSA\r\n");
		blinkLEDerror(redLed, 1);
		hub_err_status = hub_hw_err1;
	}
	configure_uart_pins(BOARD_DEBUG_UART_INSTANCE);
	dbg_uart_init();

//	// Initialize sensor record queue
//	if (cbufInit(&sensorRecordQ) != E_OK) {
//		debug_printf("error initializing sensorRecordQ\r\n");
//		blinkLEDerror(redLed, 2);
//		hub_err_status = hub_hw_err2;
//	}

	// Initialize R/G/B LED pins
	// Set RED_LED_GPIO (4)
	ledRedPin.config.outputLogic = 0;
	ledRedPin.config.slewRate = kPortSlowSlewRate;
	ledRedPin.config.driveStrength = kPortHighDriveStrength;
	ledRedPin.pinName = GPIO_MAKE_PIN(GPIOA_IDX, RED_LED_GPIO);
	GPIO_DRV_OutputPinInit(&ledRedPin);

	// Set GREEN_LED_GPIO (5)
	ledGreenPin.config.outputLogic = 0;
	ledGreenPin.config.slewRate = kPortSlowSlewRate;
	ledGreenPin.config.driveStrength = kPortHighDriveStrength;
	ledGreenPin.pinName = GPIO_MAKE_PIN(GPIOA_IDX, GREEN_LED_GPIO);
	GPIO_DRV_OutputPinInit(&ledGreenPin);
	// Set BLUE_LED_GPIO (12)
	ledBluePin.config.outputLogic = 0;
	ledBluePin.config.slewRate = kPortSlowSlewRate;
	ledBluePin.config.driveStrength = kPortHighDriveStrength;
	ledBluePin.pinName = GPIO_MAKE_PIN(GPIOA_IDX, BLUE_LED_GPIO);
	GPIO_DRV_OutputPinInit(&ledBluePin);

#if PRODUCTION1
	// Set NORDIC soft reset (PTB19)
	nordicSwdioNrst.config.outputLogic = 1;
	nordicSwdioNrst.config.slewRate = kPortSlowSlewRate;
	nordicSwdioNrst.config.driveStrength = kPortHighDriveStrength;
	nordicSwdioNrst.pinName = GPIO_MAKE_PIN(GPIOB_IDX, NORDIC_SWDIO_NRST);
	GPIO_DRV_OutputPinInit(&nordicSwdioNrst);
#endif

#if OUTPUT_3D_CUBE
	turnOnLED(blueLed);
#endif /* OUTPUT_3D_CUBE */
#if DEBUG
	long int i;
	float test;
	for(i=0; i<10; i++)
		debug_printf("HELLO TS CDC, %d \n\r", i);
	for(i=0; i<10; i++){
		test += 101.25;
		debug_printf("FLOAT VALUE = %f \n\r", test);
	}
#endif /* DEBUG */

//// Testing k22f RTC
//	debug_printf("time now: %u", k22f_get_rtc());
//	k22f_set_rtc(65533);
//	debug_printf("time now: %u", k22f_get_rtc());
//	OSA_TimeDelay(100);
//	debug_printf("time now: %u", k22f_get_rtc());
//
//
//	k22f_set_rtc(0);
//	debug_printf("time now: %u", k22f_get_rtc());
//	OSA_TimeDelay(100);
//	debug_printf("time now: %u", k22f_get_rtc());
//
//
//	// Initialize MPU9250 (configure SPI1 master, calibrate, init regs, data structures, etc.)
//	debug_printf("\r\nInitializing Invensense MPU-9250\r\n");
//	debug_printf(
//			"\r\n(Freescale's Open Source Sensor Fusion is used instead of Invensense's Motion Driver 6.1)\r\n");
//	if (!mpu9250_init()) {
//		debug_printf("error initializing MPU9250\r\n");
//		blinkLEDerror(redLed, 3);
//		hub_err_status = hub_hw_err3;
//	}
//
//#if DO_SELFTEST
//	// We'd like to test both the accel and gyro, but as of 2015-07-23, there
//	// is a problem with taking gyro self-test data which results in the
//	// data being out of range, as if the accelerometer is clipping at
//	// full-scale range.
//	// NOTE: also or in  ST_FLAG_DEBUG_PRINTF to diagnose self-test failures
//	int st_flags = ST_FLAG_GYRO | ST_FLAG_ACCEL;
//	OSA_TimeDelay(100);
//
//	if (!mpu9250_get_factory_self_test_results(&factory_st)) {
////		blinkLEDerror(yellowLed, 1);
//		blinkLEDerror(greenLed, 1);
//		hub_err_status = hub_inv_err1;
//	}
//
//	if (st_flags & ST_FLAG_DEBUG_PRINTF) {
//		for (int axis = X; axis <= Z; axis++)
//			debug_printf("factory self-test gyro %c:  %d\r\n", 'X' + axis,
//					factory_st.gyro[axis]);
//		for (int axis = X; axis <= Z; axis++)
//			debug_printf("factory self-test accel %c: %d\r\n", 'X' + axis,
//					factory_st.accel[axis]);
//	}
//
//	debug_printf("starting self-test\r\n");
//
//	if (!mpu9250_self_test(st_flags, &avg_normal, &avg_selftest)) {
//		debug_printf("error collecting self-test data samples\r\n");
//		blinkLEDerror(yellowLed, 2);
//		hub_err_status = hub_inv_err2;
//	}
//	debug_printf("self-test completed\r\n");
//
//	if ((st_flags & ST_FLAG_DEBUG_PRINTF) && (st_flags & ST_FLAG_ACCEL)) {
//		for (int axis = X; axis <= Z; axis++)
//			debug_printf("normal accel %c average:  %d\r\n", 'X' + axis,
//					avg_normal.accel[axis]);
//		for (int axis = X; axis <= Z; axis++)
//			debug_printf("self-test accel %c average:  %d\r\n", 'X' + axis,
//					avg_selftest.accel[axis]);
//		for (int axis = X; axis <= Z; axis++)
//			debug_printf("self-test accel %c response:  %d\r\n", 'X' + axis,
//					avg_selftest.accel[axis] - avg_normal.accel[axis]);
//	}
//
//	if ((st_flags & ST_FLAG_DEBUG_PRINTF) && (st_flags & ST_FLAG_GYRO)) {
//		for (int axis = X; axis <= Z; axis++)
//			debug_printf("normal gyro %c average:   %d\r\n", 'X' + axis,
//					avg_normal.gyro[axis]);
//		for (int axis = X; axis <= Z; axis++)
//			debug_printf("self-test gyro %c average:   %d\r\n", 'X' + axis,
//					avg_selftest.gyro[axis]);
//		for (int axis = X; axis <= Z; axis++)
//			debug_printf("self-test gyro %c response:   %d\r\n", 'X' + axis,
//					avg_selftest.gyro[axis] - avg_normal.gyro[axis]);
//	}
//
//	uint32_t status = 0;  // assume OK
//	OSA_TimeDelay(100);
//
//	status = mpu9250_check_selftest_results(st_flags, &factory_st, &avg_normal,
//			&avg_selftest);
//	if (status) {
//		debug_printf("self-test results out of tolerance FAILED: %u\r\n", status);
//		blinkLEDerror(purpleLed, 3);
//		hub_err_status = hub_inv_err3;
//	}
//	debug_printf("self-test results good\r\n");
//#endif // DO_SELFTEST
//
//
//	simpleReset(SAT_BIAS_SENDING,SAT_MAGBIAS_X,SAT_MAGBIAS_Y,SAT_MAGBIAS_Z);
////
////	/* Reset RTC to 0 */
////	k22f_set_rtc(0);
//
//	/* Initializing satellite id */
//	dataToSend.data.sat_id = SAT_ID;
//
//	hw_init_done = true;
//
//	// Initialize nRF51822 (configure SPI0 slave, data structures, etc.)
//	debug_printf("\r\nInitializing Nordic nRF51822\r\n");
	turnOnLED(offLed);
	nrf51822_swdio_reset();

	dspiResult = nrf51822_init();
	if (dspiResult != kStatus_DSPI_Success)
    {
		debug_printf("\r\n ERROR: Can not initialize DSPI slave driver \r\n");
		blinkLEDerror(redLed, 4);
    };

	// Done with all initializations, turn on GREEN LED, entering Main loop
	turnOnLED(greenLed);
//	debug_printf("GOT HERE \n\r");

	// Main loop
#if OUTPUT_3D_CUBE
	main_loop_output_3d_cube();
#else

	bool alternateSending = true;
	last_nordic_comm = k22f_get_rtc();

	while(1) {

		dspiResult = recv_from_nordic_diag();
        if (dspiResult != kStatus_DSPI_Success)
        {
    		debug_printf("\r\n ERROR: Can not recv SPI from to Nordic \r\n");
    		blinkLEDerror(redLed, 4);
        };


		/* Check SPI response */
		while (DSPI_DRV_SlaveGetTransferStatus(knRF51822SpiInstance, &wordsTransfer) == kStatus_DSPI_Busy) {}

		/* TODO use error check here to check validity of SPI packet received. */
		processCommand_diag();

		dspiResult = send_to_nordic_diag();
        if (dspiResult != kStatus_DSPI_Success)
        {
    		debug_printf("\r\n ERROR: Can not send SPI to Nordic \r\n");
    		blinkLEDerror(redLed, 5);
        };

		/* Check SPI response */
		while (DSPI_DRV_SlaveGetTransferStatus(knRF51822SpiInstance, &wordsTransfer) == kStatus_DSPI_Busy) {}

	}

	while (true) {

		dspiResult = talk_to_nordic();
        if (dspiResult != kStatus_DSPI_Success)
        {
    		debug_printf("\r\n ERROR: Can not start SPI to Nordic \r\n");
    		//cwati this is where we reset SPI & soft reset Nordic?
    		blinkLEDerror(redLed, 4);
        };

        while (1) {
        	/* Talk to sensor */
			if (sensor_active == true) {
				retbool = get_raw_data_sample_ts(&raw_data, &sensor_record.data, &sensor_record.timestamp);

				if (retbool) {
#if PACE_SENSOR_OUTPUT
					if (alternateSending) {
						/* We want to discard older data */
						if (cbufIsFull(&sensorRecordQ)) {
							cbufPopDiscard(&sensorRecordQ);
						}

						if (cbufPush(&sensorRecordQ, sensor_record) != E_OK) {
							debug_printf("error inserting to sensorRecordQ\r\n");
							blinkLEDerror(yellowLed, 3);
						}
					}

					alternateSending = !alternateSending;
#else
					/* We want to discard older data */
					if (cbufIsFull(&sensorRecordQ)) {
						cbufPopDiscard(&sensorRecordQ);
					}

					if (cbufPush(&sensorRecordQ, sensor_record) != E_OK) {
						debug_printf("error inserting to sensorRecordQ\r\n");
						blinkLEDerror(yellowLed, 3);
					}
#endif /* PACE_SENSOR_OUTPUT */
				}
			} else {
				OSA_TimeDelay(5); /* ms */
			}

			/* Check SPI response */
			if (DSPI_DRV_SlaveGetTransferStatus(knRF51822SpiInstance, &wordsTransfer) == kStatus_DSPI_Success) {
				/* TODO use error check here to check validity of SPI packet received. */
				processCommand(spiReceiveBuffer.packet);

//				if (!cwati_tmp_check()) {
//					cwati_spi_fail++;
//				}
//				tmp_nordic = last_nordic_comm;
//
//				last_nordic_comm = k22f_get_rtc();
//				cw3 = k22f_get_rtc();
//
//				tmp_nordic = last_nordic_comm - tmp_nordic;
//				period_spi_nordic[cnt_spi_nordic % 200] = tmp_nordic;
//				total_spi_nordic += tmp_nordic;
//				ave_spi_nordic = total_spi_nordic / cnt_spi_nordic;
//				cnt_spi_nordic++;

				break;
			} else {
				if (k22f_get_rtc() - last_nordic_comm > max_nordic_quiet) {
					/* Power cycle Nordic */
					nrf51822_swdio_reset();
					if (nrf51822_init() != kStatus_DSPI_Success)
					{
						debug_printf("\r\n ERROR: Can not initialize DSPI slave driver \r\n");
						blinkLEDerror(redLed, 4);
					};
					last_nordic_comm = k22f_get_rtc();
					reinit_nordic++;
					break;
				}
			}
        } /* End of while(1), waiting for Nordic to talk */
	}
#endif /* OUTPUT_3D_CUBE */
	return 0;  // will never reach this

} // end main()

/**
 * HardFaultHandler_C:
 * This is called from the HardFault_HandlerAsm with a pointer the Fault stack
 * as the parameter. We can then read the values from the stack and place them
 * into local variables for ease of reading.
 * We then read the various Fault Status and Address Registers to help decode
 * cause of the fault.
 * The function ends with a BKPT instruction to force control back into the debugger
 */
void GenericFault_HandlerC(unsigned long *hardfault_args){
  volatile unsigned long stacked_r0 ;
  volatile unsigned long stacked_r1 ;
  volatile unsigned long stacked_r2 ;
  volatile unsigned long stacked_r3 ;
  volatile unsigned long stacked_r12 ;
  volatile unsigned long stacked_lr ;
  volatile unsigned long stacked_pc ;
  volatile unsigned long stacked_psr ;
  volatile unsigned long _CFSR ;
  volatile unsigned long _HFSR ;
  volatile unsigned long _DFSR ;
  volatile unsigned long _AFSR ;
  volatile unsigned long _BFAR ;
  volatile unsigned long _MMAR ;

  stacked_r0 = ((unsigned long)hardfault_args[0]) ;
  stacked_r1 = ((unsigned long)hardfault_args[1]) ;
  stacked_r2 = ((unsigned long)hardfault_args[2]) ;
  stacked_r3 = ((unsigned long)hardfault_args[3]) ;
  stacked_r12 = ((unsigned long)hardfault_args[4]) ;
  stacked_lr = ((unsigned long)hardfault_args[5]) ;
  stacked_pc = ((unsigned long)hardfault_args[6]) ;
  stacked_psr = ((unsigned long)hardfault_args[7]) ;

  // Configurable Fault Status Register
  // Consists of MMSR, BFSR and UFSR
  _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;

  // Hard Fault Status Register
  _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

  // Debug Fault Status Register
  _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

  // Auxiliary Fault Status Register
  _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

  // Read the Fault Address Registers. These may not contain valid values.
  // Check BFARVALID/MMARVALID to see if they are valid values
  // MemManage Fault Address Register
  _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
  // Bus Fault Address Register
  _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

  __asm("BKPT #0\n") ; // Break into the debugger
}

__attribute__((naked))
void Default_HandlerC(void)
{
  __asm volatile (
    " movs r0,#4       \n"
    " movs r1, lr      \n"
    " tst r0, r1       \n"
    " beq _MSP         \n"
    " mrs r0, psp      \n"
    " b _HALT          \n"
  "_MSP:               \n"
    " mrs r0, msp      \n"
  "_HALT:              \n"
    " ldr r1,[r0,#20]  \n"
    " b GenericFault_HandlerC \n"
    " bkpt #0          \n"
  );
}

