/*
 * Copyright (c) 2015 TuringSense
 *
 * diag_22f_sensor.c
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
#include <stdbool.h>
#include <stdio.h>
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

#if PROTO_BOARD
#include "MK22F12810.h"			/* For RTC registers */
#else
#if PRODUCTION1
#include "MK22F25612.h"			/* For RTC registers */
#endif
#endif
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
#include "main_22f_sensor.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEBUG			0
#define OUTPUT_3D_CUBE	0
#define SAT_ID			60	/* TODO later will read from memory region */

#define	DIAG1			1 /* This diag is for tracking the life of a sensor record.
 	 	 	 	 	 	 	 Although generating real sensor data, the data transmitted will be overwritten
 	 	 	 	 	 	 	 with a simple counter data. */
#if DIAG1
#define DIAG1_STORE_TS_DIFF_SIZE		100
#define DIAG1_TS_THRESHOLD				30
static uint32_t			diag1_big_ts[DIAG1_STORE_TS_DIFF_SIZE] = {0}; /* Timestamp diff bigger than DIAG1_TS_THRESHOLD */
static uint32_t			diag1_big_ts_loc[DIAG1_STORE_TS_DIFF_SIZE] = {0}; /* Tracking at which point biggest timestamp diffs happen */
static uint32_t			diag1_accelx_ctr = 0; /* Simple counter to replace the accel_x data */
static uint32_t			diag1_ave_ts = 0, diag1_max_ts = 0, diag1_min_ts = UINT32_MAX;
static uint32_t			diag1_lost_data = 0;  /* How much data doesn't make it to Nordic */
static uint32_t			diag1_pop_ctr = 0;

/* Measuring periodic time, how long between each pop.
 */
static uint32_t			diag1_last_pop_ms = 0;
static uint32_t			diag1_ave_pop_period = 0;
static uint32_t			diag1_max_pop_period = 0;
static uint32_t			diag1_min_pop_period = UINT32_MAX;
#endif

#if PACE_SENSOR_OUTPUT
uint32_t 				pace_timestamp = 8;			/* ms todo, make this a function of MPU9250_SENSOR_FREQ.
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 10ms is too much, we'll be losing data. */
uint32_t 				start_rtc;

#endif
hub_err_t				hub_err_status = hub_no_err;

uint32_t				reset_rtc_diff = 0;	/* Elapsed time before resetting RTC happens.*/

static volatile bool	hw_init_done = false;

//debug
uint32_t ts1, ts2, ts3;

static uint32_t 		last_ts10 = 0;
static uint32_t			last_sec, last_tpr;
static uint32_t			last_total_ms;	//debug cw todo

//#define SPI_INSTANCE	1	// Proto Sensor Module: Invensense MPU9250 is SPI 1 */
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
extern uint32_t g_dspiBaseAddr[];
static dspi_slave_state_t nrf51822_dspiSlaveState;
static cbuf_t sensorRecordQ={0};  // Queue of sensor data records (see sat_module.h and cbuf.h)
// Power up defaults: don't collect sensor data, but if told to start
// collecting, send it right away.
static volatile bool sensor_active = false;
static volatile bool sensor_sending = false;

static union {
	sat_mosi_packet_t packet;
	uint8_t bytes[/*sizeof(sat_mosi_packet_t)*/SPI_COMM_LENGTH];
} receivedCommand;
typedef union dataToSend_{
	sat_miso_packet_t data;
	uint8_t bytes[sizeof(sat_miso_packet_t)];
} dataToSend_t;

dataToSend_t dataToSend;

static uint32_t timeAtBootInMillis = 0;
static int currSourceByte = 0;  // SPI data byte position for SPI slave
static gpio_output_pin_user_config_t ledRedPin;    // Red LED pin
static gpio_output_pin_user_config_t ledGreenPin;  // Green LED pin
static gpio_output_pin_user_config_t ledBluePin;   // Blue LED pin
#if PRODUCTION1
static gpio_output_pin_user_config_t nordicSwdioNrst;
#endif
void nrf51822_init();
uint32_t k22f_get_rtc();
static void sensorFusionInit();

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
 *
 ******************************************************************************/
void nrf51822_swdio_reset() {
	GPIO_DRV_ClearPinOutput(nordicSwdioNrst.pinName);
	OSA_TimeDelay(150);	/* 150ms */
	GPIO_DRV_SetPinOutput(nordicSwdioNrst.pinName);
}

/* Upon unrecoverable error, we call this function and
 * it will blink the LED.
 *
 * WARNING: This function does NOT return!!!
 * It's an infinite loop!!
 */
void blinkLEDerror(sat_led_t color, uint32_t multiplier) {
	printf("Unrecoverable error happened!");

	while (1) {
		turnOnLED(color);
		OSA_TimeDelay(300*multiplier);
		turnOnLED(offLed);
		OSA_TimeDelay(300*multiplier);
	}
}

#if PACE_SENSOR_OUTPUT
void k22f_set_elapsed_rtc() {
	start_rtc = k22f_get_rtc();
}

uint32_t k22f_get_elapsed_rtc() {
	uint32_t elapsed_rtc;

	/* Get value */
	ts1 = k22f_get_rtc();
	elapsed_rtc = ts1 - start_rtc + reset_rtc_diff;

	reset_rtc_diff = 0;
	return (elapsed_rtc);
}
#endif

/*******************************************************************************
 * Enable K22F's RTC registers (TSR - seconds counter, TPR - prescaler counter
 * TSR is incremented every second
 * TPR is incremented every 32.768 kHz
 ******************************************************************************/
void k22f_enable_rtc() {
	// Enable 32.768 kHz oscillator
	BW_RTC_CR_OSCE(RTC_BASE, 1);
	// Wait 100ms for oscillator output to settle down
	OSA_TimeDelay(100);
	// Disable time counters TSR and TPR before writing
	BW_RTC_SR_TCE(RTC_BASE, 0);
	// Clear the RTC_TPR counter
	HW_RTC_TPR_CLR(RTC_BASE, 0xFFFFFFFFU);
	// Write the RTC_TSR with 0 second value
	HW_RTC_TSR_WR(RTC_BASE, 0);
	// Enable time counters TSR and TPR after writing
	BW_RTC_SR_TCE(RTC_BASE, 1);

	last_sec = 0;
	last_tpr = 0;
	last_total_ms = 0;
}

/*******************************************************************************
 * Set K22F's RTC to reset any drift among satellite modules
 ******************************************************************************/
void k22f_set_rtc(uint32_t hub_rtc_milliseconds) {
#if PACE_SENSOR_OUTPUT
	uint32_t rtc_diff, last_rtc;
	last_rtc = k22f_get_rtc();

	/* Elapsed time so far */
	reset_rtc_diff = last_rtc - start_rtc;
	/* Resetting start_rtc to time now */
	start_rtc = hub_rtc_milliseconds;
#endif

	uint32_t rtc_milliseconds = hub_rtc_milliseconds % 1000;
	last_sec = (hub_rtc_milliseconds - rtc_milliseconds) / 1000;
	// TPR = 1 equals to ~30.5us = 1000000us / 32768Hz = (30.5/1000)ms = (1000/32768)ms
	last_tpr = (uint32_t)((float)(rtc_milliseconds*32768) / 1000);
	last_total_ms = (last_sec * 1000) + rtc_milliseconds;

	// Disable time counters TSR and TPR before writing
	BW_RTC_SR_TCE(RTC_BASE, 0);
	// Write the RTC_TPR counter first
	HW_RTC_TPR_WR(RTC_BASE, last_tpr);
	// Write the RTC_TSR with Hub's RTC's seconds value
	HW_RTC_TSR_WR(RTC_BASE, last_sec);
	// Re-Enable time counters TSR and TPR after writing
	BW_RTC_SR_TCE(RTC_BASE, 1);
}

/*******************************************************************************
 * Get K22F's RTC in milliseconds
 ******************************************************************************/
uint32_t k22f_get_rtc(void) {


	uint32_t 			dummy_tpr, dummy_tpr1, current_tpr;
	uint32_t 			rtc_milliseconds;
	uint32_t 			dummy_rtc, current_sec;
	uint32_t			cnt;
	const uint32_t		cnt_max = 0xFF;

	//debug

	uint32_t total_ms_diff, current_total_ms;

	/* WARNING: There will be consequences if you change the order of the reading!
	 * Make sure you know what you're doing.
	 *
	 * For example:
	 * At time 0 second, 0x7FFF
	 * Read TPR: get 0x7FFF
	 * Read second: get 1 (already incremented)
	 * So you need to fix it.
	 *
	 * If you read second first
	 * second: get 0,
	 * TPR: get 0 (already incremented).
	 * Must handle these cases.
	 */
	/* We have to read twice or more until we get same values */
	dummy_tpr = HW_RTC_TPR_RD(RTC_BASE);
	current_tpr = HW_RTC_TPR_RD(RTC_BASE);
	cnt = 0;
	while (dummy_tpr != current_tpr) {
		if (++cnt == cnt_max) {
			/* Put a count to avoid infinite loop.  Shouldn't arrive here unless HW
			 * goes kaput. */
			return UINT32_MAX;
		}
		dummy_tpr = HW_RTC_TPR_RD(RTC_BASE);
		current_tpr = HW_RTC_TPR_RD(RTC_BASE);
	}

	/* We have to read twice or more until we get same values */
	dummy_rtc = HW_RTC_TSR_RD(RTC_BASE);
	current_sec = HW_RTC_TSR_RD(RTC_BASE);
	cnt = 0;
	while (dummy_rtc != current_sec) {
		if (++cnt == cnt_max) {
			/* Put a count to avoid infinite loop.  Shouldn't arrive here unless HW
			 * goes kaput. */
			return UINT32_MAX;
		}
		dummy_rtc = HW_RTC_TSR_RD(RTC_BASE);
		current_sec = HW_RTC_TSR_RD(RTC_BASE);
	}

	/* Check again.
	 * The hardware is bad.
	 *
	 * On our first read, say time was
	 * 0/7FFF
	 *
	 * It is possible that the above read yields:
	 * 7FFF, then 0 (which is correct)
	 * or
	 * 7FFF, then 1  --> when second has incremented.
	 *
	 */
	dummy_tpr = HW_RTC_TPR_RD(RTC_BASE);
	dummy_tpr1 = HW_RTC_TPR_RD(RTC_BASE);
	while (dummy_tpr != dummy_tpr1) {
		if (++cnt == cnt_max) {
			/* Put a count to avoid infinite loop.  Shouldn't arrive here unless HW
			 * goes kaput. */
			return UINT32_MAX;
		}
		dummy_tpr = HW_RTC_TPR_RD(RTC_BASE);
		dummy_tpr1 = HW_RTC_TPR_RD(RTC_BASE);
	}

	if (dummy_tpr < current_tpr) {
		current_sec = current_sec - 1;
	}

	// TPR = 1 equals to ~30.5us = 1000000us / 32768Hz = (30.5/1000)ms = (1000/32768)ms
	rtc_milliseconds = current_tpr * 1000 / 32768;
	current_total_ms = (current_sec * 1000) + rtc_milliseconds;

	/* In the case that timer has reached max (max is the same with INVALID_TIMESTAMP,
	 * then use current_total_ms calculation.
	 */
	if ((current_total_ms < last_total_ms) && (last_total_ms != INVALID_TIMESTAMP)) {
		/* Very bad.  HW problem.  Not much we could do.
		 * Let's just return last value. */
		return last_total_ms;
	} else {
		last_sec = current_sec;
		last_tpr = current_tpr;
		last_total_ms = current_total_ms;

		return current_total_ms;
	}
// DEBUG
//	static count = 1;
//	static uint32_t prevtimeMillisecond = 0;
//	uint32_t timeMillisecond = current_sec*1000 + rtc_milliseconds;
//	if ((count % 10) == 0) {
//		count = 1;
//	} else {
//		count++;
//	}
//	prevtimeMillisecond = timeMillisecond;
//	return timeMillisecond;

}

/*******************************************************************************
 * Process Hub command coming from Nordic's SPI
 ******************************************************************************/
void processCommand(sat_mosi_packet_t packet) {

	uint32_t cmd = packet.command;

	if (cmd & SAT_SPI_START) {
		sensor_active = true;
	} else {
		sensor_active = false;
	}

	if (cmd & SAT_SPI_WAIT) {
		sensor_sending = false;
	} else {
		sensor_sending = true;
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

	if (cmd & SAT_SPI_SET_RTC) {
		/* By default we're setting it to 0 */
		k22f_set_rtc(0);
		/* We're starting a new recording, let's purge our cbuf.
		 * Resetting head, tail, etc. */
		cbufInit(&sensorRecordQ);
#if DIAG1
		diag1_accelx_ctr = 0;
#endif
	}
}

/*******************************************************************************
 * SPI slave callback function to handle incoming Hub command from Nordic
 ******************************************************************************/
dspi_status_t from_nordic_data_sink(uint8_t sinkWord, uint32_t instance)
{
	/* Sometimes Nordic awakens faster than 22F, and it already tries to
	 * set sensor active while 22F hasn't completely finished HW init.
	 * Must wait.
	 */
	if (!hw_init_done) {
		return kStatus_DSPI_Success;
	}
	/* In the case of unrecoverable error, do nothing */
	if (hub_err_status != hub_no_err) {
		return kStatus_DSPI_Success;
	}

	static int currSinkByte = 0;
	static unsigned int pktsyncByte = 0xF9;
	if(currSinkByte < sizeof(receivedCommand.bytes)) {
		receivedCommand.bytes[currSinkByte] = sinkWord;
	}
	// Check if we're detecting packet-sync-byte sequence [FAFBFCFDFEFF]
	if(sinkWord >= 0xFA && sinkWord == pktsyncByte+1) {
		pktsyncByte = sinkWord;
	} else {
		pktsyncByte = 0xF9;
	}
	currSinkByte++;
	if (currSinkByte == SPI_COMM_LENGTH) {
		if ((pktsyncByte == 0xF9) || (pktsyncByte == 0xFF)) {
			// Proper packet, not in the middle of packet-sync-byte sequence
			processCommand(receivedCommand.packet);
			pktsyncByte = 0xF9;
		}
		currSinkByte = 0;
	} else if (pktsyncByte == 0xFF) {
		// Master only sends [FAFBFCFDFEFF] at the end of a packet. Must have gotten out of sync.
		printf("Resync from %d\r\n",currSinkByte);
		pktsyncByte = 0xF9;
		currSinkByte = 0;
		currSourceByte = SPI_COMM_LENGTH;
		nrf51822_init();
	}
	return kStatus_DSPI_Success;
}

/*******************************************************************************
 * SPI slave callback function to send SPI data upon receiving msg from Nordic
 *******************************************************************************/
dspi_status_t to_nordic_data_source(uint8_t *sourceWord, uint32_t instance)
{
	sat_cbuf_packet_t cbuf_data;
	/* Sometimes Nordic awakens faster than 22F, and it already tries to
	 * set sensor active while 22F hasn't completely finished HW init.
	 * Must wait.
	 */
	/* Also, in the case of unrecoverable error, do nothing */
	if ((!hw_init_done) || (hub_err_status != hub_no_err)) {
		dataToSend_t invalidDataToSend;

		invalidDataToSend.data.timestamp = INVALID_TIMESTAMP;
		invalidDataToSend.data.sat_id = SAT_ID;
		(*sourceWord) = invalidDataToSend.bytes[0];

		return kStatus_DSPI_Success;
	}


	if (currSourceByte == SPI_COMM_LENGTH) {
		err_t errCode = E_NO_MEM;
		if (sensor_sending == true) {
			errCode = cbufPop(&sensorRecordQ, &cbuf_data);
#if DIAG1
			if (errCode == E_OK) {
				/* Expecting data to be starting from 0, and increasing */
				if (cbuf_data.data.accel_x == diag1_pop_ctr) {
					diag1_pop_ctr++;
				} else {
					/* Count how many data are lost */
					if (cbuf_data.data.accel_x > diag1_pop_ctr) {
						diag1_lost_data += cbuf_data.data.accel_x - diag1_pop_ctr;
					} else {
						/* We only have 16-bit counter, it's possible it has wrapped around */
						diag1_lost_data += (cbuf_data.data.accel_x) + (UINT16_MAX - diag1_lost_data);
					}
					diag1_pop_ctr = cbuf_data.data.accel_x;
				}

				uint32_t time_now = k22f_get_rtc();
				if (diag1_last_pop_ms != 0) {
					/* WARNING: This can wrap around after 1193 hours */
					uint32_t tmp = time_now - diag1_last_pop_ms;
					if (diag1_max_pop_period < tmp) {
						diag1_max_pop_period = tmp;
					}
					if (diag1_min_pop_period > tmp) {
						diag1_min_pop_period = tmp;
					}
					if (diag1_ave_pop_period == 0) {
						diag1_ave_pop_period = tmp;
					} else {
						diag1_ave_pop_period = (diag1_ave_pop_period + tmp) / 2;
					}

				}
				diag1_last_pop_ms = time_now;
				uint32_t time_diff;
				static uint32_t diag1_max_tsdiff_cnt = 0;

				time_diff = time_now - cbuf_data.timestamp;

				if (time_diff >= DIAG1_TS_THRESHOLD) {
					diag1_big_ts[diag1_max_tsdiff_cnt % DIAG1_STORE_TS_DIFF_SIZE] = time_diff;
					diag1_big_ts_loc[diag1_max_tsdiff_cnt % DIAG1_STORE_TS_DIFF_SIZE] = diag1_pop_ctr;
					diag1_max_tsdiff_cnt++;
				}
				if (diag1_max_ts < time_diff) {
					diag1_max_ts = time_diff;
				}
				if (diag1_min_ts > time_diff) {
					diag1_min_ts = time_diff;
				}
				if (diag1_ave_ts == 0) {
					diag1_ave_ts = time_diff;
				} else {
					diag1_ave_ts = (diag1_ave_ts + time_diff) / 2;
				}

#endif /* DIAG1 */
				dataToSend.data.data = cbuf_data.data;
				dataToSend.data.timestamp = cbuf_data.timestamp;
			}
		}
		// If no record could be popped, or if we've been told to not send
		// anything, send previous sensor data with incremented timestamp.
		if (errCode != E_OK) {
			// sensorRecordQ is empty, send invalid timestamp
			dataToSend.data.timestamp = INVALID_TIMESTAMP;
		}
		currSourceByte = 0;
#if DEBUG
		printf("sending record @%d (x%X) [%f[%X] / %f[%X],%f[%X],%f[%X]]\r\n",
				dataToSend.data.timestamp,
				dataToSend.data.timestamp,
				dataToSend.data.data.quat_w,
				*(uint32_t*)(&dataToSend.data.data.quat_w),
				dataToSend.data.data.quat_x,
				*(uint32_t*)(&dataToSend.data.data.quat_x),
				dataToSend.data.data.quat_y,
				*(uint32_t*)(&dataToSend.data.data.quat_y),
				dataToSend.data.data.quat_z,
				*(uint32_t*)(&dataToSend.data.data.quat_z));
#endif
	}
	if (currSourceByte < sizeof(dataToSend.bytes)) {
		(*sourceWord) = dataToSend.bytes[currSourceByte];
	} else {
		(*sourceWord) = 0;
	}
	currSourceByte++;

    return kStatus_DSPI_Success;
}

/*******************************************************************************
 * SPI slave callback function to handle error
 *******************************************************************************/
void on_error(dspi_status_t error, uint32_t instance)
{
    /* Perform error handling in here. */
   	printf("on_error(): error in SPI slave comm\r\n");
}

/*******************************************************************************
 * Initialize Nordic nRF51822
 ******************************************************************************/
void nrf51822_init()
{
	// Setting up K22F as SPI Slave of Nordic nRF51822
	static const dspi_slave_user_config_t nrf51822_dspiSlaveUserConfig =
	{
		.callbacks.dataSink = from_nordic_data_sink,
		.callbacks.dataSource = to_nordic_data_source,
		.callbacks.onError = on_error,
		.dataConfig.clkPhase = kDspiClockPhase_FirstEdge,
		.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh,
		.dataConfig.bitsPerFrame = 8,
	};

	configure_spi_pins(knRF51822SpiInstance);

    DSPI_DRV_SlaveInit(knRF51822SpiInstance, &nrf51822_dspiSlaveState, &nrf51822_dspiSlaveUserConfig);
}

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

#if OUTPUT_3D_CUBE
/*******************************************************************************
 * Display Quarternion numbers
 ******************************************************************************/
static void output_quaternion_packet(void)
{
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
}
#endif /* OUTPUT_3D_CUBE */

#if PACE_SENSOR_OUTPUT
/*******************************************************************************
 * Read MPU9250 raw output (Accelerometer, Gyrometer, Magnetometer, Temperature)
 * Will return only after "time_ms" has elapsed.
 * Otherwise it will wait until we get data before we return.
 *
 * Never return false!! Always return true!!
 * Might loop forever if invensense never gives data!
 ******************************************************************************/
static bool get_raw_data_sample_w_freq (sensor_data_t *raw_data, uint32_t time_ms,
		sensor_record_t* data)
{
	uint8_t			got_data = false;
	bool			ret;

	k22f_set_elapsed_rtc();

	ts3 = k22f_get_elapsed_rtc();//debug todo
	while (ts3 < time_ms) {
		if (mpu9250_data_avail()) {

			/* Clear it first to avoid race condition */
			mpu9250_clear_data_avail();
			// Raw output of MPU9250 (accel, gyro, mag)
			ret = mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP, raw_data);
			if (ret) {
				got_data = true;
				process_sensor_fusion(raw_data, data);
			}
		}

		ts3 = k22f_get_elapsed_rtc();//debug todo
	}

	if (ts3 >= time_ms) {
		if (got_data) {
			return true;
		} else {
			while (! mpu9250_data_avail())
				;
			/* Clear it first to avoid race condition */
			mpu9250_clear_data_avail();
			// Raw output of MPU9250 (accel, gyro, mag)
			ret = mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP, raw_data);
			if (ret) {
				got_data = true;
				process_sensor_fusion(raw_data, data);
			}

			return true;
		}
	}

	return true;
}
#else
/*******************************************************************************
 * Read MPU9250 raw output (Accelerometer, Gyrometer, Magnetometer, Temperature)
 ******************************************************************************/
static bool get_raw_data_sample(sensor_data_t *raw_data, sensor_record_t *data)
{
	bool ret;;

	while (! mpu9250_data_avail())
		;
	mpu9250_clear_data_avail();
	ret = mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP, raw_data);

	if(ret) {
		process_sensor_fusion(raw_data, data);
	}

	return ret;
}


#endif

static void sensorFusionInit() {
	/* Initializing Turing Sense sensor fusion */
	ts_fusion_init();
	mpu9250_start();

}

/*******************************************************************************
 * Main routine
 ******************************************************************************/
int main(void)
{
	sensor_data_t 		raw_data;				// Raw output of MPU9250 (accel, gyro, mag)
	sat_cbuf_packet_t 	sensor_record;	// Timestamp + sensor data (raw + sensor-fusion output)
	uint32_t			current_ts10;
#if DO_SELFTEST
	mpu9250_self_test_results_t factory_st;
	mpu9250_average_data_t avg_normal, avg_selftest;
#endif /* DO_SELFTEST */

	// Initialize K22F
	hardware_init();
	if (OSA_Init() != kStatus_OSA_Success) {
    	printf("error initializing OSA\r\n");
    	blinkLEDerror(redLed, 1);
    	hub_err_status = hub_hw_err1;
	}
    configure_uart_pins(BOARD_DEBUG_UART_INSTANCE);
    dbg_uart_init();
    k22f_enable_rtc();

    // Initialize sensor record queue
    if (cbufInit(&sensorRecordQ) != E_OK) {
    	printf("error initializing sensorRecordQ\r\n");
    	blinkLEDerror(redLed, 2);
    	hub_err_status = hub_hw_err2;
    }

    // Initialize R/G/B LED pins
	// Set RED_LED_GPIO (4)
	ledRedPin.config.outputLogic = 0;
	ledRedPin.config.slewRate = kPortSlowSlewRate;
	ledRedPin.config.driveStrength = kPortHighDriveStrength;
	ledRedPin.pinName = GPIO_MAKE_PIN(HW_GPIOA, RED_LED_GPIO);
	GPIO_DRV_OutputPinInit(&ledRedPin);

    // Set GREEN_LED_GPIO (5)
	ledGreenPin.config.outputLogic = 0;
	ledGreenPin.config.slewRate = kPortSlowSlewRate;
	ledGreenPin.config.driveStrength = kPortHighDriveStrength;
	ledGreenPin.pinName = GPIO_MAKE_PIN(HW_GPIOA, GREEN_LED_GPIO);
	GPIO_DRV_OutputPinInit(&ledGreenPin);
    // Set BLUE_LED_GPIO (12)
	ledBluePin.config.outputLogic = 0;
	ledBluePin.config.slewRate = kPortSlowSlewRate;
	ledBluePin.config.driveStrength = kPortHighDriveStrength;
	ledBluePin.pinName = GPIO_MAKE_PIN(HW_GPIOA, BLUE_LED_GPIO);
	GPIO_DRV_OutputPinInit(&ledBluePin);

#if PRODUCTION1
    // Set NORDIC soft reset (PTB19)
	nordicSwdioNrst.config.outputLogic = 1;
	nordicSwdioNrst.config.slewRate = kPortSlowSlewRate;
	nordicSwdioNrst.config.driveStrength = kPortHighDriveStrength;
	nordicSwdioNrst.pinName = GPIO_MAKE_PIN(HW_GPIOB, NORDIC_SWDIO_NRST);
	GPIO_DRV_OutputPinInit(&nordicSwdioNrst);
#endif

    // Initialize MPU9250 (configure SPI1 master, calibrate, init regs, data structures, etc.)
    printf("\r\nInitializing Invensense MPU-9250\r\n");
    printf("\r\n(Freescale's Open Source Sensor Fusion is used instead of Invensense's Motion Driver 6.1)\r\n");
	if (! mpu9250_init()) {
		printf("error initializing MPU9250\r\n");
    	blinkLEDerror(redLed, 3);
    	hub_err_status = hub_hw_err3;
	}


#if DO_SELFTEST
	// We'd like to test both the accel and gyro, but as of 2015-07-23, there
	// is a problem with taking gyro self-test data which results in the
	// data being out of range, as if the accelerometer is clipping at
	// full-scale range.
	// NOTE: also or in  ST_FLAG_DEBUG_PRINTF to diagnose self-test failures
	int st_flags = ST_FLAG_GYRO | ST_FLAG_ACCEL;
	OSA_TimeDelay(100);

	if (! mpu9250_get_factory_self_test_results(& factory_st))
	{
//		blinkLEDerror(yellowLed, 1);
		blinkLEDerror(greenLed, 1);
    	hub_err_status = hub_inv_err1;
	}

	if (st_flags & ST_FLAG_DEBUG_PRINTF)
	{
		for (int axis = X; axis <= Z; axis++)
			printf("factory self-test gyro %c:  %d\r\n", 'X' + axis, factory_st.gyro[axis]);
		for (int axis = X; axis <= Z; axis++)
			printf("factory self-test accel %c: %d\r\n", 'X' + axis, factory_st.accel[axis]);
	}

	printf("starting self-test\r\n");

	if (! mpu9250_self_test(st_flags, & avg_normal, & avg_selftest))
	{
		printf("error collecting self-test data samples\r\n");
		blinkLEDerror(yellowLed, 2);
    	hub_err_status = hub_inv_err2;
	}
	printf("self-test completed\r\n");

	if ((st_flags & ST_FLAG_DEBUG_PRINTF) && (st_flags & ST_FLAG_ACCEL))
	{
		for (int axis = X; axis <= Z; axis++)
			printf("normal accel %c average:  %d\r\n", 'X' + axis, avg_normal.accel[axis]);
		for (int axis = X; axis <= Z; axis++)
			printf("self-test accel %c average:  %d\r\n", 'X' + axis, avg_selftest.accel[axis]);
		for (int axis = X; axis <= Z; axis++)
			printf("self-test accel %c response:  %d\r\n", 'X' + axis, avg_selftest.accel[axis] - avg_normal.accel[axis]);
	}

	if ((st_flags & ST_FLAG_DEBUG_PRINTF) && (st_flags & ST_FLAG_GYRO))
	{
		for (int axis = X; axis <= Z; axis++)
			printf("normal gyro %c average:   %d\r\n", 'X' + axis, avg_normal.gyro[axis]);
		for (int axis = X; axis <= Z; axis++)
			printf("self-test gyro %c average:   %d\r\n", 'X' + axis, avg_selftest.gyro[axis]);
		for (int axis = X; axis <= Z; axis++)
			printf("self-test gyro %c response:   %d\r\n", 'X' + axis, avg_selftest.gyro[axis] - avg_normal.gyro[axis]);
	}

	uint32_t status = 0;  // assume OK
	OSA_TimeDelay(100);

	status = mpu9250_check_selftest_results(st_flags, & factory_st, & avg_normal, & avg_selftest);
	if (status)
	{
		printf("self-test results out of tolerance FAILED: %u\r\n", status);
		blinkLEDerror(purpleLed, 3);
    	hub_err_status = hub_inv_err3;
	}
	printf("self-test results good\r\n");
#endif // DO_SELFTEST

	sensorFusionInit();

	/* Initializing satellite id */
	dataToSend.data.sat_id = SAT_ID;

	hw_init_done = true;

    // Initialize nRF51822 (configure SPI0 slave, data structures, etc.)
    printf("\r\nInitializing Nordic nRF51822\r\n");
	turnOnLED(offLed);
    nrf51822_swdio_reset();

	nrf51822_init();

	// Done with all initializations, turn on GREEN LED, entering Main loop
	turnOnLED(greenLed);

	// Main loop
	while (true)
	{
		if (sensor_active == false) {
			// Put MPU9250 to sleep by disabling the accel, gyro, and clock
			mpu9250_sleep();
			while (sensor_active == false) {
				// Wait for Event.  TODO: Put K22F in low power mode
			}
			// Out of sleep mode, wake up MPU9250
			mpu9250_wakeup();
			cbufInit(&sensorRecordQ);  // Reset sensorRecordQ to avoid stale records
		}

#if OUTPUT_3D_CUBE
		if (TRUE) {
#else
		if (cbufIsFull(&sensorRecordQ) == FALSE) {
#endif

#if PACE_SENSOR_OUTPUT
			// Read MPU9250's raw sensor data (accel, gyro, mag)
			if (! get_raw_data_sample_w_freq(&raw_data, pace_timestamp, &sensor_record.data))
			{
				continue;  // go to top of while loop
			}
#else
			if (! get_raw_data_sample(&raw_data, &sensor_record.data))
			{
				continue;  // go to top of while loop
			}
#endif /* PACE_SENSOR_OUTPUT */

#if OUTPUT_3D_CUBE
		    output_quaternion_packet();
#else
		    // Push the sensor data record to the queue to send to Nordic nRF51822
		    sensor_record.timestamp = k22f_get_rtc();

#if PACE_SENSOR_OUTPUT

		    current_ts10 = (sensor_record.timestamp / 10) * 10;

		    /* Only push 1 record per 10ms. */
		    if ((last_ts10 != 0) && (current_ts10 != last_ts10)) {
#if DIAG1
		    	sensor_record.data.accel_x = diag1_accelx_ctr++;
#endif
				if (cbufPush(&sensorRecordQ, sensor_record) != E_OK) {
					printf("error inserting to sensorRecordQ\r\n");
					while (true)
						;
				}
		    }

		    last_ts10 = current_ts10;
#else
		    cbufPush(&sensorRecordQ, sensor_record);
#endif /* PACE_SENSOR_OUTPUT */
		}

#endif /* OUTPUT_3D_CUBE */
	}

	return 0;  // will never reach this

} // end main()



