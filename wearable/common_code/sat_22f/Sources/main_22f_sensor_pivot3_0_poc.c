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
#include "mpu9250_pivot3_0_poc.h"
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
#include "crc16.h"
#include "main_22f_sensor.h"
#include "adc_ts.h"

// New SF
#include "KF.h"
#include "matrixComputations.h"
#include "orientationComputation.h"

#if USE_FLASH
/* TODO eventually using bootloader will be the default method. */
#include "ts_flash.h"
#endif /* USE_BOOTLOADER */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEBUG			0
#define OUTPUT_3D_CUBE	0
#define SATID 321
uint32_t sat_id = SATID;

/*******************************************************************************
 * BIAS DETECTION AND BIAS SETTING *** NOTE! VALID FOR SINGOL MEMS
 ******************************************************************************/
#define SAT_BIAS_SENDING 	false
#define SAT_MAGBIAS_X 	 0
#define SAT_MAGBIAS_Y    0
#define SAT_MAGBIAS_Z    0
float adcValue;

#define Z_DOWN 	0
#define Z_UP   	1
#define Y_DOWN 	2
#define Y_UP 	3
#define X_DOWN 	4
#define X_UP	5

#define SAT_BIAS_ALWAYSACTIVECALIBRATION 	false //Even if set the fixed values of BIAS (with SAT_BIAS_SENDING=false), calibration of the magnetometer remains active. In this case the calibration values will be recalculated

#define USE_HWE_DEBUG_CONFIGURATION 1
static bool read_sensors_launch_fusion (sensor_data_t *raw_data, sensor_record_t* data);

static bool get_raw_data_sample(uint8_t, sensor_data_t *raw_data, sensor_record_t *data);
static bool get_raw_data_sample_basic(sensor_data_t *raw_data);
static int counterDataSample[MAX_SENSORS_IN_CS] = {0};
static sat_cbuf_packet_t sensorDataForInterpolation = { 0 };

#if PACE_SENSOR_OUTPUT
static uint8_t alternateSending = TS_INITIAL_SENDING_PERIOD,
		alternateSending_PERIOD = TS_INITIAL_SENDING_PERIOD;
#endif
hub_err_t hub_err_status = hub_no_err;

uint32_t reset_rtc_diff = 0; /* Elapsed time before resetting RTC happens.*/

static volatile bool hw_init_done = false;
int communicationTestCounterInitialization = 10;

// New SF
#define STDG 0.5*PI/180
#define STDA 0.02/9.81

#define CA 0.1
#define CB 0.01
#define PO 0.001

#define XDIM 6
#define YDIM 3


static bool estimateGyroBias_run = false;
static bool estimateAccBias_run = false;
int16_t gyrBiasInt16_X[MAX_SENSORS_IN_CS] = {0};
int16_t gyrBiasInt16_Y[MAX_SENSORS_IN_CS] = {0};
int16_t gyrBiasInt16_Z[MAX_SENSORS_IN_CS] = {0};
int16_t accSensInt16_X[MAX_SENSORS_IN_CS]; //Initialized in main()
int16_t accSensInt16_Y[MAX_SENSORS_IN_CS]; //Initialized in main()
int16_t accSensInt16_Z[MAX_SENSORS_IN_CS]; //Initialized in main()
int16_t accBiasInt16_X[MAX_SENSORS_IN_CS] = {0};
int16_t accBiasInt16_Y[MAX_SENSORS_IN_CS] = {0};
int16_t accBiasInt16_Z[MAX_SENSORS_IN_CS] = {0};


uint32_t myHubID9250 = 0;
uint32_t myHubIDrcvd = 0;
uint32_t myHubID = 0;

uint16_t myLCP = 0;

// ts self test
static bool tsSelfTest_run = false;

//debug
uint32_t ts1, ts2, ts3;

static uint32_t last_ts10 = 0;
static uint32_t last_sec, last_tpr;
static uint32_t last_total_ms;
uint32_t ts[500] = { 0 }, ts_cnt = 0;

/* Heartbeat check for Nordic communication */
uint32_t last_nordic_comm;
uint32_t last_bat_read = 0;

#define MAX_NORDIC_QUIET	5000 /* in ms */

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
static cbuf_t sensorRecordQ = { 0 }; // Queue of sensor data records (see sat_module.h and cbuf.h)
// Power up defaults: don't collect sensor data, but if told to start
// collecting, send it right away.
static volatile bool start_dumping = false;

static union {
	sat_mosi_packet_t packet;
	uint8_t bytes[/*sizeof(sat_mosi_packet_t)*/SPI_COMM_LENGTH];
} receivedCommand;
typedef union dataToSend_ {
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

/* PIVOT 3.0 POC
 * sensor_num is the sequence of working sensor, and
 * it is not always the same with chip select (cs) number.
 *
 * For example we can have up to 4 sensors with sensor_num 0 to 3.
 *
 * Sensor_num 0 is on cs0
 * Sensor_num 1 is on cs2
 * Sensor_num 2 is on cs3
 * Sensor_num 3 is on cs5
 *
 * As you can see, the cs is not in order.
 *
 * valid_cs[sensor_num] will give you the cs number for each sensor.
 * */
uint8_t valid_num_sensors_in_cs = 0;
uint8_t valid_cs[MAX_SENSORS_IN_CS] = {0};

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
		GPIO_DRV_ClearPinOutput(ledGreenPin.pinName);
		GPIO_DRV_ClearPinOutput(ledBluePin.pinName);
		GPIO_DRV_SetPinOutput(ledRedPin.pinName);
		break;
	case whiteLed:
	default:
		GPIO_DRV_ClearPinOutput(ledGreenPin.pinName);
		GPIO_DRV_ClearPinOutput(ledBluePin.pinName);
		GPIO_DRV_ClearPinOutput(ledRedPin.pinName);
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

	OSA_TimeDelay(NORDIC_BL_WAIT_MS + NORDIC_BL_WAIT2_MS); /* Wait for Nordic to get out of BootLoader mode */

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
static void doubleBlinkLED(sat_led_t color) {
	uint32_t green, blue, red;

	green = GPIO_DRV_ReadPinInput(ledGreenPin.pinName);
	blue = GPIO_DRV_ReadPinInput(ledBluePin.pinName);
	red = GPIO_DRV_ReadPinInput(ledRedPin.pinName);

	turnOnLED(color);
	OSA_TimeDelay(100);
	turnOnLED(offLed);
	OSA_TimeDelay(100);
	turnOnLED(color);
	OSA_TimeDelay(100);

	/* 0 means LED is on */
	if (!green) {
		GPIO_DRV_ClearPinOutput(ledGreenPin.pinName);
	} else {
		GPIO_DRV_SetPinOutput(ledGreenPin.pinName);
	}
	if (!blue) {
		GPIO_DRV_ClearPinOutput(ledBluePin.pinName);
	} else {
		GPIO_DRV_SetPinOutput(ledBluePin.pinName);
	}
	if (!red) {
		GPIO_DRV_ClearPinOutput(ledRedPin.pinName);
	} else {
		GPIO_DRV_SetPinOutput(ledBluePin.pinName);
	}
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
		OSA_TimeDelay(300 * multiplier);
		turnOnLED(offLed);
		OSA_TimeDelay(300 * multiplier);
	}
}

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
	uint32_t rtc_milliseconds = hub_rtc_milliseconds % 1000;
	last_sec = (hub_rtc_milliseconds - rtc_milliseconds) / 1000;
	// TPR = 1 equals to ~30.5us = 1000000us / 32768Hz = (30.5/1000)ms = (1000/32768)ms
	last_tpr = (uint32_t) ((float) (rtc_milliseconds * 32768) / 1000);
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

	uint32_t dummy_tpr, dummy_tpr1, current_tpr;
	uint32_t rtc_milliseconds;
	uint32_t dummy_rtc, current_sec;
	uint32_t cnt;
	const uint32_t cnt_max = 0xFF;

	//debug

	uint32_t current_total_ms;

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
	if ((current_total_ms < last_total_ms)
			&& (last_total_ms != INVALID_TIMESTAMP)) {
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

void onlyTest(bool isCommand){
 //	OSA_TimeDelay(100);
	if(isCommand)
		turnOnLED(redLed);
	else
		turnOnLED(yellowLed);

	OSA_TimeDelay(100);
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


bool detectMotion(sensor_data_t * raw_data)
{
	float gyro[3];
	float squareNormGyro = 0.0f;

	for (int i = 0; i < 3; i++)
	{
		gyro[i] = ( (float) raw_data->gyro[i])*MPU9250_DEGPERSECPERCOUNT;
		squareNormGyro += gyro[i]*gyro[i];
	}

	if (squareNormGyro > 36)
		return true;
	else
		return false;
}

bool estimateGyroBias(uint8_t sensor_num)
{
	bool retboolG;
	int c = 0;
	int N = 500;

	sensor_data_t raw_data;

	bool ret;

	float coeff = MPU9250_DEGPERSECPERCOUNT/(float)N;
	float biasFloat_X = 0.0f;
	float biasFloat_Y = 0.0f;
	float biasFloat_Z = 0.0f;

	turnOnLED(turqoiseLed);

		while (c < N) {
			OSA_TimeDelay(5);

			get_raw_data_sample_basic(&raw_data);

			// compute the bias
			biasFloat_X += ( (float) raw_data.gyro[0])*coeff;
			biasFloat_Y += ( (float) raw_data.gyro[1])*coeff;
			biasFloat_Z += ( (float) raw_data.gyro[2])*coeff;

			c++;

			if (detectMotion(&raw_data))
			{
				turnOnLED(redLed);
				OSA_TimeDelay(1000);
				return false;
			}

		}

		gyrBiasInt16_X[sensor_num] = (int16_t) (biasFloat_X/MPU9250_DEGPERSECPERCOUNT);
		gyrBiasInt16_Y[sensor_num] = (int16_t) (biasFloat_Y/MPU9250_DEGPERSECPERCOUNT);
		gyrBiasInt16_Z[sensor_num] = (int16_t) (biasFloat_Z/MPU9250_DEGPERSECPERCOUNT);

	turnOnLED(offLed);
	return true;

}


void fInvertGyrCal(sensor_data_t * raw_data)
{
	raw_data->gyro[0] -= gyrBiasInt16_X[mpu9250_num];
	raw_data->gyro[1] -= gyrBiasInt16_Y[mpu9250_num];
	raw_data->gyro[2] -= gyrBiasInt16_Z[mpu9250_num];
}

void fInvertAccCal(sensor_data_t * raw_data)
{
	float tmpX = (float)(raw_data->accel[0] - accBiasInt16_X[mpu9250_num])*MPU9250_GPERCOUNT;
	float tmpY = (float)(raw_data->accel[1] - accBiasInt16_Y[mpu9250_num])*MPU9250_GPERCOUNT;
	float tmpZ = (float)(raw_data->accel[2] - accBiasInt16_Z[mpu9250_num])*MPU9250_GPERCOUNT;

	float Sx = (float)accSensInt16_X[mpu9250_num]*MPU9250_GPERCOUNT;
	float Sy = (float)accSensInt16_Y[mpu9250_num]*MPU9250_GPERCOUNT;
	float Sz = (float)accSensInt16_Z[mpu9250_num]*MPU9250_GPERCOUNT;

	tmpX *= Sx;
	tmpY *= Sy;
	tmpZ *= Sz;

	raw_data->accel[0] = (int16_t)(tmpX/MPU9250_GPERCOUNT);
	raw_data->accel[1] = (int16_t)(tmpY/MPU9250_GPERCOUNT);
	raw_data->accel[2] = (int16_t)(tmpZ/MPU9250_GPERCOUNT);


}

bool waitForNextPosition(int p)
{
	float M = 1.3;
	float m = 0.7;

	float accX, accY, accZ, accC;

	int count = 0;

	sensor_data_t raw_data;

	turnOnLED(purpleLed);

	while (count < 200)
	{
		if (!estimateAccBias_run)
			return false;

		count++;
		OSA_TimeDelay(5);
		get_raw_data_sample_basic(&raw_data);

		accX = ( (float) raw_data.accel[0])*MPU9250_GPERCOUNT;
		accY = ( (float) raw_data.accel[1])*MPU9250_GPERCOUNT;
		accZ = ( (float) raw_data.accel[2])*MPU9250_GPERCOUNT;

		switch(p) {

		   case Z_DOWN :
		      accC = -accZ;
		      break;
		   case Z_UP  :
		      accC = accZ;
		      break;
		   case Y_DOWN :
		      accC = -accY;
		      break;
		   case Y_UP  :
		      accC = accY;
		      break;
		   case X_DOWN :
		   	  accC = -accX;
		   	  break;
		   case X_UP  :
		   	  accC = accX;
		   	  break;
		}

		if ((accC > M)|(accC < m)|detectMotion(&raw_data))
			count = 0;

	}

	return true;

}


void computeJacobianAccBias(float data[6][6], float x[6], float J[6][6])
{
	int Nr = 6;
	int Nc = 6;

	for (int i = 0; i < Nr; i++)
	{
		J[i][0] = 2*x[0]*pow(data[0][i]-x[3],2);
		J[i][1] = 2*x[1]*pow(data[1][i]-x[4],2);
		J[i][2] = 2*x[2]*pow(data[2][i]-x[5],2);
		J[i][3] = 2*pow(x[0],2)*(x[3]-data[0][i]);
		J[i][4] = 2*pow(x[1],2)*(x[4]-data[1][i]);
		J[i][5] = 2*pow(x[2],2)*(x[5]-data[2][i]);
	}
}

void computeResidualsAccBias(float data[][6], float x[6], float beta[6])
{
	int Np = 6;
	for (int i = 0; i < Np; i++)
		beta[i] = pow(x[0],2)*pow(data[0][i]-x[3],2) + pow(x[1],2)*pow(data[1][i]-x[4],2) + pow(x[2],2)*pow(data[2][i]-x[5],2) - 1.0f;

}




float determinant(float matrix[6][6],float size)
{
    float s=1,det=0,m_minor[6][6];
    int i,j,m,n,c;
    if (size==1)
    {
        return (matrix[0][0]);
    }
    else
    {
        det=0;
        for (c=0;c<size;c++)
        {
            m=0;
            n=0;
            for (i=0;i<size;i++)
            {
                for (j=0;j<size;j++)
                {
                    m_minor[i][j]=0;
                    if (i != 0 && j != c)
                    {
                       m_minor[m][n]=matrix[i][j];
                       if (n<(size-2))
                          n++;
                       else
                       {
                           n=0;
                           m++;
                       }
                    }
                }
            }
            det=det + s * (matrix[0][c] * determinant(m_minor,size-1));
            s=-1 * s;
        }
    }

    return (det);
}


/*Finding transpose of cofactor of matrix*/
void transpose(float M[][6],float MT[][6])
{
    int N = 6;
     for (int i=0;i<N;i++)
        for (int j=0;j<N;j++)
            MT[i][j]=M[j][i];


 }


 /*calculate cofactor of matrix*/
void cofactor(float matrix[6][6],float Mcof[6][6])
{
    int size = 6;

     float m_cofactor[6][6];
     int p,q,m,n,i,j;
     for (q=0;q<size;q++)
     {
         for (p=0;p<size;p++)
         {
             m=0;
             n=0;
             for (i=0;i<size;i++)
             {
                 for (j=0;j<size;j++)
                 {
                     if (i != q && j != p)
                     {
                        m_cofactor[m][n]=matrix[i][j];
                        if (n<(size-2))
                           n++;
                        else
                        {
                            n=0;
                            m++;
                        }
                     }
                 }
             }
             Mcof[q][p]=pow(-1,q + p) * determinant(m_cofactor,size-1);
         }
     }

}


void invMatrixAccBias(float M[6][6], float iM[6][6])
{
    int N = 6;
    float d=determinant(M,N);

    float Mcof[6][6], McofT[6][6];

    cofactor(M,Mcof);

    transpose(Mcof,McofT);

    for (int i=0;i<N;i++)
        for (int j=0;j<N;j++)
             iM[i][j]=McofT[i][j]/d;


}

void matVectMult(float M[][6], float v[6], float d[6])
{
    int N = 6;

    for (int i = 0; i < N; i++)
    {
        d[i] = 0;
        for (int j = 0; j < N; j++)
            d[i] += M[i][j]*v[j];
    }

}



bool estimateAccBias(uint8_t sensor_num)
{

	bool retboolA;

	int N = 500;
	int Niter = 4;
	sensor_data_t raw_data;

	bool ret;

	float coeff = MPU9250_GPERCOUNT/(float)N;

	float data[3][6], J[6][6], iJ[6][6];
	float beta[6], deltaB[6];
	// Initial calib parameter vector - {sx,sy,sz,bx,by,bz};
	float x[6] = {1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f};

	bool stopCalAccCommand = false;

	// Acquire data matrix
	for (int i = 0; i < 6; i++)
	{
		stopCalAccCommand = !waitForNextPosition(i);

		turnOnLED(turqoiseLed);

		int c = 0;

		data[0][i] = 0.0f;
		data[1][i] = 0.0f;
		data[2][i] = 0.0f;

		while (c < N)
		{

			if (stopCalAccCommand || !estimateAccBias_run)
				return false;

			OSA_TimeDelay(5);

			get_raw_data_sample_basic(&raw_data);

			// acquire the data
			data[0][i] += ( (float) raw_data.accel[0])*coeff;
			data[1][i] += ( (float) raw_data.accel[1])*coeff;
			data[2][i] += ( (float) raw_data.accel[2])*coeff;

			c++;

			if (detectMotion(&raw_data))
			{
				turnOnLED(redLed);
				OSA_TimeDelay(1000);
				return false;
			}
		}
	}

	turnOnLED(yellowLed);

	// Computations
	for (int i = 0; i < Niter; i++)
	{
	    computeJacobianAccBias(data,x,J);
	    invMatrixAccBias(J,iJ);
	    computeResidualsAccBias(data, x, beta);

	    matVectMult(iJ,beta,deltaB);

	    for (int j = 0; j < 6; j++)
	       x[j] -=deltaB[j];
	 }

	turnOnLED(greenLed);

	// Write the result in the global variables
	accSensInt16_X[sensor_num] = (int16_t) (x[0]/MPU9250_GPERCOUNT);
	accSensInt16_Y[sensor_num] = (int16_t) (x[1]/MPU9250_GPERCOUNT);
	accSensInt16_Z[sensor_num] = (int16_t) (x[2]/MPU9250_GPERCOUNT);
	accBiasInt16_X[sensor_num] = (int16_t) (x[3]/MPU9250_GPERCOUNT);
	accBiasInt16_Y[sensor_num] = (int16_t) (x[4]/MPU9250_GPERCOUNT);
	accBiasInt16_Z[sensor_num] = (int16_t) (x[5]/MPU9250_GPERCOUNT);

		//turnOnLED(offLed);
		return true;
}


void computeAverageVector(float meanACC[3], float meanGYR[3], float meanMAG[3])
{
	int c = 0;
	int N = 200;
	int offset = 50;

	sensor_data_t raw_data;
	sensor_record_t data;

	bool ret;

	float coeffACC = MPU9250_GPERCOUNT/(float)N;
	float coeffGYR = MPU9250_DEGPERSECPERCOUNT/(float)N;
	float coeffMAG = MPU9250_UTPERCOUNT/(float)N;

	turnOnLED(turqoiseLed);

	while (c < N+offset)
	{
		OSA_TimeDelay(10);

		get_raw_data_sample_basic(&raw_data);

		if (c > offset)
		{
			for(int k = 0; k < 3; k++)
			{
				meanACC[k] += ( (float) raw_data.accel[k])*coeffACC;
				meanGYR[k] += ( (float) raw_data.gyro[k])*coeffGYR;
				meanMAG[k] += ( (float) raw_data.mag[k])*coeffMAG;
			}
		}

		c++;

	}

	turnOnLED(greenLed);
	OSA_TimeDelay(500);

}


void computeStdVector(float meanACC[3], float meanGYR[3], float meanMAG[3], float stdACC[3], float stdGYR[3], float stdMAG[3])
{
	int c = 0;
	int N = 200;
	int offset = 50;
	sensor_data_t raw_data;

	turnOnLED(turqoiseLed);

	while (c < N+offset)
	{
		OSA_TimeDelay(10);

		get_raw_data_sample_basic(&raw_data);

		if (c > offset)
		{
			for(int k = 0; k < 3; k++)
			{
				stdACC[k] += pow(( (float) raw_data.accel[k])*MPU9250_GPERCOUNT - meanACC[k],2)/(float)N;
				stdGYR[k] += pow(( (float) raw_data.gyro[k])*MPU9250_DEGPERSECPERCOUNT - meanGYR[k],2)/(float)N;
				stdMAG[k] += pow(( (float) raw_data.mag[k])*MPU9250_UTPERCOUNT - meanMAG[k],2)/(float)N;
			}
		}

		c++;

	}

	for(int k = 0; k < 3; k++)
	{
		stdACC[k] = sqrt(stdACC[k]);
		stdGYR[k] = sqrt(stdGYR[k]);
		stdMAG[k] = sqrt(stdMAG[k]);
	}

	turnOnLED(greenLed);
	OSA_TimeDelay(500);


}


void computeMinMaxVector(float minACC[3], float minGYR[3], float minMAG[3],float maxACC[3], float maxGYR[3], float maxMAG[3])
{
	int c = 0;
	int N = 200;
	int offset = 50;


	sensor_data_t raw_data;

	bool ret;

	turnOnLED(turqoiseLed);

	while (c < N+offset)
	{
		OSA_TimeDelay(10);

		get_raw_data_sample_basic(&raw_data);

		if(c > offset)
		{
			for(int k = 0; k < 3; k++)

			{
				minACC[k] = fmin(minACC[k],( (float) raw_data.accel[k])*MPU9250_GPERCOUNT);
				minGYR[k] = fmin(minGYR[k],( (float) raw_data.gyro[k])*MPU9250_DEGPERSECPERCOUNT);
				minMAG[k] = fmin(minMAG[k],( (float) raw_data.mag[k])*MPU9250_UTPERCOUNT);

				maxACC[k] = fmax(maxACC[k],( (float) raw_data.accel[k])*MPU9250_GPERCOUNT);
				maxGYR[k] = fmax(maxGYR[k],( (float) raw_data.gyro[k])*MPU9250_DEGPERSECPERCOUNT);
				maxMAG[k] = fmax(maxMAG[k],( (float) raw_data.mag[k])*MPU9250_UTPERCOUNT);
			}
		}

		c++;
	}

	turnOnLED(greenLed);
	OSA_TimeDelay(500);

}



bool checkInitialGyrData(float m, float M, float std)
{
	return !((M > 10)|(m < -10)|(std > 1));
}

bool checkInitialAccData(float m, float M, float std)
{
	return !((M > 2)|(m < -2)|(std > 0.1));
}

bool checkInitialMagData(float m, float M, float std)
{
	return !((M > 500)|(m < -500)|(std > 5));
}

bool tsSelftest()
{
	bool retboolG;
	int c = 0;
	int N = 500;
	uint8_t sensor_num;

	float MAG_RANGE = 4800;

	sensor_data_t raw_data;

	bool goodAcc[4];
	bool goodGyr[4];
	bool goodMag[4];

	//mpu9250_init();
	//mpu9250_start();

	float meanACC[3], meanGYR[3], meanMAG[3];
	float stdACC[3], stdGYR[3], stdMAG[3];
	float minACC[3], minGYR[3], minMAG[3];
	float maxACC[3], maxGYR[3], maxMAG[3];

	for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {

		// Must set the correct CS to select the correct Sensor
		mpu9250_enable_sensor(sensor_num);

		// Initialize Values
		for (uint8_t qq = 0; qq < 3; qq++) {
			meanACC[qq] = 0;
			meanGYR[qq] = 0;
			meanMAG[qq] = 0;

			stdACC[qq] = 0;
			stdGYR[qq] = 0;
			stdMAG[qq] = 0;

			minACC[qq] = ACCEL_RANGE_G;
			minGYR[qq] = GYRO_RANGE_DPS;
			minMAG[qq] = MAG_RANGE;

			maxACC[qq] = -ACCEL_RANGE_G;
			maxGYR[qq] = -GYRO_RANGE_DPS;
			maxMAG[qq] = -MAG_RANGE;
		}

		computeAverageVector(meanACC,meanGYR,meanMAG);

		computeMinMaxVector(minACC,minGYR,minMAG,maxACC,maxGYR,maxMAG);

		computeStdVector(meanACC,meanGYR,meanMAG,stdACC,stdGYR,stdMAG);

		//Thresholding
		for (int k = 0; k < 3; k++)
		{

			goodAcc[k] = checkInitialAccData(minACC[k],maxACC[k],stdACC[k]);

			goodGyr[k] = checkInitialGyrData(minGYR[k],maxGYR[k],stdGYR[k]);

			goodMag[k] = checkInitialMagData(minMAG[k],maxMAG[k],stdMAG[k]);

		}

		goodAcc[4] = goodAcc[0]&goodAcc[1]&goodAcc[2];
		goodGyr[4] = goodGyr[0]&goodGyr[1]&goodGyr[2];
		goodMag[4] = goodMag[0]&goodMag[1]&goodMag[2];


		if (!(goodAcc[4]&goodGyr[4]&goodMag[4]))
		{
			turnOnLED(redLed);
			OSA_TimeDelay(1000);
		} else {
			turnOnLED(offLed);
			OSA_TimeDelay(200);
			turnOnLED(greenLed);
		}
	}

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

void flash_write_2bytes(uint32_t value, uint32_t addr)
{
#if USE_FLASH

	uint8_t program_flash[4] = {0};

	program_flash[0] = value & 0xFF;
	program_flash[1] = (value & 0xFF00) >> 8;
	program_flash[2] = 0x00;
	program_flash[3] = 0x00;


	err_t flash_ret = ts_write_to_flash(addr, 4, program_flash);
	if (flash_ret != E_OK)
		blinkLEDerror(redLed, 1000);


#endif

}

int16_t flash_read_2bytes(uint32_t addr, int16_t defaultValue)
{
#if USE_FLASH

	int16_t out;

	int16_t tmp_val;

	// Get int16 from memory
	tmp_val = ts_read_from_flash(addr);

	out = *((uint16_t*)&tmp_val);

	if (tmp_val == UINT32_MAX)
		out = defaultValue;

	return out;

#endif
}

void flash_write_4bytes(uint32_t value, uint32_t addr)
{
#if USE_FLASH

	uint8_t program_flash[4] = {0};

	program_flash[0] = value & 0xFF;
	program_flash[1] = (value & 0xFF00) >> 8;
	program_flash[2] = (value & 0xFF0000) >> 16;
	program_flash[3] = (value & 0xFF000000) >> 24;

	err_t flash_ret = ts_write_to_flash(addr, 4, program_flash);
	if (flash_ret != E_OK)
		blinkLEDerror(redLed, 1000);


#endif
}

uint32_t flash_read_4bytes(uint32_t addr, uint32_t defaultValue)
{
#if USE_FLASH

	uint32_t out;

	// Get 4 bytes from memory
	uint32_t tmp_val = ts_read_from_flash(addr);

	if (tmp_val != UINT32_MAX)
		out = tmp_val;
	else
		out = defaultValue;


	return out;

#endif
}

void flash_write_satID_calib(int par, uint32_t in)
{
#if PIVOT_3_0
#if USE_FLASH
	uint32_t satID, hubID;
	int16_t 	gBx[MAX_SENSORS_IN_CS], gBy[MAX_SENSORS_IN_CS], gBz[MAX_SENSORS_IN_CS],
				aBx[MAX_SENSORS_IN_CS], aBy[MAX_SENSORS_IN_CS], aBz[MAX_SENSORS_IN_CS],
				aSx[MAX_SENSORS_IN_CS], aSy[MAX_SENSORS_IN_CS], aSz[MAX_SENSORS_IN_CS];
	uint32_t 	mBx32[MAX_SENSORS_IN_CS], mBy32[MAX_SENSORS_IN_CS], mBz32[MAX_SENSORS_IN_CS];
	uint32_t 	mBFactx32[MAX_SENSORS_IN_CS], mBFacty32[MAX_SENSORS_IN_CS], mBFactz32[MAX_SENSORS_IN_CS];
	int16_t 	lcp;
	uint32_t 	base;

	// Save current flash state
	satID = flash_read_4bytes(TS_FLASH_STARTING_ADDR, sat_id);
	hubID = flash_read_4bytes(TS_FLASH_HUBID_ADDR, DEFAULT_HUB_ID);
	lcp = flash_read_2bytes(TS_FLASH_LCP_ADDR, INITIAL_LCP);

	for (uint8_t qq = 0; qq < MAX_SENSORS_IN_CS; qq++) {
		if (qq == 0) {
			gBx[qq] = flash_read_2bytes(TS_FLASH_GYRO_X_ADDR, 0);
			gBy[qq] = flash_read_2bytes(TS_FLASH_GYRO_Y_ADDR, 0);
			gBz[qq] = flash_read_2bytes(TS_FLASH_GYRO_Z_ADDR, 0);

			aBx[qq] = flash_read_2bytes(TS_FLASH_ACCEL_X_ADDR, 0);
			aBy[qq] = flash_read_2bytes(TS_FLASH_ACCEL_Y_ADDR, 0);
			aBz[qq] = flash_read_2bytes(TS_FLASH_ACCEL_Z_ADDR, 0);

			aSx[qq] = flash_read_2bytes(TS_FLASH_SENS_ACCEL_X_ADDR, (int16_t)(1.0F/MPU9250_GPERCOUNT));
			aSy[qq] = flash_read_2bytes(TS_FLASH_SENS_ACCEL_Y_ADDR, (int16_t)(1.0F/MPU9250_GPERCOUNT));
			aSz[qq] = flash_read_2bytes(TS_FLASH_SENS_ACCEL_Z_ADDR, (int16_t)(1.0F/MPU9250_GPERCOUNT));

			mBx32[qq] = flash_read_4bytes(TS_FLASH_MAG_X_ADDR, 0);
			mBy32[qq] = flash_read_4bytes(TS_FLASH_MAG_Y_ADDR, 0);
			mBz32[qq] = flash_read_4bytes(TS_FLASH_MAG_Z_ADDR, 0);

			mBFactx32[qq] = flash_read_4bytes(TS_FLASH_MAG_FACTORY_X_ADDR, 0);
			mBFacty32[qq] = flash_read_4bytes(TS_FLASH_MAG_FACTORY_Y_ADDR, 0);
			mBFactz32[qq] = flash_read_4bytes(TS_FLASH_MAG_FACTORY_Z_ADDR, 0);
		} else {
			base = TS_FLASH_SENS_BASE(qq);

			gBx[qq] = flash_read_2bytes(TS_FLASH_GYRO_X_ADDR_N(base), 0);
			gBy[qq] = flash_read_2bytes(TS_FLASH_GYRO_Y_ADDR_N(base), 0);
			gBz[qq] = flash_read_2bytes(TS_FLASH_GYRO_Z_ADDR_N(base), 0);

			aBx[qq] = flash_read_2bytes(TS_FLASH_ACCEL_X_ADDR_N(base), 0);
			aBy[qq] = flash_read_2bytes(TS_FLASH_ACCEL_Y_ADDR_N(base), 0);
			aBz[qq] = flash_read_2bytes(TS_FLASH_ACCEL_Z_ADDR_N(base), 0);

			aSx[qq] = flash_read_2bytes(TS_FLASH_SENS_ACCEL_X_ADDR_N(base), (int16_t)(1.0F/MPU9250_GPERCOUNT));
			aSy[qq] = flash_read_2bytes(TS_FLASH_SENS_ACCEL_Y_ADDR_N(base), (int16_t)(1.0F/MPU9250_GPERCOUNT));
			aSz[qq] = flash_read_2bytes(TS_FLASH_SENS_ACCEL_Z_ADDR_N(base), (int16_t)(1.0F/MPU9250_GPERCOUNT));

			mBx32[qq] = flash_read_4bytes(TS_FLASH_MAG_X_ADDR_N(base), 0);
			mBy32[qq] = flash_read_4bytes(TS_FLASH_MAG_Y_ADDR_N(base), 0);
			mBz32[qq] = flash_read_4bytes(TS_FLASH_MAG_Z_ADDR_N(base), 0);

			mBFactx32[qq] = flash_read_4bytes(TS_FLASH_MAG_FACTORY_X_ADDR_N(base), 0);
			mBFacty32[qq] = flash_read_4bytes(TS_FLASH_MAG_FACTORY_Y_ADDR_N(base), 0);
			mBFactz32[qq] = flash_read_4bytes(TS_FLASH_MAG_FACTORY_Z_ADDR_N(base), 0);
		}
	}


	// erase the whole 2Kb sector
	err_t flash_ret;

	flash_ret = ts_erase_flash_sector();
	if (flash_ret != E_OK) {
		blinkLEDerror(redLed, 300);
	}


	// Overwrite the local variables according to the command
	switch (par)
	{

		case 0: // Sat Id only
			satID = in;
			break;

		case 1: // Acc Calib only
			for (uint8_t qq = 0; qq < MAX_SENSORS_IN_CS; qq++) {
				aBx[qq] = accBiasInt16_X[qq];
				aBy[qq] = accBiasInt16_Y[qq];
				aBz[qq] = accBiasInt16_Z[qq];

				aSx[qq] = accSensInt16_X[qq];
				aSy[qq] = accSensInt16_Y[qq];
				aSz[qq] = accSensInt16_Z[qq];
			}

			break;

		case 2: // Gyr Calib only
			for (uint8_t qq = 0; qq < MAX_SENSORS_IN_CS; qq++) {
				gBx[qq] = gyrBiasInt16_X[qq];
				gBy[qq] = gyrBiasInt16_Y[qq];
				gBz[qq] = gyrBiasInt16_Z[qq];
			}
			break;

		case 3: // Mag Calib only
			for (uint8_t qq = 0; qq < MAX_SENSORS_IN_CS; qq++) {
				mBx32[qq] = *((uint32_t*) &(thisMagCal[qq].fV[0]));
				mBy32[qq] = *((uint32_t*) &thisMagCal[qq].fV[1]);
				mBz32[qq] = *((uint32_t*) &thisMagCal[qq].fV[2]);
			}
			break;

		case 4: // All Sensor Calibs (NO Sat ID)
			for (uint8_t qq = 0; qq < MAX_SENSORS_IN_CS; qq++) {

				aBx[qq] = accBiasInt16_X[qq];
				aBy[qq] = accBiasInt16_Y[qq];
				aBz[qq] = accBiasInt16_Z[qq];

				aSx[qq] = accSensInt16_X[qq];
				aSy[qq] = accSensInt16_Y[qq];
				aSz[qq] = accSensInt16_Z[qq];

				gBx[qq] = gyrBiasInt16_X[qq];
				gBy[qq] = gyrBiasInt16_Y[qq];
				gBz[qq] = gyrBiasInt16_Z[qq];

				mBx32[qq] = *((uint32_t*) &thisMagCal[qq].fV[0]);
				mBy32[qq] = *((uint32_t*) &thisMagCal[qq].fV[1]);
				mBz32[qq] = *((uint32_t*) &thisMagCal[qq].fV[2]);
			}

			break;

		case 5:  // All Sensor Calibs (no factory mag Bias) + Sat ID
			satID = in;

			for (uint8_t qq = 0; qq < MAX_SENSORS_IN_CS; qq++) {
				aBx[qq] = accBiasInt16_X[qq];
				aBy[qq] = accBiasInt16_Y[qq];
				aBz[qq] = accBiasInt16_Z[qq];

				aSx[qq] = accSensInt16_X[qq];
				aSy[qq] = accSensInt16_Y[qq];
				aSz[qq] = accSensInt16_Z[qq];

				gBx[qq] = gyrBiasInt16_X[qq];
				gBy[qq] = gyrBiasInt16_Y[qq];
				gBz[qq] = gyrBiasInt16_Z[qq];

				mBx32[qq] = *((uint32_t*) &thisMagCal[qq].fV[0]);
				mBy32[qq] = *((uint32_t*) &thisMagCal[qq].fV[1]);
				mBz32[qq] = *((uint32_t*) &thisMagCal[qq].fV[2]);
			}
			break;

		case 6:  // hubID
			hubID = in;
			break;

		case 7: // Factory mag Bias
			for (uint8_t qq = 0; qq < MAX_SENSORS_IN_CS; qq++) {

				//Write in the "usual" mag Bias locations anyways
				mBx32[qq] = *((uint32_t*) &thisMagCal[qq].fV[0]);
				mBy32[qq] = *((uint32_t*) &thisMagCal[qq].fV[1]);
				mBz32[qq] = *((uint32_t*) &thisMagCal[qq].fV[2]);

				// Write in the Factory magBias location
				mBFactx32[qq] = *((uint32_t*) &thisMagCal[qq].fV[0]);
				mBFacty32[qq] = *((uint32_t*) &thisMagCal[qq].fV[1]);
				mBFactz32[qq] = *((uint32_t*) &thisMagCal[qq].fV[2]);
			}
			break;

		case 8:  // All Sensor Calibs (WITH factory mag Bias) + Sat ID
			satID = in;
			for (uint8_t qq = 0; qq < MAX_SENSORS_IN_CS; qq++) {
				aBx[qq] = accBiasInt16_X[qq];
				aBy[qq] = accBiasInt16_Y[qq];
				aBz[qq] = accBiasInt16_Z[qq];

				aSx[qq] = accSensInt16_X[qq];
				aSy[qq] = accSensInt16_Y[qq];
				aSz[qq] = accSensInt16_Z[qq];

				gBx[qq] = gyrBiasInt16_X[qq];
				gBy[qq] = gyrBiasInt16_Y[qq];
				gBz[qq] = gyrBiasInt16_Z[qq];

				mBx32[qq] = *((uint32_t*) &thisMagCal[qq].fV[0]);
				mBy32[qq] = *((uint32_t*) &thisMagCal[qq].fV[1]);
				mBz32[qq] = *((uint32_t*) &thisMagCal[qq].fV[2]);

				mBFactx32[qq] = *((uint32_t*) &thisMagCal[qq].fV[0]);
				mBFacty32[qq] = *((uint32_t*) &thisMagCal[qq].fV[1]);
				mBFactz32[qq] = *((uint32_t*) &thisMagCal[qq].fV[2]);
			}
			break;

		case 9: // write current lcp channel in flash
			if ((in >= 0) && (in <= MAX_LCP))
				lcp = in;
			else
				onlyTest(true);

			break;

	}

	// Write the flash
	flash_write_4bytes(satID, TS_FLASH_STARTING_ADDR);

	if (hubID != DEFAULT_HUB_ID)
		flash_write_4bytes(hubID, TS_FLASH_HUBID_ADDR);

	if (lcp != INITIAL_LCP)
		flash_write_2bytes(lcp,TS_FLASH_LCP_ADDR);

	for (uint8_t qq = 0; qq < MAX_SENSORS_IN_CS; qq++) {
		if (qq == 0) {
			flash_write_2bytes(aBx[qq], TS_FLASH_ACCEL_X_ADDR);
			flash_write_2bytes(aBy[qq], TS_FLASH_ACCEL_Y_ADDR);
			flash_write_2bytes(aBz[qq], TS_FLASH_ACCEL_Z_ADDR);

			flash_write_2bytes(aSx[qq], TS_FLASH_SENS_ACCEL_X_ADDR);
			flash_write_2bytes(aSy[qq], TS_FLASH_SENS_ACCEL_Y_ADDR);
			flash_write_2bytes(aSz[qq], TS_FLASH_SENS_ACCEL_Z_ADDR);

			flash_write_2bytes(gBx[qq], TS_FLASH_GYRO_X_ADDR);
			flash_write_2bytes(gBy[qq], TS_FLASH_GYRO_Y_ADDR);
			flash_write_2bytes(gBz[qq], TS_FLASH_GYRO_Z_ADDR);

			flash_write_4bytes(mBx32[qq], TS_FLASH_MAG_X_ADDR);
			flash_write_4bytes(mBy32[qq], TS_FLASH_MAG_Y_ADDR);
			flash_write_4bytes(mBz32[qq], TS_FLASH_MAG_Z_ADDR);

			flash_write_4bytes(mBFactx32[qq], TS_FLASH_MAG_FACTORY_X_ADDR);
			flash_write_4bytes(mBFacty32[qq], TS_FLASH_MAG_FACTORY_Y_ADDR);
			flash_write_4bytes(mBFactz32[qq], TS_FLASH_MAG_FACTORY_Z_ADDR);
		} else {
			base = TS_FLASH_SENS_BASE(qq);

			flash_write_2bytes(aBx[qq], TS_FLASH_ACCEL_X_ADDR_N(base));
			flash_write_2bytes(aBy[qq], TS_FLASH_ACCEL_Y_ADDR_N(base));
			flash_write_2bytes(aBz[qq], TS_FLASH_ACCEL_Z_ADDR_N(base));

			flash_write_2bytes(aSx[qq], TS_FLASH_SENS_ACCEL_X_ADDR_N(base));
			flash_write_2bytes(aSy[qq], TS_FLASH_SENS_ACCEL_Y_ADDR_N(base));
			flash_write_2bytes(aSz[qq], TS_FLASH_SENS_ACCEL_Z_ADDR_N(base));

			flash_write_2bytes(gBx[qq], TS_FLASH_GYRO_X_ADDR_N(base));
			flash_write_2bytes(gBy[qq], TS_FLASH_GYRO_Y_ADDR_N(base));
			flash_write_2bytes(gBz[qq], TS_FLASH_GYRO_Z_ADDR_N(base));

			flash_write_4bytes(mBx32[qq], TS_FLASH_MAG_X_ADDR_N(base));
			flash_write_4bytes(mBy32[qq], TS_FLASH_MAG_Y_ADDR_N(base));
			flash_write_4bytes(mBz32[qq], TS_FLASH_MAG_Z_ADDR_N(base));

			flash_write_4bytes(mBFactx32[qq], TS_FLASH_MAG_FACTORY_X_ADDR_N(base));
			flash_write_4bytes(mBFacty32[qq], TS_FLASH_MAG_FACTORY_Y_ADDR_N(base));
			flash_write_4bytes(mBFactz32[qq], TS_FLASH_MAG_FACTORY_Z_ADDR_N(base));
		}
	}

#endif
#else
#if USE_FLASH
	uint32_t satID, hubID;
	int16_t gBx, gBy, gBz, aBx, aBy, aBz, aSx, aSy, aSz;
	uint32_t mBx32, mBy32, mBz32;
	uint32_t mBFactx32, mBFacty32, mBFactz32;
	int16_t lcp;

	// Save current flash state
	satID = flash_read_4bytes(TS_FLASH_STARTING_ADDR, sat_id);

	gBx = flash_read_2bytes(TS_FLASH_GYRO_X_ADDR, 0);
	gBy = flash_read_2bytes(TS_FLASH_GYRO_Y_ADDR, 0);
	gBz = flash_read_2bytes(TS_FLASH_GYRO_Z_ADDR, 0);

	aBx = flash_read_2bytes(TS_FLASH_ACCEL_X_ADDR, 0);
	aBy = flash_read_2bytes(TS_FLASH_ACCEL_Y_ADDR, 0);
	aBz = flash_read_2bytes(TS_FLASH_ACCEL_Z_ADDR, 0);

	aSx = flash_read_2bytes(TS_FLASH_SENS_ACCEL_X_ADDR, (int16_t)(1.0F/MPU9250_GPERCOUNT));
	aSy = flash_read_2bytes(TS_FLASH_SENS_ACCEL_Y_ADDR, (int16_t)(1.0F/MPU9250_GPERCOUNT));
	aSz = flash_read_2bytes(TS_FLASH_SENS_ACCEL_Z_ADDR, (int16_t)(1.0F/MPU9250_GPERCOUNT));

	mBx32 = flash_read_4bytes(TS_FLASH_MAG_X_ADDR, 0);
	mBy32 = flash_read_4bytes(TS_FLASH_MAG_Y_ADDR, 0);
	mBz32 = flash_read_4bytes(TS_FLASH_MAG_Z_ADDR, 0);

	hubID = flash_read_4bytes(TS_FLASH_HUBID_ADDR, DEFAULT_HUB_ID);

	mBFactx32 = flash_read_4bytes(TS_FLASH_MAG_FACTORY_X_ADDR, 0);
	mBFacty32 = flash_read_4bytes(TS_FLASH_MAG_FACTORY_Y_ADDR, 0);
	mBFactz32 = flash_read_4bytes(TS_FLASH_MAG_FACTORY_Z_ADDR, 0);

	lcp = flash_read_2bytes(TS_FLASH_LCP_ADDR, INITIAL_LCP);


	// erase the whole 2Kb sector
	err_t flash_ret;

	flash_ret = ts_erase_flash_sector();
	if (flash_ret != E_OK) {
		blinkLEDerror(redLed, 300);
	}


	// Overwrite the local variables according to the command
	switch (par)
	{

		case 0: // Sat Id only
			satID = in;
			break;

		case 1: // Acc Calib only
			aBx = accBiasInt16_X;
			aBy = accBiasInt16_Y;
			aBz = accBiasInt16_Z;

			aSx = accSensInt16_X;
			aSy = accSensInt16_Y;
			aSz = accSensInt16_Z;
			break;

		case 2: // Gyr Calib only
			gBx = gyrBiasInt16_X;
			gBy = gyrBiasInt16_Y;
			gBz = gyrBiasInt16_Z;
			break;

		case 3: // Mag Calib only
			mBx32 = *((uint32_t*) &(thisMagCal[mpu9250_num].fV[0]));
			mBy32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[1]);
			mBz32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[2]);

			break;

		case 4: // All Sensor Calibs (NO Sat ID)

			aBx = accBiasInt16_X;
			aBy = accBiasInt16_Y;
			aBz = accBiasInt16_Z;

			aSx = accSensInt16_X;
			aSy = accSensInt16_Y;
			aSz = accSensInt16_Z;

			gBx = gyrBiasInt16_X;
			gBy = gyrBiasInt16_Y;
			gBz = gyrBiasInt16_Z;

			mBx32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[0]);
			mBy32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[1]);
			mBz32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[2]);

			break;

		case 5:  // All Sensor Calibs (no factory mag Bias) + Sat ID
			satID = in;

			aBx = accBiasInt16_X;
			aBy = accBiasInt16_Y;
			aBz = accBiasInt16_Z;

			aSx = accSensInt16_X;
			aSy = accSensInt16_Y;
			aSz = accSensInt16_Z;

			gBx = gyrBiasInt16_X;
			gBy = gyrBiasInt16_Y;
			gBz = gyrBiasInt16_Z;

			mBx32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[0]);
			mBy32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[1]);
			mBz32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[2]);

			break;

		case 6:  // hubID
			hubID = in;
			break;

		case 7: // Factory mag Bias

			//Write in the "usual" mag Bias locations anyways
			mBx32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[0]);
			mBy32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[1]);
			mBz32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[2]);

			// Write in the Factory magBias location
			mBFactx32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[0]);
			mBFacty32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[1]);
			mBFactz32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[2]);

			break;

		case 8:  // All Sensor Calibs (WITH factory mag Bias) + Sat ID
			satID = in;

			aBx = accBiasInt16_X;
			aBy = accBiasInt16_Y;
			aBz = accBiasInt16_Z;

			aSx = accSensInt16_X;
			aSy = accSensInt16_Y;
			aSz = accSensInt16_Z;

			gBx = gyrBiasInt16_X;
			gBy = gyrBiasInt16_Y;
			gBz = gyrBiasInt16_Z;

			mBx32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[0]);
			mBy32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[1]);
			mBz32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[2]);

			mBFactx32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[0]);
			mBFacty32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[1]);
			mBFactz32 = *((uint32_t*) &thisMagCal[mpu9250_num].fV[2]);
			break;

		case 9: // write current lcp channel in flash
			if ((in >= 0) && (in <= MAX_LCP))
				lcp = in;
			else
				onlyTest(true);

			break;

	}

	// Write the flash
	flash_write_4bytes(satID, TS_FLASH_STARTING_ADDR);

	flash_write_2bytes(aBx, TS_FLASH_ACCEL_X_ADDR);
	flash_write_2bytes(aBy, TS_FLASH_ACCEL_Y_ADDR);
	flash_write_2bytes(aBz, TS_FLASH_ACCEL_Z_ADDR);

	flash_write_2bytes(aSx, TS_FLASH_SENS_ACCEL_X_ADDR);
	flash_write_2bytes(aSy, TS_FLASH_SENS_ACCEL_Y_ADDR);
	flash_write_2bytes(aSz, TS_FLASH_SENS_ACCEL_Z_ADDR);

	flash_write_2bytes(gBx, TS_FLASH_GYRO_X_ADDR);
	flash_write_2bytes(gBy, TS_FLASH_GYRO_Y_ADDR);
	flash_write_2bytes(gBz, TS_FLASH_GYRO_Z_ADDR);

	flash_write_4bytes(mBx32,TS_FLASH_MAG_X_ADDR);
	flash_write_4bytes(mBy32,TS_FLASH_MAG_Y_ADDR);
	flash_write_4bytes(mBz32,TS_FLASH_MAG_Z_ADDR);

	if (hubID != DEFAULT_HUB_ID)
		flash_write_4bytes(hubID, TS_FLASH_HUBID_ADDR);

	flash_write_4bytes(mBFactx32,TS_FLASH_MAG_FACTORY_X_ADDR);
	flash_write_4bytes(mBFacty32,TS_FLASH_MAG_FACTORY_Y_ADDR);
	flash_write_4bytes(mBFactz32,TS_FLASH_MAG_FACTORY_Z_ADDR);

	if (lcp != INITIAL_LCP)
		flash_write_2bytes(lcp,TS_FLASH_LCP_ADDR);


#endif
#endif //PIVOT_3_0
}

void flash_load_satID_calib()
{
#if USE_FLASH

	uint32_t 	mBx32, mBy32, mBz32;
	uint8_t 	sensor_num;
	uint32_t 	base;	/* Base flash address for a particular sensor data */

	sat_id = flash_read_4bytes(TS_FLASH_STARTING_ADDR,sat_id);
	dataToSend.data.sat_id = sat_id;

	for (sensor_num = 0; sensor_num < MAX_SENSORS_IN_CS; sensor_num++) {
		if (sensor_num == 0) {
			gyrBiasInt16_X[sensor_num] = flash_read_2bytes(TS_FLASH_GYRO_X_ADDR, 0);
			gyrBiasInt16_Y[sensor_num] = flash_read_2bytes(TS_FLASH_GYRO_Y_ADDR, 0);
			gyrBiasInt16_Z[sensor_num] = flash_read_2bytes(TS_FLASH_GYRO_Z_ADDR, 0);

			accBiasInt16_X[sensor_num] = flash_read_2bytes(TS_FLASH_ACCEL_X_ADDR, 0);
			accBiasInt16_Y[sensor_num] = flash_read_2bytes(TS_FLASH_ACCEL_Y_ADDR, 0);
			accBiasInt16_Z[sensor_num] = flash_read_2bytes(TS_FLASH_ACCEL_Z_ADDR, 0);

			accSensInt16_X[sensor_num] = flash_read_2bytes(TS_FLASH_SENS_ACCEL_X_ADDR, (int16_t)(1.0F/MPU9250_GPERCOUNT));
			accSensInt16_Y[sensor_num] = flash_read_2bytes(TS_FLASH_SENS_ACCEL_Y_ADDR, (int16_t)(1.0F/MPU9250_GPERCOUNT));
			accSensInt16_Z[sensor_num] = flash_read_2bytes(TS_FLASH_SENS_ACCEL_Z_ADDR, (int16_t)(1.0F/MPU9250_GPERCOUNT));

			mBx32 = flash_read_4bytes(TS_FLASH_MAG_X_ADDR, 0);
			mBy32 = flash_read_4bytes(TS_FLASH_MAG_Y_ADDR, 0);
			mBz32 = flash_read_4bytes(TS_FLASH_MAG_Z_ADDR, 0);

			thisMagCal[sensor_num].fV[0] = *((float *) &mBx32);
			thisMagCal[sensor_num].fV[1] = *((float *) &mBy32);
			thisMagCal[sensor_num].fV[2] = *((float *) &mBz32);
		} else {
			base = TS_FLASH_SENS_BASE(sensor_num);

			gyrBiasInt16_X[sensor_num] = flash_read_2bytes(TS_FLASH_GYRO_X_ADDR_N(base), 0);
			gyrBiasInt16_Y[sensor_num] = flash_read_2bytes(TS_FLASH_GYRO_Y_ADDR_N(base), 0);
			gyrBiasInt16_Z[sensor_num] = flash_read_2bytes(TS_FLASH_GYRO_Z_ADDR_N(base), 0);

			accBiasInt16_X[sensor_num] = flash_read_2bytes(TS_FLASH_ACCEL_X_ADDR_N(base), 0);
			accBiasInt16_Y[sensor_num] = flash_read_2bytes(TS_FLASH_ACCEL_Y_ADDR_N(base), 0);
			accBiasInt16_Z[sensor_num] = flash_read_2bytes(TS_FLASH_ACCEL_Z_ADDR_N(base), 0);

			accSensInt16_X[sensor_num] = flash_read_2bytes(TS_FLASH_SENS_ACCEL_X_ADDR_N(base), (int16_t)(1.0F/MPU9250_GPERCOUNT));
			accSensInt16_Y[sensor_num] = flash_read_2bytes(TS_FLASH_SENS_ACCEL_Y_ADDR_N(base), (int16_t)(1.0F/MPU9250_GPERCOUNT));
			accSensInt16_Z[sensor_num] = flash_read_2bytes(TS_FLASH_SENS_ACCEL_Z_ADDR_N(base), (int16_t)(1.0F/MPU9250_GPERCOUNT));

			mBx32 = flash_read_4bytes(TS_FLASH_MAG_X_ADDR_N(base), 0);
			mBy32 = flash_read_4bytes(TS_FLASH_MAG_Y_ADDR_N(base), 0);
			mBz32 = flash_read_4bytes(TS_FLASH_MAG_Z_ADDR_N(base), 0);

			thisMagCal[sensor_num].fV[0] = *((float *) &mBx32);
			thisMagCal[sensor_num].fV[1] = *((float *) &mBy32);
			thisMagCal[sensor_num].fV[2] = *((float *) &mBz32);
		}
	}

	myHubID = flash_read_4bytes(TS_FLASH_HUBID_ADDR, DEFAULT_HUB_ID);
	dataToSend.data.hub_id = myHubID;

	myLCP = flash_read_2bytes(TS_FLASH_LCP_ADDR, INITIAL_LCP);
	dataToSend.data.nrdChannel = myLCP;

	#endif
}

void flash_restore_factory_mag_calib()
{
#if PIVOT_3_0
#if USE_FLASH
	uint32_t mBFactx32;
	uint32_t mBFacty32;
	uint32_t mBFactz32;
	uint32_t base;

	for (uint8_t qq = 0; qq < MAX_SENSORS_IN_CS; qq++) {
		if (qq == 0) {
			mBFactx32 = flash_read_4bytes(TS_FLASH_MAG_FACTORY_X_ADDR, 0);
			mBFacty32 = flash_read_4bytes(TS_FLASH_MAG_FACTORY_Y_ADDR, 0);
			mBFactz32 = flash_read_4bytes(TS_FLASH_MAG_FACTORY_Z_ADDR, 0);

			thisMagCal[qq].fV[0] = *((float *) &mBFactx32);
			thisMagCal[qq].fV[1] = *((float *) &mBFacty32);
			thisMagCal[qq].fV[2] = *((float *) &mBFactz32);
		} else {
			base = TS_FLASH_SENS_BASE(qq);
			mBFactx32 = flash_read_4bytes(TS_FLASH_MAG_FACTORY_X_ADDR_N(base), 0);
			mBFacty32 = flash_read_4bytes(TS_FLASH_MAG_FACTORY_Y_ADDR_N(base), 0);
			mBFactz32 = flash_read_4bytes(TS_FLASH_MAG_FACTORY_Z_ADDR_N(base), 0);

			thisMagCal[qq].fV[0] = *((float *) &mBFactx32);
			thisMagCal[qq].fV[1] = *((float *) &mBFacty32);
			thisMagCal[qq].fV[2] = *((float *) &mBFactz32);
		}
	}

	flash_write_satID_calib(7,0);

#endif
#else
#if USE_FLASH

	uint32_t mBFactx32 = flash_read_4bytes(TS_FLASH_MAG_FACTORY_X_ADDR, 0);
	uint32_t mBFacty32 = flash_read_4bytes(TS_FLASH_MAG_FACTORY_Y_ADDR, 0);
	uint32_t mBFactz32 = flash_read_4bytes(TS_FLASH_MAG_FACTORY_Z_ADDR, 0);

	thisMagCal[mpu9250_num].fV[0] = *((float *) &mBFactx32);
	thisMagCal[mpu9250_num].fV[1] = *((float *) &mBFacty32);
	thisMagCal[mpu9250_num].fV[2] = *((float *) &mBFactz32);

	flash_write_satID_calib(7,0);

#endif
#endif
}


/*******************************************************************************
 * Process Hub command coming from Nordic's SPI
 ******************************************************************************/
void processCommand(sat_mosi_packet_t packet) {
	uint8_t sensor_num;

	last_nordic_comm = k22f_get_rtc();

	//uint32_t timeStampOld = 0;
	int cmd_sat = packet.sat_command[SAT_COMMSTRUCT_COMMA];

	if ((estimateAccBias_run == false)||(cmd_sat == SAT_COMM_STOPCALACC))
	{
		//from < 0 to 9 RESERVED COMMANDS
		if(cmd_sat==SAT_COMM_TESTVALUE0)//test command 0
		{
			testCommand0(packet);
		}else if(cmd_sat==SAT_COMM_TESTVALUE1){//test command 1
			testCommand1(packet);
		}else if(cmd_sat==SAT_COMM_SETBTCOMCH && !start_dumping){//command 2 - set BT change comunication channel - used by Nordic; here only confirmation
			turnOnLED(purpleLed);
			dataToSend.data.nrdChannel=(packet.sat_command[SAT_COMMSTRUCT_VALY2]?packet.sat_command[SAT_COMMSTRUCT_VALX1]:dataToSend.data.nrdChannel);
			onlyTest(!packet.sat_command[SAT_COMMSTRUCT_VALY2]);
			onlyTest(!packet.sat_command[SAT_COMMSTRUCT_VALZ3]);
		}else if(cmd_sat==SAT_COMM_TESTCOMSTC){//command 3 - set COMMUNICATION TEST
			onlyTest(false);


			communicationTestCounterInitialization = 10;
			communicationTestCounter[COMMUNICATIONCOUNTER_SPIP]=0;
			communicationTestCounter[COMMUNICATIONCOUNTER_BUFF]=0;
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_COUN,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
						accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],
						thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
			}
		}else if(cmd_sat==SAT_COMM_SETWFCOMCH){//command 4 - set WIFI change comunication channel - used by HUB 22F; here only confirmation
			turnOnLED(purpleLed);
			onlyTest(!packet.sat_command[SAT_COMMSTRUCT_VALY2]);
			onlyTest(!packet.sat_command[SAT_COMMSTRUCT_VALZ3]);
		}else if(cmd_sat==SAT_COMM_SETBTCOMAU && !start_dumping){//command 5 - set BT change comunication channel automatically - generated from SAT NORDIC when non receive packets from HUB
			turnOnLED(purpleLed);
			OSA_TimeDelay(50);
			dataToSend.data.nrdChannel++;
			if(dataToSend.data.nrdChannel > packet.sat_command[SAT_COMMSTRUCT_VALX1]) //max BTT channel
				dataToSend.data.nrdChannel=0;
		}else if(cmd_sat==SAT_COMM_START_DUMP){//command 6 - START DUMPING
			start_dumping = true;
		}else if(cmd_sat==SAT_COMM_STOP_DUMPI){//command 7 - STOP DUMPING
			/* empty cbuf FAST_DRAIN */
			cbufInit(&sensorRecordQ);
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_QUMA,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
						accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],
						thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]); //CF
			}
			start_dumping = false;
			onlyTest(false);
		}else if(cmd_sat==SAT_COMM_MBIASSENDG){//command 10 - MAG BIAS SENDING MODE
			onlyTest(false);
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_BMAG,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
						accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],
						thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
			}
			globals.disable_magCalibrationAlways = false;

			fInitMagCalibration(&thisMagCal[mpu9250_num], &thisMagBuffer[mpu9250_num]);
		}else if(cmd_sat==SAT_COMM_NORMLSENDG){//command 11 - NORMAL SENDING MODE
//			onlyTest(false);
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_QUMA,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
						accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],
						thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
			}
		}else if(cmd_sat==SAT_COMM_SETMAGBIAS){//command 12 - NORMAL SENDING MODE SET BIAS
			onlyTest(false);
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_QUMA,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
						accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],
						((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALX1]))/1000.0F,
						((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALY2]))/1000.0F,
						((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALZ3]))/1000.0F); //CF
			}

		}else if(cmd_sat==SAT_COMM_MSTBIASENDG){//command 13 - MAG BIAS SETTED SENDING MODE
			onlyTest(false);
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_BMAG,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
						accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],
						thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
			}
			globals.disable_magCalibrationAlways = true;
		}else if(cmd_sat==SAT_COMM_B_GYRO_EST){// command 14 - GYR BIAS ESTIMATION
			estimateGyroBias_run = true;
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_QUMA,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
									accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],0,0,0,
									thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
			}
		}else if(cmd_sat==SAT_COMM_B_ACCE_EST){// command 15 - ACC BIAS ESTIMATION
			estimateAccBias_run = true;
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_QUMA,(int16_t)(1.0F/MPU9250_GPERCOUNT),(int16_t)(1.0F/MPU9250_GPERCOUNT),(int16_t)(1.0F/MPU9250_GPERCOUNT),
						0,0,0,gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],
						thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
			}
		}else if(cmd_sat==SAT_COMM_BACCGYRSND){// command 16 - GYR & ACC BIAS SENDING
			onlyTest(false);
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_BGYR,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
						accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],
						thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
			}

		}else if(cmd_sat==SAT_COMM_GETRELEASE){// command 18 - write release version
			onlyTest(false);
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_VERS,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
						accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],
						thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
			}

		}else if(cmd_sat==SAT_COMM_SETACCSENS){//command 19 - SET ACC SENSITIVITY
			onlyTest(false);

			float accSensX = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALX1]))/1000.0F;
			float accSensY = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALY2]))/1000.0F;
			float accSensZ = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALZ3]))/1000.0F;

			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_QUMA,(int16_t)(accSensX/MPU9250_GPERCOUNT),(int16_t)(accSensY/MPU9250_GPERCOUNT),
						(int16_t)(accSensZ/MPU9250_GPERCOUNT),accBiasInt16_X[sensor_num],accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],
						gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
			}
		}else if(cmd_sat==SAT_COMM_SETACCBIAS){//command 20 - SET ACC BIAS
			onlyTest(false);

			float accBiasX = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALX1]))/1000.0F;
			float accBiasY = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALY2]))/1000.0F;
			float accBiasZ = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALZ3]))/1000.0F;

			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_QUMA,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],(int16_t)(accBiasX/MPU9250_GPERCOUNT),
						(int16_t)(accBiasY/MPU9250_GPERCOUNT),(int16_t)(accBiasZ/MPU9250_GPERCOUNT),gyrBiasInt16_X[sensor_num],
						gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
			}
		}else if(cmd_sat==SAT_COMM_SETGYRBIAS){//command 21 - SET GYR BIAS
			onlyTest(false);

			float gyrBiasX = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALX1]))/1000.0F;
			float gyrBiasY = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALY2]))/1000.0F;
			float gyrBiasZ = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALZ3]))/1000.0F;

			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_QUMA,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],accBiasInt16_Y[sensor_num],
						accBiasInt16_Z[sensor_num],(int16_t)(gyrBiasX/MPU9250_DEGPERSECPERCOUNT),(int16_t)(gyrBiasY/MPU9250_DEGPERSECPERCOUNT),
						(int16_t)(gyrBiasZ/MPU9250_DEGPERSECPERCOUNT),thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
			}
		}else if(cmd_sat==SAT_COMM_TSSELFTEST){ //command 22 - RUN TS SELF TEST
			tsSelfTest_run = true;
			onlyTest(false);
		}else if(cmd_sat==SAT_COMM_FLASHWRITE){ //command 23 - WRITE ID AND/OR CALIB PARAMETERS IN THE FLASH MEMORY
			flash_write_satID_calib(packet.sat_command[SAT_COMMSTRUCT_VALX1], packet.sat_command[SAT_COMMSTRUCT_VALY2]);
			onlyTest(false);
		}else if(cmd_sat==SAT_COMM_FLASH_READ){ //command 24 - READ ID AND/OR CALIB PARAMETERS IN THE FLASH MEMORY
			flash_load_satID_calib();
			//nrf51822_init();
			onlyTest(false);
		}else if(cmd_sat==SAT_COMM_FLASHERASE){ //command 25 -ERASE FLASH MEMORY
			onlyTest(false);
			// erase the whole 2Kb sector and check the successful erase
			if (ts_erase_flash_sector() != E_OK) {
				blinkLEDerror(redLed, 300);
			}
			sat_id = SATID;
		}else if(cmd_sat==SAT_COMM_GET_ALLIDS){//command 26 - GET SAT ID, MPU SAT ID, HUB ID, MPU HUB ID, LCP
			onlyTest(false);
			myHubIDrcvd = packet.sat_command[SAT_COMMSTRUCT_VALX1];
			myHubID9250 = packet.sat_command[SAT_COMMSTRUCT_VALY2];
			myLCP = dataToSend.data.nrdChannel;
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_HIDS,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
						accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],
						thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
			}
		}else if(cmd_sat==SAT_COMM_BATTERY_LV){//command 27 - get battery level
			onlyTest(false);
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				simpleReset(OUTPUT_MODE_BATL,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
						accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],
						thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
			}
		}else if(cmd_sat==SAT_COMM_RESTOREMAG){//command 28 - RESTORE FACTORY MAG BIAS
			onlyTest(false);
			flash_restore_factory_mag_calib();
		}else if(cmd_sat==SAT_COMM_FRQDIVIDER){//command 29 - SET FREQ DIVIDER
			onlyTest(false);
			int tmp = WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALX1]);

			if(tmp>0 && tmp<256)
				alternateSending_PERIOD = (uint8_t)tmp;
			else
				onlyTest(true);

			//sensorDataForInterpolation = (sensor_record_t*)malloc(alternateSending_PERIOD*sizeof(sensor_record_t));

		}else if(cmd_sat==SAT_COMM_SETQTMAGMD){//command 34 - QUAT/MAG output mode
			onlyTest(false);
			int16_t modeOutput;
			if(packet.sat_command[SAT_COMMSTRUCT_VALX1] == 1)
				modeOutput = OUTPUT_MODE_QUAT;
			else if (packet.sat_command[SAT_COMMSTRUCT_VALX1] == 2)
				modeOutput = OUTPUT_MODE_MAGO;
			else
				modeOutput = OUTPUT_MODE_QUMA;
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				simpleReset(modeOutput,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
						accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],
						thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
			}
		}else if(cmd_sat==SAT_COMM_SETQTTIMES){//COMMAND 35 - TIMESTAMP SEND
			onlyTest(false);
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				simpleReset(OUTPUT_MODE_TIME,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
					accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],
					thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
		}
		}else if(cmd_sat==SAT_COMM_IDLE_CYCLE){//command 0 - NOP

		}else if (cmd_sat == SAT_COMM_WIF_SETRTC) {
			onlyTest(false);
			if (packet.sat_command[SAT_COMMSTRUCT_VALX1] == 0) {
				/* We're starting a new recording, let's purge our cbuf.
				 * Resetting head, tail, etc. */
				cbufInit(&sensorRecordQ);
			}
			/* By default we're setting it to 0 */
			k22f_set_rtc(packet.sat_command[SAT_COMMSTRUCT_VALX1]);
			/* Led blink for visible indication that Sat receives command */
			//onlyTest(false);
		} else if(cmd_sat == SAT_COMM_WIFI_PING)
		{

		} else if(cmd_sat == SAT_COMM_STOPCALACC)
		{
			estimateAccBias_run = false;
		} else if(cmd_sat == SAT_COMM_NEWKFENABL) {
			onlyTest(false);
			if (packet.sat_command[SAT_COMMSTRUCT_VALX1] == 1)
				globals.newKF_enabled = true;
			else
				globals.newKF_enabled = false;
		}
		else
			onlyTest(true); //red led means NOT IMPLEMENTED YET

	}
}

/*******************************************************************************
 * SPI slave callback function to handle incoming Hub command from Nordic
 ******************************************************************************/
dspi_status_t from_nordic_data_sink(uint8_t sinkWord, uint32_t instance) {
	/* Sometimes Nordic awakens faster than 22F, and it already tries to
	 * set sensor active while 22F hasn't completely finished HW init.
	 * Must wait.
	 */
	uint16_t crc16, recv_crc16;

	if (!hw_init_done) {
		return kStatus_DSPI_Success;
	}
	/* In the case of unrecoverable error, do nothing */
	if (hub_err_status != hub_no_err) {
		return kStatus_DSPI_Success;
	}

	static int currSinkByte = 0;
	static unsigned int pktsyncByte = 0xF9;
	if (currSinkByte < sizeof(receivedCommand.bytes)) {
		receivedCommand.bytes[currSinkByte] = sinkWord;
	}

	if(receivedCommand.packet.sat_command[SAT_COMMSTRUCT_COMMA] != 0)
		receivedCommand.packet.sat_command[SAT_COMMSTRUCT_COMMA] = receivedCommand.packet.sat_command[SAT_COMMSTRUCT_COMMA];


	// Check if we're detecting packet-sync-byte sequence [FAFBFCFDFEFF]
	if (sinkWord >= 0xFA && sinkWord == pktsyncByte + 1) {
		pktsyncByte = sinkWord;
	} else {
		pktsyncByte = 0xF9;
	}
	currSinkByte++;
	if (currSinkByte == SPI_COMM_LENGTH) {
		if ((pktsyncByte == 0xF9) || (pktsyncByte == 0xFF)) {
			// Proper packet, not in the middle of packet-sync-byte sequence

			/* Check CRC16 */
			recv_crc16 = receivedCommand.packet.crc16;
			receivedCommand.packet.crc16 = 0;
			crc16 = crc16_compute((uint8_t*)&(receivedCommand.packet), sizeof(receivedCommand.packet), 0);

			if (crc16 == recv_crc16) {
				processCommand(receivedCommand.packet);
			}
			pktsyncByte = 0xF9;
		}
		currSinkByte = 0;
	} else if (pktsyncByte == 0xFF) {
		// Master only sends [FAFBFCFDFEFF] at the end of a packet. Must have gotten out of sync.
		printf("Resync from %d\r\n", currSinkByte);
		pktsyncByte = 0xF9;
		currSinkByte = 0;
		currSourceByte = SPI_COMM_LENGTH;

		/* Reinitialize SPI communication to Nordic */
		nrf51822_init();
	}
	return kStatus_DSPI_Success;
}

/*******************************************************************************
 * SPI slave callback function to send SPI data upon receiving msg from Nordic
 *******************************************************************************/
dspi_status_t to_nordic_data_source(uint8_t *sourceWord, uint32_t instance) {
	sat_cbuf_packet_t cbuf_data;
	err_t errCode = E_NO_MEM;

	/* Sometimes Nordic awakens faster than 22F, and it already tries to
	 * set sensor active while 22F hasn't completely finished HW init.
	 */
	/* Also, in the case of unrecoverable error, do nothing */
	if ((!hw_init_done) || (hub_err_status != hub_no_err)) {
		dataToSend_t invalidDataToSend;

		invalidDataToSend.data.timestamp = INVALID_TIMESTAMP;
		invalidDataToSend.data.sat_id = sat_id;
		invalidDataToSend.data.hub_id = myHubID;
		(*sourceWord) = invalidDataToSend.bytes[0];

		return kStatus_DSPI_Success;
	}

	if (currSourceByte == SPI_COMM_LENGTH) {
		errCode = E_NO_MEM;
		errCode = cbufPop(&sensorRecordQ, &cbuf_data);
		if (errCode != E_OK) {
			; //empty cbuf
		} else {
			memcpy(&(dataToSend.data.data[0]), &(cbuf_data.data[0]), sizeof(cbuf_data.data));
			dataToSend.data.timestamp = cbuf_data.timestamp;
			if(communicationTestCounterInitialization < 0)
				communicationTestCounter[COMMUNICATIONCOUNTER_SPIP]++;
			else {
				communicationTestCounterInitialization--;
				communicationTestCounter[COMMUNICATIONCOUNTER_SPIP]=0;
				communicationTestCounter[COMMUNICATIONCOUNTER_BUFF]=0;
			}
		}

		// If no record could be popped, or if we've been told to not send
		// anything, send previous sensor data with incremented timestamp.
		if (errCode != E_OK) {
			// sensorRecordQ is empty, send invalid timestamp
			dataToSend.data.timestamp = INVALID_TIMESTAMP;
		}
		currSourceByte = 0;

		/* Setting CRC16 */
		dataToSend.data.crc16 = 0;
		dataToSend.data.crc16 = crc16_compute((uint8_t*)&dataToSend.data, sizeof (dataToSend.data), 0);

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
void on_error(dspi_status_t error, uint32_t instance) {
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

	DSPI_DRV_SlaveInit(knRF51822SpiInstance, &nrf51822_dspiSlaveState,
			&nrf51822_dspiSlaveUserConfig);
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
	putchar((q >> 8) & 0xff);
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

/*******************************************************************************
 * Read MPU9250 raw output (Accelerometer, Gyrometer, Magnetometer, Temperature)
 ******************************************************************************/
static bool get_raw_data_sample_basic(sensor_data_t *raw_data) {
	bool ret;
	uint32_t time_start, time_now;

	//cwati
//	time_start = k22f_get_rtc();
//	while (!mpu9250_data_avail()) {
//		time_now = k22f_get_rtc();
//		if (time_now - time_start > 20) {
//			return 0;
//		}
//	}

//	mpu9250_clear_data_avail();
	ret = mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP,
			raw_data);

	return ret;
}

static bool get_raw_data_sample(uint8_t sensor_num, sensor_data_t *raw_data, sensor_record_t *data) {

	bool ret = get_raw_data_sample_basic(raw_data);

	if (!ret) {
		return ret;
	}

	fInvertGyrCal(raw_data);
	fInvertAccCal(raw_data);

	if (ret)
		if (counterDataSample[sensor_num] > 20)
			process_sensor_fusion(raw_data, data);
		else
			counterDataSample[sensor_num]++;

	return ret;
}

//#endif

/* PIVOT_3_0 WARNING: You must first call mpu9250_enable_sensor for the correct sensor
 * before you call this function,
 * to make sure that you enable the right sensor!
 */
static void sensorFusionInit() {
	/* Initializing Turing Sense sensor fusion */
	ts_fusion_init();
	mpu9250_start();

}

/* PIVOT_3_0 WARNING: You must first call mpu9250_enable_sensor for the correct sensor
 * before you call this function,
 * to make sure that you enable the right sensor!
 */
static void simpleReset(int16_t setOutputMode, int16_t accSensX, int16_t accSensY, int16_t accSensZ,
		int16_t accBiasX, int16_t accBiasY, int16_t accBiasZ, int16_t gyrBiasX, int16_t gyrBiasY,
		int16_t gyrBiasZ, float magBiasX, float magBiasY, float magBiasZ) {


	globals.alternateQuaternionMag = true;

	globals.outputMode = setOutputMode;

	globals.enable_MagBiasCalibrationAlways = SAT_BIAS_ALWAYSACTIVECALIBRATION;

	counterDataSample[mpu9250_num] = 0;

	sensorFusionInit();

	//TODO - Accelerometer calibration
	accSensInt16_X[mpu9250_num] = accSensX;
	accSensInt16_Y[mpu9250_num] = accSensY;
	accSensInt16_Z[mpu9250_num] = accSensZ;
	accBiasInt16_X[mpu9250_num] = accBiasX;
	accBiasInt16_Y[mpu9250_num] = accBiasY;
	accBiasInt16_Z[mpu9250_num] = accBiasZ;

	// Gyroscope Bias
	gyrBiasInt16_X[mpu9250_num] = gyrBiasX;
	gyrBiasInt16_Y[mpu9250_num] = gyrBiasY;
	gyrBiasInt16_Z[mpu9250_num] = gyrBiasZ;

	// Magnetometer Bias
	thisMagCal[mpu9250_num].fV[0] = magBiasX; //(float)SAT_MAGBIAS_X;
	thisMagCal[mpu9250_num].fV[1] = magBiasY; //(float)SAT_MAGBIAS_Y;
	thisMagCal[mpu9250_num].fV[2] = magBiasZ; //(float)SAT_MAGBIAS_Z;
}

static void update_battery_led (void) {
	uint16_t 	adc;
	float 		bat_level;
	float 		temp;
	uint32_t 	time_now;

	/* Check for last time we received from Nordic */
	time_now = k22f_get_rtc();
	if (time_now - last_bat_read > BATTERY_READ_MS) {
		/* Get battery level */
		adc = adc_read_1 (0);
		bat_level = readBatteryApplyFilter(adc);

		/* If fully charged or enough battery then we don't need to blink during streaming */
		if (bat_level >= 4.0) {
		  /* White for fully charged */
		  doubleBlinkLED(whiteLed);
		} else if (((bat_level >= 3.6) && (bat_level < 4.0))) {
		  /* Blue for normal, working level */
		  doubleBlinkLED(blueLed);
		} else {
		  /* If running out of battery then have to blink. */
		  /* Red means you should charge now. */
		  doubleBlinkLED(redLed);
		}
		last_bat_read = time_now;
	}
}

static void update_led (void) {

	if ((estimateGyroBias_run == false)&&(estimateAccBias_run == false)&&(tsSelfTest_run == false))
	{
		if (start_dumping == false) {
			turnOnLED(greenLed);
		} else {
			turnOnLED(turqoiseLed);
		}
	}


	if (start_dumping == false) {
		/* Update battery indicator periodically */
		update_battery_led();
	}
}

int get_sensor_num_from_cs(uint8_t cs) {
	for (uint8_t qq = 0; qq < valid_num_sensors_in_cs; qq++) {
		if (valid_cs[qq] == cs) {
			return qq;
		}
	}

	return -1;
}

static sat_cbuf_packet_t addSetMeasurement(uint16_t mult, sat_cbuf_packet_t sensor_record){
return  sensor_record;
	if (globals.outputMode == OUTPUT_MODE_COUN)
		mult = 3;

	if (mult == 0){
		sensor_record.timestamp    = (sensor_record.timestamp    + sensorDataForInterpolation.timestamp   ) / alternateSending_PERIOD;
		sensor_record.data.accel_x = (sensor_record.data.accel_x + sensorDataForInterpolation.data.accel_x) / alternateSending_PERIOD;
		sensor_record.data.accel_y = (sensor_record.data.accel_y + sensorDataForInterpolation.data.accel_y) / alternateSending_PERIOD;
		sensor_record.data.accel_z = (sensor_record.data.accel_z + sensorDataForInterpolation.data.accel_z) / alternateSending_PERIOD;
		sensor_record.data.gyro_x  = (sensor_record.data.gyro_x  + sensorDataForInterpolation.data.gyro_x ) / alternateSending_PERIOD;
		sensor_record.data.gyro_y  = (sensor_record.data.gyro_y  + sensorDataForInterpolation.data.gyro_y ) / alternateSending_PERIOD;
		sensor_record.data.gyro_z  = (sensor_record.data.gyro_z  + sensorDataForInterpolation.data.gyro_z ) / alternateSending_PERIOD;
	#if ENABLE_MAG
		sensor_record.data.mag_x   = (sensor_record.data.mag_x   + sensorDataForInterpolation.data.mag_x  ) / alternateSending_PERIOD;
		sensor_record.data.mag_y   = (sensor_record.data.mag_y   + sensorDataForInterpolation.data.mag_y  ) / alternateSending_PERIOD;
		sensor_record.data.mag_z   = (sensor_record.data.mag_z   + sensorDataForInterpolation.data.mag_z  ) / alternateSending_PERIOD;
	#endif
		sensor_record.data.quat_w  = (sensorDataForInterpolation.data.quat_w); //send previous value of QUAT
		sensor_record.data.quat_x  = (sensorDataForInterpolation.data.quat_x);
		sensor_record.data.quat_y  = (sensorDataForInterpolation.data.quat_y);
		sensor_record.data.quat_z  = (sensorDataForInterpolation.data.quat_z);

		if(globals.outputMode == OUTPUT_MODE_TIME){
				// send timestamp in quaternion
				uint16_t * timestramp = (int16_t*) &(sensor_record.timestamp);

				sensor_record.data.quat_w = (int16_t)(CPI_QUATERNION16_BIAS_MULTIPLIER*CPI_QUATERNION16_TIMESTA_COMMAND);
				sensor_record.data.quat_x = *timestramp;
				sensor_record.data.quat_y = *(timestramp+1);
				sensor_record.data.quat_z = 0;
		}
	}

	if(mult <= 1){//0 reset or 1 add measurement
		sensorDataForInterpolation.timestamp    = mult * (sensorDataForInterpolation.timestamp    + sensor_record.timestamp);
		sensorDataForInterpolation.data.accel_x = mult * (sensorDataForInterpolation.data.accel_x + sensor_record.data.accel_x);
		sensorDataForInterpolation.data.accel_y = mult * (sensorDataForInterpolation.data.accel_y + sensor_record.data.accel_y);
		sensorDataForInterpolation.data.accel_z = mult * (sensorDataForInterpolation.data.accel_z + sensor_record.data.accel_z);
		sensorDataForInterpolation.data.gyro_x  = mult * (sensorDataForInterpolation.data.gyro_x  + sensor_record.data.gyro_x);
		sensorDataForInterpolation.data.gyro_y  = mult * (sensorDataForInterpolation.data.gyro_y  + sensor_record.data.gyro_y);
		sensorDataForInterpolation.data.gyro_z  = mult * (sensorDataForInterpolation.data.gyro_z  + sensor_record.data.gyro_z);
#if ENABLE_MAG
		sensorDataForInterpolation.data.mag_x   = mult * (sensorDataForInterpolation.data.mag_x   + sensor_record.data.mag_x);
		sensorDataForInterpolation.data.mag_y   = mult * (sensorDataForInterpolation.data.mag_y   + sensor_record.data.mag_y);
		sensorDataForInterpolation.data.mag_z   = mult * (sensorDataForInterpolation.data.mag_z   + sensor_record.data.mag_z);
#endif
		sensorDataForInterpolation.data.quat_w  =  (sensor_record.data.quat_w);
		sensorDataForInterpolation.data.quat_x  =  (sensor_record.data.quat_x);
		sensorDataForInterpolation.data.quat_y  =  (sensor_record.data.quat_y);
		sensorDataForInterpolation.data.quat_z  =  (sensor_record.data.quat_z);
	}
		return sensor_record;
}

/*******************************************************************************
 * Main routine
 ******************************************************************************/
int main(void) {

	sensor_data_t raw_data;			// Raw output of MPU9250 (accel, gyro, mag)
	sat_cbuf_packet_t sensor_record;// Timestamp + sensor data (raw + sensor-fusion output)
	uint32_t current_ts10, time_now;
	uint8_t	whoami = 0xff;
	bool is_sleeping = false, retbool;
	uint8_t sensor_num, cs;
	bool has_data;

#if DO_SELFTEST
	mpu9250_self_test_results_t factory_st;
	mpu9250_average_data_t avg_normal, avg_selftest;
#endif /* DO_SELFTEST */

	// Initialize values
	for (sensor_num = 0; sensor_num < MAX_SENSORS_IN_CS; sensor_num++) {
		accSensInt16_X[sensor_num] = (int16_t)(1.0F/MPU9250_GPERCOUNT);
		accSensInt16_Y[sensor_num] = (int16_t)(1.0F/MPU9250_GPERCOUNT);
		accSensInt16_Z[sensor_num] = (int16_t)(1.0F/MPU9250_GPERCOUNT);
	}

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

	// read ID from flash memory, otherwise consider the MPU9250 SATID
	// read all the calib parameters stored in the flash
	flash_load_satID_calib();

	#if !USE_FLASH
		myHubID = DEFAULT_HUB_ID;
		dataToSend.data.hub_id = myHubID;
		dataToSend.data.sat_id = sat_id;
		dataToSend.data.nrdChannel = INITIAL_LCP;
	#endif

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

	/* Initialize sensors.  Check how many sensors are available. */
	mpu9250_init_cs_gpio();
	mpu9250_init_all_pins(); /* all pins except for CS */

	/* Go through all possible CS
	 *
	 * It is possible that sensor_num is not the same with cs.  For example,
	 * we can have:
	 * sensor0 - cs0
	 * sensor1 - cs2
	 * etc...
	 * */
	valid_num_sensors_in_cs = 0;
//	for (cs = 0; cs < MAX_SPI_CS; cs++) {
	for(cs = 0; cs < MAX_SPI_CS; cs++) {

		mpu9250_detect_sensor(cs, &whoami);

		/* Valid sensor detected */
		if (whoami == MPU9250_VALID_WHO_AM_I) {

			valid_cs[valid_num_sensors_in_cs] = cs;

			mpu9250_enable_sensor(valid_num_sensors_in_cs);

			/* Turn on LED for indicator of a sensor getting found */
			turnOnLED(greenLed);

			mpu9250_init_regs();

			OSA_TimeDelay(800);
			turnOnLED(offLed);
			OSA_TimeDelay(500);

			valid_num_sensors_in_cs++;
			if (valid_num_sensors_in_cs == MAX_SENSORS_IN_CS) {
				/* Can't have more sensors, break... */
				break;
			}
		}
	}
//
//
//	if (!mpu9250_init()) {
//		printf("error initializing MPU9250\r\n");
//		blinkLEDerror(redLed, 3);
//		hub_err_status = hub_hw_err3;
//	}

#if DO_SELFTEST
	// We'd like to test both the accel and gyro, but as of 2015-07-23, there
	// is a problem with taking gyro self-test data which results in the
	// data being out of range, as if the accelerometer is clipping at
	// full-scale range.
	// NOTE: also or in  ST_FLAG_DEBUG_PRINTF to diagnose self-test failures
	int st_flags = ST_FLAG_GYRO | ST_FLAG_ACCEL;
	OSA_TimeDelay(100);

	if (!mpu9250_get_factory_self_test_results(&factory_st)) {
//		blinkLEDerror(yellowLed, 1);
		blinkLEDerror(greenLed, 1);
		hub_err_status = hub_inv_err1;
	}

	if (st_flags & ST_FLAG_DEBUG_PRINTF) {
		for (int axis = X; axis <= Z; axis++)
			printf("factory self-test gyro %c:  %d\r\n", 'X' + axis,
					factory_st.gyro[axis]);
		for (int axis = X; axis <= Z; axis++)
			printf("factory self-test accel %c: %d\r\n", 'X' + axis,
					factory_st.accel[axis]);
	}

	printf("starting self-test\r\n");

	if (!mpu9250_self_test(st_flags, &avg_normal, &avg_selftest)) {
		printf("error collecting self-test data samples\r\n");
		blinkLEDerror(yellowLed, 2);
		hub_err_status = hub_inv_err2;
	}
	printf("self-test completed\r\n");

	if ((st_flags & ST_FLAG_DEBUG_PRINTF) && (st_flags & ST_FLAG_ACCEL)) {
		for (int axis = X; axis <= Z; axis++)
			printf("normal accel %c average:  %d\r\n", 'X' + axis,
					avg_normal.accel[axis]);
		for (int axis = X; axis <= Z; axis++)
			printf("self-test accel %c average:  %d\r\n", 'X' + axis,
					avg_selftest.accel[axis]);
		for (int axis = X; axis <= Z; axis++)
			printf("self-test accel %c response:  %d\r\n", 'X' + axis,
					avg_selftest.accel[axis] - avg_normal.accel[axis]);
	}

	if ((st_flags & ST_FLAG_DEBUG_PRINTF) && (st_flags & ST_FLAG_GYRO)) {
		for (int axis = X; axis <= Z; axis++)
			printf("normal gyro %c average:   %d\r\n", 'X' + axis,
					avg_normal.gyro[axis]);
		for (int axis = X; axis <= Z; axis++)
			printf("self-test gyro %c average:   %d\r\n", 'X' + axis,
					avg_selftest.gyro[axis]);
		for (int axis = X; axis <= Z; axis++)
			printf("self-test gyro %c response:   %d\r\n", 'X' + axis,
					avg_selftest.gyro[axis] - avg_normal.gyro[axis]);
	}

	uint32_t status = 0;  // assume OK
	OSA_TimeDelay(100);

	status = mpu9250_check_selftest_results(st_flags, &factory_st, &avg_normal,
			&avg_selftest);
	if (status) {
		printf("self-test results out of tolerance FAILED: %u\r\n", status);
		blinkLEDerror(purpleLed, 3);
		hub_err_status = hub_inv_err3;
	}
	printf("self-test results good\r\n");
#endif // DO_SELFTEST

	for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
		mpu9250_enable_sensor(sensor_num);
		Fusion_Init1(&accGyrKFpar[sensor_num],&UVRH[sensor_num],STDG,STDA,CA,CB,PO,XDIM,YDIM);
		simpleReset(OUTPUT_MODE_QUMA,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
				accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num],
				thisMagCal[sensor_num].fV[0],thisMagCal[sensor_num].fV[1],thisMagCal[sensor_num].fV[2]);
	}

	/* Reset RTC to 0 */
	k22f_set_rtc(0);
	last_bat_read = 0;

	/* Initializing satellite id */
	//dataToSend.data.sat_id = sat_id;
	//dataToSend.data.nrdChannel=INITIAL_LCP; //CF: DO NOT MODIFY - set an invalid channel for Nordic, in this way Nordic set the default channel
	//dataToSend.data.hub_id = myHubID;

	hw_init_done = true;

	// Initialize nRF51822 (configure SPI0 slave, data structures, etc.)
	printf("\r\nInitializing Nordic nRF51822\r\n");
	turnOnLED(offLed);
	//nrf51822_swdio_reset();

	nrf51822_init();

	// Done with all initializations, turn on GREEN LED, entering Main loop
	turnOnLED(greenLed);

	/* CWATI: You cannot put a long delay (100+ ms) right before Flash Erase.
	 * I notice that if you put 2-3 seconds of OSA_TimeDelay right before ts_erase_flash_sector, then
	 * the code will jump back to bootloader.
	 */

	// Main loop
	while (true) {
		update_led();

		if (tsSelfTest_run == true)
		{
			tsSelftest();
			tsSelfTest_run = false;
		}

		if (estimateGyroBias_run == true) {
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				estimateGyroBias(sensor_num);
			}
			estimateGyroBias_run = false;
		}

		if (estimateAccBias_run == true) {
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);
				estimateAccBias(sensor_num);
			}
			estimateAccBias_run = false;
		}

		if (start_dumping == true) {
			memset(&sensor_record, 0, sizeof(sensor_record));

			has_data = false;
			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				mpu9250_enable_sensor(sensor_num);

				retbool = get_raw_data_sample(sensor_num, &raw_data, &sensor_record.data[sensor_num]);
				if (!retbool) {
					continue;
				} else {
					has_data = true;
				}
			}

			if (has_data) {
				sensor_record.timestamp = k22f_get_rtc();

#if PACE_SENSOR_OUTPUT
				if (alternateSending == alternateSending_PERIOD) {
					/* We want to discard older data */
					if (cbufIsFull(&sensorRecordQ)) {
						cbufPopDiscard(&sensorRecordQ);
					}

					if(globals.outputMode == OUTPUT_MODE_COUN && communicationTestCounterInitialization < 0)
						communicationTestCounter[COMMUNICATIONCOUNTER_BUFF]++;

					if (cbufPush(&sensorRecordQ, addSetMeasurement(0, sensor_record)) != E_OK) {
						printf("error inserting to sensorRecordQ\r\n");
						blinkLEDerror(yellowLed, 3);
					}
					alternateSending = 1;
					globals.alternateQuaternionMag = !globals.alternateQuaternionMag;
				} else if (alternateSending < alternateSending_PERIOD) {
					addSetMeasurement(1, sensor_record);
					alternateSending++;
				} else {
					/* alternateSending is bigger than TS_SENDING_PERIOD!
					 * This should not have happened.
					 */
					addSetMeasurement(0, sensor_record);
					alternateSending = 1;
				}
#else
				/* We want to discard older data */
				if (cbufIsFull(&sensorRecordQ)) {
					cbufPopDiscard(&sensorRecordQ);
				}

				if (cbufPush(&sensorRecordQ, sensor_record) != E_OK) {
					printf("error inserting to sensorRecordQ\r\n");
					blinkLEDerror(yellowLed, 3);
				}
#endif /* PACE_SENSOR_OUTPUT */
			}
		} else {
			OSA_TimeDelay(5); /* ms */
		}

		/* Check for last time we received from Nordic */
		time_now = k22f_get_rtc();
		if (time_now > last_nordic_comm) {
			if ((time_now - last_nordic_comm) > MAX_NORDIC_QUIET) {

				//nrf51822_swdio_reset();
				nrf51822_init();


				last_nordic_comm = time_now;
			}
		}

	}
	return 0;  // will never reach this

} // end main()

