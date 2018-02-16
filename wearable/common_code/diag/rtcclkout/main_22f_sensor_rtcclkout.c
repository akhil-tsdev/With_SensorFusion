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
/*******************************************************************************
 * SDK Include Files
 ******************************************************************************/
#include "fsl_device_registers.h"
#include "fsl_gpio_common.h"	/* RTC pin */
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
#include "crc16.h"
#include "main_22f_sensor_rtcclkout.h"

#if USE_BOOTLOADER
/* TODO eventually using bootloader will be the default method. */
#include "ts_flash.h"
#endif /* USE_BOOTLOADER */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEBUG			0
#define OUTPUT_3D_CUBE	0
static uint32_t sat_id = 249;

/*******************************************************************************
 * BIAS DETECTION AND BIAS SETTING *** NOTE! VALID FOR SINGOL MEMS
 ******************************************************************************/
#define SAT_BIAS_SENDING 	false
#define SAT_MAGBIAS_X 	 0
#define SAT_MAGBIAS_Y    0
#define SAT_MAGBIAS_Z    0

#define Z_DOWN 	0
#define Z_UP   	1
#define Y_DOWN 	2
#define Y_UP 	3
#define X_DOWN 	4
#define X_UP	5

#define SAT_BIAS_ALWAYSACTIVECALIBRATION 	false //Even if set the fixed values of BIAS (with SAT_BIAS_SENDING=false), calibration of the magnetometer remains active. In this case the calibration values will be recalculated

#define USE_HWE_DEBUG_CONFIGURATION 1
static bool read_sensors_launch_fusion (sensor_data_t *raw_data, sensor_record_t* data);

static bool get_raw_data_sample(sensor_data_t *raw_data, sensor_record_t *data);
static bool get_raw_data_sample_basic(sensor_data_t *raw_data);

#if PACE_SENSOR_OUTPUT
static uint8_t alternateSending = TS_SENDING_PERIOD;
#endif
hub_err_t hub_err_status = hub_no_err;

uint32_t reset_rtc_diff = 0; /* Elapsed time before resetting RTC happens.*/

static volatile bool hw_init_done = false;

static bool estimateGyroBias_run = false;
static bool estimateAccBias_run = false;
int16_t gyrBiasInt16_X = 0;
int16_t gyrBiasInt16_Y = 0;
int16_t gyrBiasInt16_Z = 0;
int16_t accSensInt16_X = (int16_t)(1.0F/MPU9250_GPERCOUNT);
int16_t accSensInt16_Y = (int16_t)(1.0F/MPU9250_GPERCOUNT);;
int16_t accSensInt16_Z = (int16_t)(1.0F/MPU9250_GPERCOUNT);;
int16_t accBiasInt16_X = 0;
int16_t accBiasInt16_Y = 0;
int16_t accBiasInt16_Z = 0;

//debug
uint32_t ts1, ts2, ts3;

static uint32_t last_ts10 = 0;
static uint32_t last_sec, last_tpr;
static uint32_t last_total_ms;	//debug cw todo
uint32_t ts[500] = { 0 }, ts_cnt = 0;

/* Heartbeat check for Nordic communication */
uint32_t last_nordic_comm;
#define MAX_NORDIC_QUIET	2500 /* in ms */

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
static volatile bool sensor_active = false;
static volatile bool sensor_sending = false;

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
 	OSA_TimeDelay(100);
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

bool estimateGyroBias()
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

		gyrBiasInt16_X = (int16_t) (biasFloat_X/MPU9250_DEGPERSECPERCOUNT);
		gyrBiasInt16_Y = (int16_t) (biasFloat_Y/MPU9250_DEGPERSECPERCOUNT);
		gyrBiasInt16_Z = (int16_t) (biasFloat_Z/MPU9250_DEGPERSECPERCOUNT);

	turnOnLED(offLed);
	return true;

}


void fInvertGyrCal(sensor_data_t * raw_data)
{
	raw_data->gyro[0] -= gyrBiasInt16_X;
	raw_data->gyro[1] -= gyrBiasInt16_Y;
	raw_data->gyro[2] -= gyrBiasInt16_Z;
}

void fInvertAccCal(sensor_data_t * raw_data)
{
	float tmpX = (float)(raw_data->accel[0] - accBiasInt16_X)*MPU9250_GPERCOUNT;
	float tmpY = (float)(raw_data->accel[1] - accBiasInt16_Y)*MPU9250_GPERCOUNT;
	float tmpZ = (float)(raw_data->accel[2] - accBiasInt16_Z)*MPU9250_GPERCOUNT;

	float Sx = (float)accSensInt16_X*MPU9250_GPERCOUNT;
	float Sy = (float)accSensInt16_Y*MPU9250_GPERCOUNT;
	float Sz = (float)accSensInt16_Z*MPU9250_GPERCOUNT;

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



bool estimateAccBias()
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


	// Acquire data matrix
	for (int i = 0; i < 6; i++)
	{
		waitForNextPosition(i);

		turnOnLED(turqoiseLed);

		int c = 0;

		data[0][i] = 0.0f;
		data[1][i] = 0.0f;
		data[2][i] = 0.0f;

		while (c < N)
		{
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
	accSensInt16_X = (int16_t) (x[0]/MPU9250_GPERCOUNT);
	accSensInt16_Y = (int16_t) (x[1]/MPU9250_GPERCOUNT);
	accSensInt16_Z = (int16_t) (x[2]/MPU9250_GPERCOUNT);
	accBiasInt16_X = (int16_t) (x[3]/MPU9250_GPERCOUNT);
	accBiasInt16_Y = (int16_t) (x[4]/MPU9250_GPERCOUNT);
	accBiasInt16_Z = (int16_t) (x[5]/MPU9250_GPERCOUNT);

		//turnOnLED(offLed);
		return true;




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
	last_nordic_comm = k22f_get_rtc();

	uint32_t cmd = packet.sat_command[SAT_COMMSTRUCT_SATID],
			 timeStampOld = 0;


	if (cmd & SAT_SPI_START) {
		timeStampOld = k22f_get_rtc();
		sensor_active = true;
	} else if(k22f_get_rtc() - timeStampOld > 150) { //CF to be sure it was commanded the stop dumping
		if(sensor_active){
			simpleReset(OUTPUT_MODE_QUAT,accSensInt16_X,accSensInt16_Y,accSensInt16_Z,accBiasInt16_X,
					accBiasInt16_Y,accBiasInt16_Z,gyrBiasInt16_X,gyrBiasInt16_Y,gyrBiasInt16_Z,
					thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2]); //CF
			sensor_active = false;
		}
	}

	if (cmd & SAT_SPI_SET_RTC) {
		if (packet.sat_command[SAT_COMMSTRUCT_VALX1] == 0) {
			/* We're starting a new recording, let's purge our cbuf.
			 * Resetting head, tail, etc. */
			cbufInit(&sensorRecordQ);
		}
		/* By default we're setting it to 0 */
		k22f_set_rtc(packet.sat_command[SAT_COMMSTRUCT_VALX1]);

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
		}else if(cmd_sat==SAT_COMM_SETBTCOMCH){//command 2 - set BT change comunication channel - used by Nordic; here only confirmation
			turnOnLED(purpleLed);
			dataToSend.data.nrdChannel=(packet.sat_command[SAT_COMMSTRUCT_VALY2]?packet.sat_command[SAT_COMMSTRUCT_VALX1]:dataToSend.data.nrdChannel);
			onlyTest(!packet.sat_command[SAT_COMMSTRUCT_VALY2]);
			onlyTest(!packet.sat_command[SAT_COMMSTRUCT_VALZ3]);
		}else if(cmd_sat==SAT_COMM_TESTCOMSTC){//command 3 - set COMMUNICATION TEST
			onlyTest(false);
			communicationTestCounter=1;
			simpleReset(OUTPUT_MODE_COUN,accSensInt16_X,accSensInt16_Y,accSensInt16_Z,accBiasInt16_X,
					accBiasInt16_Y,accBiasInt16_Z,gyrBiasInt16_X,gyrBiasInt16_Y,gyrBiasInt16_Z,
					thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2]);
		}else if(cmd_sat==SAT_COMM_SETWFCOMCH){//command 4 - set WIFI change comunication channel - used by HUB 22F; here only confirmation
			turnOnLED(purpleLed);
			onlyTest(!packet.sat_command[SAT_COMMSTRUCT_VALY2]);
			onlyTest(!packet.sat_command[SAT_COMMSTRUCT_VALZ3]);
		}else if(cmd_sat==SAT_COMM_SETBTCOMAU && !sensor_active){//command 5 - set BT change comunication channel automatically - generated from SAT NORDIC when non receive packets from HUB
			turnOnLED(purpleLed);
			OSA_TimeDelay(50);
			dataToSend.data.nrdChannel++;
			if(dataToSend.data.nrdChannel > packet.sat_command[SAT_COMMSTRUCT_VALX1]) //max BTT channel
				dataToSend.data.nrdChannel=0;
		}else if(cmd_sat==SAT_COMM_MBIASSENDG){//command 10 - MAG BIAS SENDING MODE
			onlyTest(false);
			simpleReset(OUTPUT_MODE_BMAG,accSensInt16_X,accSensInt16_Y,accSensInt16_Z,accBiasInt16_X,
					accBiasInt16_Y,accBiasInt16_Z,gyrBiasInt16_X,gyrBiasInt16_Y,gyrBiasInt16_Z,
					thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2]);
			globals.disable_magCalibrationAlways = false;
		}else if(cmd_sat==SAT_COMM_NORMLSENDG){//command 11 - NORMAL SENDING MODE
//			onlyTest(false);
			simpleReset(OUTPUT_MODE_QUAT,accSensInt16_X,accSensInt16_Y,accSensInt16_Z,accBiasInt16_X,
					accBiasInt16_Y,accBiasInt16_Z,gyrBiasInt16_X,gyrBiasInt16_Y,gyrBiasInt16_Z,
					thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2]);
		}else if(cmd_sat==SAT_COMM_SETMAGBIAS){//command 12 - NORMAL SENDING MODE SET BIAS
			onlyTest(false);
			simpleReset(OUTPUT_MODE_QUAT,accSensInt16_X,accSensInt16_Y,accSensInt16_Z,accBiasInt16_X,
					accBiasInt16_Y,accBiasInt16_Z,gyrBiasInt16_X,gyrBiasInt16_Y,gyrBiasInt16_Z,
					((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALX1]))/1000.0F,
					((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALY2]))/1000.0F,
					((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALZ3]))/1000.0F); //CF

		}else if(cmd_sat==SAT_COMM_MSTBIASENDG){//command 13 - MAG BIAS SETTED SENDING MODE
			onlyTest(false);
			simpleReset(OUTPUT_MODE_BMAG,accSensInt16_X,accSensInt16_Y,accSensInt16_Z,accBiasInt16_X,
					accBiasInt16_Y,accBiasInt16_Z,gyrBiasInt16_X,gyrBiasInt16_Y,gyrBiasInt16_Z,
					thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2]);
			globals.disable_magCalibrationAlways = true;
		}else if(cmd_sat==SAT_COMM_B_GYRO_EST){// command 14 - GYR BIAS ESTIMATION
			estimateGyroBias_run = true;
		}else if(cmd_sat==SAT_COMM_B_ACCE_EST){// command 15 - ACC BIAS ESTIMATION
			estimateAccBias_run = true;
		}else if(cmd_sat==SAT_COMM_BACCGYRSND){// command 16 - GYR & ACC BIAS SENDING
			onlyTest(false);
			simpleReset(OUTPUT_MODE_BGYR,accSensInt16_X,accSensInt16_Y,accSensInt16_Z,accBiasInt16_X,
					accBiasInt16_Y,accBiasInt16_Z,gyrBiasInt16_X,gyrBiasInt16_Y,gyrBiasInt16_Z,
					thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2]);

		}else if(cmd_sat==SAT_COMM_GETRELEASE){// command 18 - write release version
			onlyTest(false);
			simpleReset(OUTPUT_MODE_VERS,accSensInt16_X,accSensInt16_Y,accSensInt16_Z,accBiasInt16_X,
					accBiasInt16_Y,accBiasInt16_Z,gyrBiasInt16_X,gyrBiasInt16_Y,gyrBiasInt16_Z,
					thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2]);

		}else if(cmd_sat==SAT_COMM_SETACCSENS){//command 19 - SET ACC SENSITIVITY
			onlyTest(false);

			float accSensX = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALX1]))/1000.0F;
			float accSensY = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALY2]))/1000.0F;
			float accSensZ = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALZ3]))/1000.0F;

			simpleReset(OUTPUT_MODE_QUAT,(int16_t)(accSensX/MPU9250_GPERCOUNT),(int16_t)(accSensY/MPU9250_GPERCOUNT),
					(int16_t)(accSensZ/MPU9250_GPERCOUNT),accBiasInt16_X,accBiasInt16_Y,accBiasInt16_Z,gyrBiasInt16_X,
					gyrBiasInt16_Y,gyrBiasInt16_Z,thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2]);

		}else if(cmd_sat==SAT_COMM_SETACCBIAS){//command 20 - SET ACC BIAS
			onlyTest(false);

			float accBiasX = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALX1]))/1000.0F;
			float accBiasY = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALY2]))/1000.0F;
			float accBiasZ = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALZ3]))/1000.0F;

			simpleReset(OUTPUT_MODE_QUAT,accSensInt16_X,accSensInt16_Y,accSensInt16_Z,(int16_t)(accBiasX/MPU9250_GPERCOUNT),
					(int16_t)(accBiasY/MPU9250_GPERCOUNT),(int16_t)(accBiasZ/MPU9250_GPERCOUNT),gyrBiasInt16_X,
					gyrBiasInt16_Y,gyrBiasInt16_Z,thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2]);

		}else if(cmd_sat==SAT_COMM_SETGYRBIAS){//command 20 - SET ACC BIAS
			onlyTest(false);

			float gyrBiasX = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALX1]))/1000.0F;
			float gyrBiasY = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALY2]))/1000.0F;
			float gyrBiasZ = ((float)WAgiveIntoFromUint(packet.sat_command[SAT_COMMSTRUCT_VALZ3]))/1000.0F;

			simpleReset(OUTPUT_MODE_QUAT,accSensInt16_X,accSensInt16_Y,accSensInt16_Z,accBiasInt16_X,accBiasInt16_Y,
					accBiasInt16_Z,(int16_t)(gyrBiasX/MPU9250_DEGPERSECPERCOUNT),(int16_t)(gyrBiasY/MPU9250_DEGPERSECPERCOUNT),
					(int16_t)(gyrBiasZ/MPU9250_DEGPERSECPERCOUNT),thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2]);
		}


		else
			onlyTest(true); //red led means NOT IMPLEMENTED YET
		}

	if ((estimateGyroBias_run == false)&(estimateAccBias_run == false))
	{
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

		/* Power cycle Nordic */
		nrf51822_swdio_reset();

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
	 * Must wait.
	 */
	/* Also, in the case of unrecoverable error, do nothing */
	if ((!hw_init_done) || (hub_err_status != hub_no_err)) {
		dataToSend_t invalidDataToSend;

		invalidDataToSend.data.timestamp = INVALID_TIMESTAMP;
		invalidDataToSend.data.sat_id = sat_id;
		(*sourceWord) = invalidDataToSend.bytes[0];

		return kStatus_DSPI_Success;
	}

	if (currSourceByte == SPI_COMM_LENGTH) {
		errCode = E_NO_MEM;
		errCode = cbufPop(&sensorRecordQ, &cbuf_data);
		if (errCode != E_OK) {
			; //empty cbuf
		} else {
			dataToSend.data.data = cbuf_data.data;
			dataToSend.data.timestamp = cbuf_data.timestamp;
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

//#if PACE_SENSOR_OUTPUT
///*******************************************************************************
// * Read MPU9250 raw output (Accelerometer, Gyrometer, Magnetometer, Temperature)
// * Will return only after "time_ms" has elapsed.
// * Otherwise it will wait until we get data before we return.
// *
// * Never return false!! Always return true!!
// * Might loop forever if invensense never gives data!
// ******************************************************************************/
//static bool get_raw_data_sample_w_freq (sensor_data_t *raw_data, uint32_t time_ms,
//		sensor_record_t* data)
//{
//	uint8_t			got_data = false;
//	bool			ret;
//
//	k22f_set_elapsed_rtc();
//
//	ts3 = k22f_get_elapsed_rtc();//debug todo
//	while (ts3 < time_ms) {
//		if (mpu9250_data_avail()) {
//
//			/* Clear it first to avoid race condition */
//			mpu9250_clear_data_avail();
//			// Raw output of MPU9250 (accel, gyro, mag)
//			ret = mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP, raw_data);
//			if (ret) {
//				got_data = true;
//				process_sensor_fusion(raw_data, data);
//			}
//		}
//
//		ts3 = k22f_get_elapsed_rtc();//debug todo
//	}
//
//	if (ts3 >= time_ms) {
//		if (got_data) {
//			return true;
//		} else {
//			while (! mpu9250_data_avail())
//				;
//			/* Clear it first to avoid race condition */
//			mpu9250_clear_data_avail();
//			// Raw output of MPU9250 (accel, gyro, mag)
//			ret = mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP, raw_data);
//			if (ret) {
//				got_data = true;
//				process_sensor_fusion(raw_data, data);
//			}
//
//			return true;
//		}
//	}
//
//	return true;
//}
//#else
//cwati test todo

/*******************************************************************************
 * Read MPU9250 raw output (Accelerometer, Gyrometer, Magnetometer, Temperature)
 ******************************************************************************/
static bool get_raw_data_sample_basic(sensor_data_t *raw_data) {
	bool ret;

	while (!mpu9250_data_avail())
		;
	mpu9250_clear_data_avail();
	ret = mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP,
			raw_data);

	return ret;
}

static bool get_raw_data_sample(sensor_data_t *raw_data, sensor_record_t *data) {

	bool ret = get_raw_data_sample_basic(raw_data);

	fInvertGyrCal(raw_data);
	fInvertAccCal(raw_data);

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

static void simpleReset(int16_t setOutputMode, int16_t accSensX, int16_t accSensY, int16_t accSensZ,
		int16_t accBiasX, int16_t accBiasY, int16_t accBiasZ, int16_t gyrBiasX, int16_t gyrBiasY,
		int16_t gyrBiasZ, float magBiasX, float magBiasY, float magBiasZ) {

	sensorFusionInit();

	globals.outputMode = setOutputMode;

	globals.enable_MagBiasCalibrationAlways = SAT_BIAS_ALWAYSACTIVECALIBRATION;

	//TODO - Accelerometer calibration
	accSensInt16_X = accSensX;
	accSensInt16_Y = accSensY;
	accSensInt16_Z = accSensZ;
	accBiasInt16_X = accBiasX;
	accBiasInt16_Y = accBiasY;
	accBiasInt16_Z = accBiasZ;

	// Gyroscope Bias
	gyrBiasInt16_X = gyrBiasX;
	gyrBiasInt16_Y = gyrBiasY;
	gyrBiasInt16_Z = gyrBiasZ;

	// Magnetometer Bias
	thisMagCal.fV[0] = magBiasX; //(float)SAT_MAGBIAS_X;
	thisMagCal.fV[1] = magBiasY; //(float)SAT_MAGBIAS_Y;
	thisMagCal.fV[2] = magBiasZ; //(float)SAT_MAGBIAS_Z;
}

/*******************************************************************************
 * Main routine
 ******************************************************************************/
int main(void) {
	sensor_data_t raw_data;			// Raw output of MPU9250 (accel, gyro, mag)
	sat_cbuf_packet_t sensor_record;// Timestamp + sensor data (raw + sensor-fusion output)
	uint32_t current_ts10, time_now;
	bool is_sleeping = false, retbool;
	volatile uint32_t sopt2, sopt2a, rtccr, rtccr1;
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

	/* RTC clockout init */
	// PTC3 alt5
    PORT_HAL_SetMuxMode(g_portBaseAddr[2],3u,kPortMuxAlt5);
	// PTE0
    PORT_HAL_SetMuxMode(g_portBaseAddr[4],0u,kPortMuxAlt7);

    // Set RTC_CR CLKO=0, OSCE=1
	// Enable 32.768 kHz oscillator
	BW_RTC_CR_OSCE(RTC_BASE, 1);
	// Wait 100ms for oscillator output to settle down
	OSA_TimeDelay(100);
	// Output clock to other peripherals
	BW_RTC_CR_CLKO(RTC_BASE, 0);

	//SIM->SOPT2 CLKOUTSEL='101'b, RTCCLKOUTSEL=1
	SIM->SOPT2 &= ~(SIM_SOPT2_CLKOUTSEL_MASK);
	SIM->SOPT2 |= (0x5 << SIM_SOPT2_CLKOUTSEL_SHIFT);

	SIM->SOPT2 |= SIM_SOPT2_RTCCLKOUTSEL_MASK;

	// Main loop
	while (true) {

		sopt2 = SIM->SOPT2;
		rtccr = RTC->CR;

		sopt2a = sopt2;
		rtccr1 = rtccr;
		OSA_TimeDelay(1000); /* ms */
	}
	return 0;  // will never reach this

} // end main()
