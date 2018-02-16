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

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "fsl_device_registers.h"
#include "fsl_os_abstraction.h"

#include "board.h"
#include "mpu9250.h"

#include "../ossf/build.h"
#include "../ossf/tasks.h"

static int i = 0;

static void print_sensor_data(sensor_data_t *sensor_data)
{
	static int ctr=0;

	if (++ctr != (GYRO_ACCEL_RATE/5))
		return;
	ctr = 0;
	printf("Time %6lu  ", sensor_data->timestamp);
	if (sensor_data->valid_sensors & SENSOR_TEMP)
		printf("Temp %d  ", (int)roundf(sensor_data->temp));
	if (sensor_data->valid_sensors & SENSOR_ACCEL)
		printf("Accel (%6d, %6d, %6d)  ", sensor_data->accel[X], sensor_data->accel[Y], sensor_data->accel[Z]);
	if (sensor_data->valid_sensors & SENSOR_GYRO)
		printf("Gyro (%6d, %6d, %6d)  ", sensor_data->gyro[X], sensor_data->gyro[Y], sensor_data->gyro[Z]);
	if (sensor_data->valid_sensors & SENSOR_MAG)
		printf("Mag (%6d, %6d, %6d)  ", sensor_data->mag[X], sensor_data->mag[Y], sensor_data->mag[Z]);
	printf("\r\n");
}

#if 0
// for floating point printf, the linker flags must include "-u _printf_float"
static void print_fusion_output(void)
{
	printf("roll %f, pitch %f, yaw %f, compass %f, tilt from vertical %f\r\n",
		   thisSV_9DOF_GBY_KALMAN.fPhiPl,	// roll (deg)
		   thisSV_9DOF_GBY_KALMAN.fThePl,	// pitch (deg)
		   thisSV_9DOF_GBY_KALMAN.fPsiPl,	// yaw (deg)
		   thisSV_9DOF_GBY_KALMAN.fRhoPl,	// compass (deg)
		   thisSV_9DOF_GBY_KALMAN.fChiPl);	// tilt from vertical (deg)
}
#endif

static void put_float(float f)
{
	int32_t q;

	q = (int32_t) (f * (1<<30));  // Python demo uses fixed point +-1.30 bits

	putchar((q >> 24) & 0xff);
	putchar((q >> 16) & 0xff);
	putchar((q >> 8)  & 0xff);
	putchar(q & 0xff);
}

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

static bool get_sensor_data_sample(sensor_data_t *sensor_data)
{
	while (! mpu9250_data_avail())
		;
	mpu9250_clear_data_avail();
	return mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP, sensor_data);
}

#define DO_SELFTEST 1

int main(void)
{
	sensor_data_t sensor_data;
	mpu9250_self_test_results_t factory_st;
	mpu9250_average_data_t avg_normal, avg_selftest;

	hardware_init();
	OSA_Init();
#if (EVAL_BOARD)
    configure_uart_pins(BOARD_DEBUG_UART_INSTANCE);
    dbg_uart_init();

    printf("\r\nOpen Source Sensor Fusion with Invensense MPU-9250\r\n");
#endif
	if (! mpu9250_init())
	{
		printf("error initializing MPU9250\r\n");
		while (true)
			;
	}

#if DO_SELFTEST
	// We'd like to test both the accel and gyro, but as of 2015-07-23, there
	// is a problem with taking gyro self-test data which results in the
	// data being out of range, as if the accelerometer is clipping at
	// full-scale range.
	// NOTE: also or in  ST_FLAG_DEBUG_PRINTF to diagnose self-test failures
	int st_flags = ST_FLAG_GYRO | ST_FLAG_ACCEL;

	if (! mpu9250_get_factory_self_test_results(& factory_st))
	{
		printf("unable to read factory self-test results\r\n");
		while (true)
			;
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
		while(true)
			;
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
	status = mpu9250_check_selftest_results(st_flags, & factory_st, & avg_normal, & avg_selftest);
	if (status)
	{
		printf("self-test results out of tolerance FAILED: %u\r\n", status);
		while(true)
			;
	}
	printf("self-test results good\r\n");
#endif // DO_SELFTEST

	RdSensData_Init();  // initialize the sensor structures

	thisAccel.fgPerCount = MPU9250_GPERCOUNT;
	thisGyro.fDegPerSecPerCount = MPU9250_DEGPERSECPERCOUNT;
	thisMag.fuTPerCount = MPU9250_UTPERCOUNT;
	thisMag.fCountsPeruT = MPU9250_COUNTSPERUT;

	Fusion_Init();

	mpu9250_start();
	while (true)
	{
		if (! get_sensor_data_sample(& sensor_data))
		{
			printf("error reading raw sensor data\r\n");
			while (true)
				;
		}

#if 1
		print_sensor_data (& sensor_data);
#endif

    	globals.RunKF_Event_Flag = 0;

    	// copy raw sensor data to sensor fusion input
		// XXX temperature compensation and offsets should be dealt with first!
	    RdSensData_Run(sensor_data.accel, sensor_data.gyro, sensor_data.mag);

	    // reset the magnetic calibration flag (this is set by the fusion algorithm)
	    if (globals.RunKF_Event_Flag)
	    {
	    	globals.MagCal_Event_Flag = 0;
	    	Fusion_Run();
	    	if (globals.MagCal_Event_Flag)
	    		MagCal_Run(& thisMagCal, & thisMagBuffer);

// Note: neither printing the output as text nor outputing quaternion packets will
// work at a sample rate of 1 kHz. The quaternion output could be made to work
// by implementing interrupt-driven output instead of using the blocking
// putchar().
#if 0
	    	print_fusion_output();
#endif
#if 0
	    	output_quaternion_packet();
#endif
	    }
	}

	return 0;  // will never reach this
}
