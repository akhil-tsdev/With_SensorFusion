/*
 * Copyright (c) 2015, Turingsense
 *
 * cwati
 *
 * This code is for 22F FRDM eval board to run with Invensense.
 * This code is using Freescale Motion Library.
 * If you run this code, you can try the Python 3D example working.
 *
 */

#include <stdbool.h>
#include <stdio.h>

#include "fsl_device_registers.h"
#include "fsl_os_abstraction.h"

#include "board.h"
#include "mpu9250.h"

#include "../ossf/build.h"
#include "../ossf/tasks.h"

static int i = 0;

static void print_raw_data(raw_sensor_t *raw_data)
{
	printf("Time %6lu  ", raw_data->timestamp);
	if (raw_data->valid_sensors & SENSOR_TEMP)
		printf("Temp %6d  ", raw_data->temp);
	if (raw_data->valid_sensors & SENSOR_ACCEL)
		printf("Accel (%6d, %6d, %6d)  ", raw_data->accel[X], raw_data->accel[Y], raw_data->accel[Z]);
	if (raw_data->valid_sensors & SENSOR_GYRO)
		printf("Gyro (%6d, %6d, %6d)  ", raw_data->gyro[X], raw_data->gyro[Y], raw_data->gyro[Z]);
	if (raw_data->valid_sensors & SENSOR_MAG)
		printf("Mag (%6d, %6d, %6d)  ", raw_data->mag[X], raw_data->mag[Y], raw_data->mag[Z]);
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

static bool get_raw_data_sample(raw_sensor_t *raw_data)
{
	while (! mpu9250_data_avail())
		;
	mpu9250_clear_data_avail();
	return mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP, raw_data);
}

int main(void)
{
	raw_sensor_t raw_data;

	hardware_init();
	OSA_Init();
    configure_uart_pins(BOARD_DEBUG_UART_INSTANCE);
    dbg_uart_init();

    printf("\r\nOpen Source Sensor Fusion with Invensense MPU-9250\r\n");

	if (! mpu9250_init())
	{
		printf("error initializing MPU9250\r\n");
		while (true)
			;
	}

	RdSensData_Init();  // initialize the sensor structures

	thisAccel.fgPerCount = MPU9250_GPERCOUNT;
	thisGyro.fDegPerSecPerCount = MPU9250_DEGPERSECPERCOUNT;
	thisMag.fuTPerCount = MPU9250_UTPERCOUNT;
	thisMag.fCountsPeruT = MPU9250_COUNTSPERUT;

	Fusion_Init();

	while (true)
	{
		if (! get_raw_data_sample(& raw_data))
		{
			printf("error reading raw sensor data\r\n");
			while (true)
				;
		}

#if 0
		print_raw_data (& raw_data);
#endif

    	globals.RunKF_Event_Flag = 0;

    	// copy raw sensor data to sensor fusion input
		// XXX temperature compensation and offsets should be dealt with first!
	    RdSensData_Run(raw_data.accel, raw_data.gyro, raw_data.mag);

	    // reset the magnetic calibration flag (this is set by the fusion algorithm)
	    if (globals.RunKF_Event_Flag)
	    {
	    	globals.MagCal_Event_Flag = 0;
	    	Fusion_Run();
	    	if (globals.MagCal_Event_Flag)
	    		MagCal_Run(& thisMagCal, & thisMagBuffer);
#if 0
	    	print_fusion_output();
#endif
#if 1
	    	output_quaternion_packet();
#endif
	    }
	}

	return 0;  // will never reach this
}
