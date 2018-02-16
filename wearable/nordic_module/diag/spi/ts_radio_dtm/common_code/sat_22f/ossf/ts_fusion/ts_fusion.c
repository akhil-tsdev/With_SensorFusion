/*
 * ts_fusion.c
 *
 *  Created on: Sep 5, 2015
 *      Author: cwati
 */
#include <stdbool.h>
#include "ts_fusion.h"

/*******************************************************************************
 * Freescale Open Source Sensor Fusion (OSSF) Include Files
 ******************************************************************************/
#include "build.h"
#include "tasks.h"
#include "magnetic.h"

#include "fsl_debug_console.h"

static void copy_raw_data(sensor_data_t *raw_data, sensor_record_t *sensor_data);
static void copy_sensor_fusion_output(sensor_record_t *sensor_data, struct MagCalibration * thisMagCal);

void ts_fusion_init() {
	RdSensData_Init();  // initialize the sensor structures

	thisAccel.fgPerCount = MPU9250_GPERCOUNT;
	thisGyro.fDegPerSecPerCount = MPU9250_DEGPERSECPERCOUNT;
	thisMag.fuTPerCount = MPU9250_UTPERCOUNT;
	thisMag.fCountsPeruT = MPU9250_COUNTSPERUT;

	Fusion_Init();
}

void process_sensor_fusion(sensor_data_t *raw_data, sensor_record_t* data) {
    // Copy raw sensor output
    copy_raw_data(raw_data, data);

    // Do sensor fusion using Freescale's OSSF
    globals.RunKF_Event_Flag = 0;
    // copy raw sensor data to sensor fusion input
    // XXX temperature compensation and offsets should be dealt with first!
    RdSensData_Run(raw_data->accel, raw_data->gyro, raw_data->mag);
    // reset the magnetic calibration flag (this is set by the fusion algorithm)
    if (globals.RunKF_Event_Flag)
    {
    	globals.MagCal_Event_Flag = 0;
    	/* If there's MAG data, then use MAG */
    	if (raw_data->mag[0] || raw_data->mag[1] || raw_data->mag[2]) {
    		if(!globals.enable_MagBiasCalibrationAlways && !globals.enable_MagBiasSending)
    			thisMagCal.iValidMagCal = TS_VALID_MAG_CAL; //HWE ALIGNMENT CONFIGURATION MAG ENABLED 2
    	} else {
    		thisMagCal.iValidMagCal = 0;
    	}

    	Fusion_Run();

    	/*if(thisMagCal.iValidMagCal && (globals.enable_MagBiasSending || globals.enable_MagBiasCalibrationAlways))
    		globals.MagCal_Event_Flag = true;
    	else
    		globals.MagCal_Event_Flag = false;*/


    	if (globals.MagCal_Event_Flag && (globals.enable_MagBiasSending || globals.enable_MagBiasCalibrationAlways) && !globals.disable_magCalibrationAlways) //HWE ALIGNMENT CONFIGURATION MAG ENABLED 2
    		MagCal_Run(& thisMagCal, & thisMagBuffer);
    	// Copy Sensor Fusion output
    	copy_sensor_fusion_output(data, &thisMagCal);
    }
}


/*******************************************************************************
 * Copy MPU9250 raw output into the sensor data record
 * raw_data:   	source
 * sensor_data:	destination
 ******************************************************************************/
static void copy_raw_data(sensor_data_t *raw_data, sensor_record_t *sensor_data)
{
	//upper sampling
	static volatile int16_t mag[3];

#if DEBUG
	debug_printf("Time %6lu  ", raw_data->timestamp);
#endif
	// Read temperature data (but unused for now)
	if (raw_data->valid_sensors & SENSOR_TEMP) {
#if DEBUG
		debug_printf("Temp %6d  ", raw_data->temp);
#endif
	}
	// Copy accelerometer data, if valid
	if (raw_data->valid_sensors & SENSOR_ACCEL) {
		sensor_data->accel_x = raw_data->accel[X];
		sensor_data->accel_y = raw_data->accel[Y];
		sensor_data->accel_z = raw_data->accel[Z];
#if DEBUG
		debug_printf("Accel (%6d, %6d, %6d)  ", raw_data->accel[X], raw_data->accel[Y], raw_data->accel[Z]);
#endif
	} else {
		sensor_data->accel_x = 0;
		sensor_data->accel_y = 0;
		sensor_data->accel_z = 0;
	}
	// Copy gyroscope data, if valid
	if (raw_data->valid_sensors & SENSOR_GYRO) {
		sensor_data->gyro_x = raw_data->gyro[X];
		sensor_data->gyro_y = raw_data->gyro[Y];
		sensor_data->gyro_z = raw_data->gyro[Z];
#if DEBUG
		debug_printf("Gyro (%6d, %6d, %6d)  ", raw_data->gyro[X], raw_data->gyro[Y], raw_data->gyro[Z]);
#endif
	} else {
		sensor_data->gyro_x = 0;
		sensor_data->gyro_y = 0;
		sensor_data->gyro_z = 0;
	}
#if ENABLE_MAG
	if (raw_data->valid_sensors & SENSOR_MAG) {
		sensor_data->mag_x = raw_data->mag[X];
		sensor_data->mag_y = raw_data->mag[Y];
		sensor_data->mag_z = raw_data->mag[Z];
		mag[X] = raw_data->mag[X];
		mag[Y] = raw_data->mag[Y];
		mag[Z] = raw_data->mag[Z];
#if DEBUG
		printf("Mag (%6d, %6d, %6d)  ", raw_data->mag[X], raw_data->mag[Y], raw_data->mag[Z]);
#endif
	} else {
#if 2*TS_MAG_RATE==TS_GYRO_FREQ
		sensor_data->mag_x = mag[X];
		sensor_data->mag_y = mag[Y];
		sensor_data->mag_z = mag[Z];
#else
		sensor_data->mag_x = 0;
		sensor_data->mag_y = 0;
		sensor_data->mag_z = 0;
#endif
	}
#endif
#if DEBUG
	printf("\r\n");
#endif
}

/*******************************************************************************
 * Copy sensor fusion output into the sensor data record
 ******************************************************************************/
static void copy_sensor_fusion_output(sensor_record_t *sensor_data, struct MagCalibration *pthisMagCal)
{
	if(!globals.enable_MagBiasSending){
		sensor_data->quat_w = thisSV_9DOF_GBY_KALMAN.fqPl.q0;
		sensor_data->quat_x = thisSV_9DOF_GBY_KALMAN.fqPl.q1;
		sensor_data->quat_y = thisSV_9DOF_GBY_KALMAN.fqPl.q2;
		sensor_data->quat_z = thisSV_9DOF_GBY_KALMAN.fqPl.q3;
	}
	else
	{
		if(!globals.disable_magCalibrationAlways)
			sensor_data->quat_w = (float)(100 + thisMagCal.iValidMagCal);
		else
			sensor_data->quat_w = (float)(200);

		sensor_data->quat_x = thisMagCal.fV[0];
		sensor_data->quat_y = thisMagCal.fV[1];
		sensor_data->quat_z = thisMagCal.fV[2];
	}
//	sensor_data->lnaccel_x = thisSV_9DOF_GBY_KALMAN.faSePl[X];
//	sensor_data->lnaccel_y = thisSV_9DOF_GBY_KALMAN.faSePl[Y];
//	sensor_data->lnaccel_z = thisSV_9DOF_GBY_KALMAN.faSePl[Z];
#if DEBUG
	/*
	// for floating point printf, the linker flags must include "-u _printf_float"
	printf("roll %f, pitch %f, yaw %f, compass %f, tilt from vertical %f\r\n",
		   thisSV_9DOF_GBY_KALMAN.fPhiPl,	// roll (deg)
		   thisSV_9DOF_GBY_KALMAN.fThePl,	// pitch (deg)
		   thisSV_9DOF_GBY_KALMAN.fPsiPl,	// yaw (deg)
		   thisSV_9DOF_GBY_KALMAN.fRhoPl,	// compass (deg)
		   thisSV_9DOF_GBY_KALMAN.fChiPl);	// tilt from vertical (deg)
	*/
#endif
}
