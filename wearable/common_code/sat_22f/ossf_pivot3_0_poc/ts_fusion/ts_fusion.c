/*
 * ts_fusion.c
 *
 *  Created on: Sep 5, 2015
 *      Author: cwati
 */
#include <stdbool.h>
#include "ts_fusion.h"
#include "MK22F25612.h"
/*******************************************************************************
 * Freescale Open Source Sensor Fusion (OSSF) Include Files
 ******************************************************************************/
#include "build.h"
#include "tasks.h"
#include "magnetic.h"
#include "adc_ts.h"

//int16_t gyrBiasInt16_X;
//int16_t gyrBiasInt16_Y;
//int16_t gyrBiasInt16_Z;

int communicationTestCounter[2] = {0};
sensor_data_t *local_raw_data;


static void copy_raw_data(sensor_data_t *raw_data, sensor_record_t *sensor_data);
static void copy_sensor_fusion_output(sensor_record_t *sensor_data, struct MagCalibration * thisMagCal);


static uint32_t t;

uint32_t calculateSATID() {
      //uint32_t MOD=1073741824;
      //return SIM_UIDH%MOD+SIM_UIDMH%MOD+SIM_UIDML%MOD+SIM_UIDL%MOD;
      return (( SIM_UIDML >> 16) & 0x007F) | ((( SIM_UIDMH >> 0) & 0x007F) << 7) | ((( SIM_UIDMH >> 16) & 0x000F) << 14);
}

void set_mpu9250_sensor_num(uint8_t num) {
	mpu9250_num = num;
}

static bool firstTime = true;
void ts_fusion_init() {
	RdSensData_Init();  // initialize the sensor structures

	thisAccel[mpu9250_num].fgPerCount = MPU9250_GPERCOUNT;
	thisGyro[mpu9250_num].fDegPerSecPerCount = MPU9250_DEGPERSECPERCOUNT;
	thisMag[mpu9250_num].fuTPerCount = MPU9250_UTPERCOUNT;
	thisMag[mpu9250_num].fCountsPeruT = MPU9250_COUNTSPERUT;

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
    if (globals.RunKF_Event_Flag || globals.newKF_enabled)
    {
    	globals.MagCal_Event_Flag = 0;
    	/* If there's MAG data, then use MAG */
    	if (raw_data->mag[0] || raw_data->mag[1] || raw_data->mag[2]) {
    		if(!globals.enable_MagBiasCalibrationAlways && globals.outputMode !=3)
    			thisMagCal[mpu9250_num].iValidMagCal = TS_VALID_MAG_CAL; //HWE ALIGNMENT CONFIGURATION MAG ENABLED 2
    	} else {
    		thisMagCal[mpu9250_num].iValidMagCal = 0;
    	}

    	Fusion_Run(raw_data->accel, raw_data->gyro, raw_data->mag);
    	if (globals.newKF_enabled)
    	{
    		thisSV_9DOF_GBY_KALMAN[mpu9250_num].fqPl.q0 = qbn[0];
        	thisSV_9DOF_GBY_KALMAN[mpu9250_num].fqPl.q1 = qbn[1];
        	thisSV_9DOF_GBY_KALMAN[mpu9250_num].fqPl.q2 = qbn[2];
        	thisSV_9DOF_GBY_KALMAN[mpu9250_num].fqPl.q3 = qbn[3];
    	}
    	if (globals.MagCal_Event_Flag && (globals.outputMode == OUTPUT_MODE_BMAG || globals.enable_MagBiasCalibrationAlways) && !globals.disable_magCalibrationAlways) //HWE ALIGNMENT CONFIGURATION MAG ENABLED 2
    		MagCal_Run(&thisMagCal[mpu9250_num], &thisMagBuffer[mpu9250_num]);
    }
    // Copy Sensor Fusion output
    copy_sensor_fusion_output(data, &thisMagCal[mpu9250_num]);



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

	local_raw_data = raw_data;

#if ENABLE_PRINTF
	debug_printf("Time %6lu  ", raw_data->timestamp);
#endif
	// Read temperature data (but unused for now)
	if (raw_data->valid_sensors & SENSOR_TEMP) {
#if ENABLE_PRINTF
		debug_printf("Temp %6d  ", raw_data->temp);
#endif
	}
	// Copy accelerometer data, if valid
	if (raw_data->valid_sensors & SENSOR_ACCEL) {
		sensor_data->accel_x = raw_data->accel[X];
		sensor_data->accel_y = raw_data->accel[Y];
		sensor_data->accel_z = raw_data->accel[Z];
#if ENABLE_PRINTF
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
#if ENABLE_PRINTF
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
#if ENABLE_PRINTF
		debug_printf("Mag (%6d, %6d, %6d)  ", raw_data->mag[X], raw_data->mag[Y], raw_data->mag[Z]);
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
#if ENABLE_PRINTF
	debug_printf("\r\n");
#endif
}


/*******************************************************************************
 * Copy sensor fusion output into the sensor data record
 ******************************************************************************/
static void copy_sensor_fusion_output(sensor_record_t *sensor_data, struct MagCalibration *pthisMagCal)
{
	// mettere if
	static bool alternateQuaternionMag = true;

	if (globals.outputMode == OUTPUT_MODE_QUAT || globals.outputMode == OUTPUT_MODE_QUMA || globals.outputMode == OUTPUT_MODE_MAGO)
	{
		if((!globals.alternateQuaternionMag || globals.outputMode == OUTPUT_MODE_QUAT) && !(globals.outputMode == OUTPUT_MODE_MAGO))
		{
			sensor_data->quat_w = (int16_t)(CPI_QUATERNION16_MULTIPLIER*thisSV_9DOF_GBY_KALMAN[mpu9250_num].fqPl.q0);
			sensor_data->quat_x = (int16_t)(CPI_QUATERNION16_MULTIPLIER*thisSV_9DOF_GBY_KALMAN[mpu9250_num].fqPl.q1);
			sensor_data->quat_y = (int16_t)(CPI_QUATERNION16_MULTIPLIER*thisSV_9DOF_GBY_KALMAN[mpu9250_num].fqPl.q2);
			sensor_data->quat_z = (int16_t)(CPI_QUATERNION16_MULTIPLIER*thisSV_9DOF_GBY_KALMAN[mpu9250_num].fqPl.q3);

		}else{
			sensor_data->quat_w = (int16_t)(CPI_QUATERNION16_BIAS_MULTIPLIER*CPI_QUATERNION16_MAGONQU_COMMAND);
			sensor_data->quat_x = local_raw_data->mag[X];
			sensor_data->quat_y = local_raw_data->mag[Y];
			sensor_data->quat_z = local_raw_data->mag[Z];
		}

	}else if (globals.outputMode == OUTPUT_MODE_BGYR)	{
		sensor_data->quat_w = (int16_t)(CPI_QUATERNION16_BIAS_MULTIPLIER*CPI_QUATERNION16_AGBISEN_COMMAND);
		sensor_data->accel_x = accBiasInt16_X[mpu9250_num];
		sensor_data->accel_y = accBiasInt16_Y[mpu9250_num];
		sensor_data->accel_z = accBiasInt16_Z[mpu9250_num];
		sensor_data->gyro_x = gyrBiasInt16_X[mpu9250_num];
		sensor_data->gyro_y = gyrBiasInt16_Y[mpu9250_num];
		sensor_data->gyro_z = gyrBiasInt16_Z[mpu9250_num];
#if ENABLE_MAG
		sensor_data->mag_x = accSensInt16_X[mpu9250_num];
		sensor_data->mag_y = accSensInt16_Y[mpu9250_num];
		sensor_data->mag_z = accSensInt16_Z[mpu9250_num];
#else
		if(globals.alternateQuaternionMag)
		{
			sensor_data->quat_w = (int16_t)(CPI_QUATERNION16_BIAS_MULTIPLIER*CPI_QUATERNION16_MAGONQU_COMMAND);
			sensor_data->quat_x = accSensInt16_X[mpu9250_num];
			sensor_data->quat_y = accSensInt16_Y[mpu9250_num];
			sensor_data->quat_z = accSensInt16_Z[mpu9250_num];
		}
#endif
	}else if(globals.outputMode == OUTPUT_MODE_BMAG) {
		if(!globals.alternateQuaternionMag)
		{
			sensor_data->quat_w = (int16_t)(CPI_QUATERNION16_BIAS_MULTIPLIER*(100.0f + (float)thisMagCal[mpu9250_num].iValidMagCal));
			sensor_data->quat_x = (int16_t)(CPI_QUATERNION16_BIAS_MULTIPLIER*thisMagCal[mpu9250_num].fV[0]);
			sensor_data->quat_y = (int16_t)(CPI_QUATERNION16_BIAS_MULTIPLIER*thisMagCal[mpu9250_num].fV[1]);
			sensor_data->quat_z = (int16_t)(CPI_QUATERNION16_BIAS_MULTIPLIER*thisMagCal[mpu9250_num].fV[2]);

		}else{
			sensor_data->quat_w = (int16_t)(CPI_QUATERNION16_BIAS_MULTIPLIER*CPI_QUATERNION16_MAGONQU_COMMAND);
			sensor_data->quat_x = local_raw_data->mag[X];
			sensor_data->quat_y = local_raw_data->mag[Y];
			sensor_data->quat_z = local_raw_data->mag[Z];
		}
	}else if(globals.outputMode == OUTPUT_MODE_COUN){
		sensor_data->quat_w = (int16_t)(CPI_QUATERNION16_BIAS_MULTIPLIER*CPI_QUATERNION16_COUNTER_COMMAND);
		sensor_data->accel_x = (int16_t)(communicationTestCounter[COMMUNICATIONCOUNTER_BUFF]%CPI_QUATERNION16_COUNTER_MAXVALU);
		sensor_data->accel_y = (int16_t)(communicationTestCounter[COMMUNICATIONCOUNTER_SPIP]%CPI_QUATERNION16_COUNTER_MAXVALU);
	}else if(globals.outputMode == OUTPUT_MODE_VERS){

		// compute and send satID
		uint32_t mpuID_32 = calculateSATID();
		uint16_t * mpuID16 = (int16_t*) &mpuID_32;

		sensor_data->quat_w = (int16_t)(CPI_QUATERNION16_BIAS_MULTIPLIER*CPI_QUATERNION16_VERSION_COMMAND);
		sensor_data->accel_x = (int16_t)MAJOR_VERSION;
		sensor_data->accel_y = (int16_t)MINOR_VERSION;
		sensor_data->accel_z = (int16_t)REVIS_VERSION_SAT22F;
		sensor_data->gyro_x = 0;//(int16_t)REVIS_VERSION_SATNRD;
		sensor_data->gyro_y = 0;//(int16_t)REVIS_VERSION_HUB22F;
		sensor_data->gyro_z = 0;//(int16_t)REVIS_VERSION_HUBNRD;
#if ENABLE_MAG
		sensor_data->mag_x = (int16_t)REVIS_VERSION;
		sensor_data->mag_y = *mpuID16;
		sensor_data->mag_z = *(mpuID16+1);
#else
		sensor_data->quat_x = (int16_t)REVIS_VERSION;
		sensor_data->quat_y = *mpuID16;
		sensor_data->quat_z = *(mpuID16+1);
#endif
	}else if(globals.outputMode == OUTPUT_MODE_HIDS){

			// my mpu ID
			uint32_t mpuID_32 = calculateSATID();

			// Send to the client the sat id, mpu sat id, the HUB id, mpu hub ID and lcp
			uint16_t * myID16Sat = (int16_t*) &sat_id;
			uint16_t * mpuID16Sat = (int16_t*) &mpuID_32;
			uint16_t * rcvID16Hub = (int16_t*) &myHubIDrcvd;
			uint16_t * mpuID16Hub = (int16_t*) &myHubID9250;


			sensor_data->quat_w = (int16_t)(CPI_QUATERNION16_BIAS_MULTIPLIER*CPI_QUATERNION16_GET_IDS_COMMAND);
			sensor_data->accel_x = *myID16Sat;		// MY ID1
			sensor_data->accel_y = *(myID16Sat+1); 	// MY ID2
			sensor_data->accel_z = *mpuID16Sat;	// MY MPU ID1
			sensor_data->gyro_x = *(mpuID16Sat +1);	// MY MPU ID2
			sensor_data->gyro_y = *rcvID16Hub;		// HUB ID1;
			sensor_data->gyro_z = *(rcvID16Hub+1);	// HUB ID2;
#if ENABLE_MAG
			sensor_data->mag_x = *mpuID16Hub;		// HUB MPU ID1
			sensor_data->mag_y = *(mpuID16Hub+1);	// HUB MPU ID2
			sensor_data->mag_z = myLCP;				// CURRENT LCP
#else
		if(globals.alternateQuaternionMag)
		{
			sensor_data->quat_w = (int16_t)(CPI_QUATERNION16_BIAS_MULTIPLIER*CPI_QUATERNION16_MAGONQU_COMMAND);
			sensor_data->quat_x = *mpuID16Hub;		// HUB MPU ID1
			sensor_data->quat_y = *(mpuID16Hub+1);	// HUB MPU ID2
			sensor_data->quat_z = myLCP;			// CURRENT LCP
		}
#endif
	}else if(globals.outputMode == OUTPUT_MODE_BATL){
		uint16_t res16 = adc_read_1 (0);

		float batLev = readBatteryApplyFilter(res16);
		uint16_t batLevFlag = 0;

		if (batLev > 4.0)
			batLevFlag = 3;
		else
			if (batLev < 3.6)
				batLevFlag = 1;
			else
				batLevFlag = 2;

		sensor_data->quat_w = (int16_t)(CPI_QUATERNION16_BIAS_MULTIPLIER*CPI_QUATERNION16_BATTERY_COMMAND);
		sensor_data->quat_x = (int16_t)(CPI_QUATERNION16_MULTIPLIER*batLev);
		sensor_data->quat_y = (int16_t)(CPI_QUATERNION16_MULTIPLIER*0);
		sensor_data->quat_z = (int16_t)(CPI_QUATERNION16_MULTIPLIER*0);

		sensor_data->accel_x = batLevFlag;
	}

#if ENABLE_PRINTF
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
