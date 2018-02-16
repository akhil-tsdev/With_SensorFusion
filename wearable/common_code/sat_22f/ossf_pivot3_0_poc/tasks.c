// Copyright (c) 2014, Freescale Semiconductor, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Freescale Semiconductor, Inc. nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL FREESCALE SEMICONDUCTOR, INC. BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This file defines sensor fusion apps in terms of very high level functions.
// It also includes functions for applying hardware abstraction mapping (HAL functions).
// These include:
//    RdSensData_Init()
//    Fusion_Init()
//    RdSensData_Run()
//    Fusion_Run()
//    MagCal_Run()
//    ApplyAccelHAL()
//    ApplyMagHAL()
//    ApplyGyroHAL()

#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "time.h"
#include "string.h"

#include "build.h"
#include "magnetic.h"
#include "tasks.h"
#include "fusion.h"

// new SF
#include "KF.h"
#include "orientationComputation.h"
#include "matrixComputations.h"

struct ProjectGlobals globals;

// sensor data structures
struct AccelSensor thisAccel[MAX_SENSORS_IN_CS];				// this accelerometer

struct MagSensor thisMag[MAX_SENSORS_IN_CS];					// this magnetometer
struct MagCalibration thisMagCal[MAX_SENSORS_IN_CS];			// hard and soft iron magnetic calibration
struct MagneticBuffer thisMagBuffer[MAX_SENSORS_IN_CS];		// magnetometer measurement buffer

struct GyroSensor thisGyro[MAX_SENSORS_IN_CS];					// this gyro

// 9DOF accelerometer, magnetometer and gyro (Kalman) structure
struct SV_9DOF_GBY_KALMAN thisSV_9DOF_GBY_KALMAN[MAX_SENSORS_IN_CS];

// New KF struct gyr/acc only
KF accGyrKF[MAX_SENSORS_IN_CS];
KF_params accGyrKFpar[MAX_SENSORS_IN_CS];
refFrame UVRH[MAX_SENSORS_IN_CS];
float qbn[4];


uint8_t mpu9250_num = 0;										// The sensor number for PIVOT 3.0

// function initializes the sensor data structures
void RdSensData_Init(void)
{
	int8_t i;							// loop counter

	// zero sums of sensor data (typically 200Hz) on first execution
	for (i = X; i <= Z; i++)
	{
		thisAccel[mpu9250_num].iSumGpFast[i] = 0;
		thisMag[mpu9250_num].iSumBpFast[i] = 0;
		thisGyro[mpu9250_num].iSumYpFast[i] = 0;
	}

	return;
}

// function initializes the sensor fusion and magnetic calibration and sets loopcounter to zero
void Fusion_Init(void)
{
	struct MagneticBuffer *ptr2;
	struct MagCalibration *ptr1;

	ptr1 = &thisMagCal[mpu9250_num];
	ptr2 = &thisMagBuffer[mpu9250_num];

	// magnetic DOF: reset magnetic calibration and magnetometer data buffer (not needed for 3DOF)
	fInitMagCalibration(ptr1, ptr2);

	// reset the default quaternion type to the simplest Q3 (it will be updated during the initializations)
	globals.DefaultQuaternionPacketType = Q3;
	
	// force a reset of all the algorithms next time they execute
	// the initialization will result in the default and current quaternion being set to the most sophisticated
	// algorithm supported by the build
	thisSV_9DOF_GBY_KALMAN[mpu9250_num].resetflag = true;

	UVRH[mpu9250_num].initialized = false;

	// reset the loop counter to zero for first iteration
	globals.loopcounter = 0;

	return;
}


void Fusion_Init1(KF_params * thisKFpar, float stdg, float stda, float ca, float cb, float po, float Xdim, float Ydim)
{
  thisKFpar->stdg = stdg;
  thisKFpar->stds = stda;

  thisKFpar->ca = ca;
  thisKFpar->cb = cb;
  thisKFpar->po = po;

  thisKFpar->Xdim = Xdim;
  thisKFpar->Ydim = Ydim;


  // ref frame (Up Vertical, Right Horizontal)
  UVRH[mpu9250_num].initialized = false;
  UVRH[mpu9250_num].allocated = false;
  accGyrKFaccGyrKF.allocated = false;

  // reset the default quaternion type to the simplest Q3 (it will be updated during the initializations)
  globals.DefaultQuaternionPacketType = Q3;

  // reset the loop counter to zero for first iteration
  globals.loopcounter = 0;


}


// this function is called at (typically 200Hz) to apply sensor data to the HAL
void RdSensData_Run(uint16_t *accel, uint16_t *gyro, uint16_t *mag)
{
	static int8_t iCounter[MAX_SENSORS_IN_CS] = {0};	// the number of gyro readings summed
	int8_t i;							// loop counter

	memcpy(thisAccel[mpu9250_num].iGpFast, accel, sizeof(thisAccel[mpu9250_num].iGpFast));
	memcpy(thisGyro[mpu9250_num].iYpFast[iCounter[mpu9250_num]], gyro, sizeof(thisGyro[mpu9250_num].iYpFast[iCounter[mpu9250_num]]));
	memcpy(thisMag[mpu9250_num].iBpFast, mag, sizeof(thisMag[mpu9250_num].iBpFast));

	// apply the 200Hz sensor data to the HAL
	//ApplyAccelHAL(&thisAccel[mpu9250_num]);
	//ApplyGyroHAL(&thisGyro[mpu9250_num], iCounter[mpu9250_num]);
	//ApplyMagHAL(&thisMag[mpu9250_num]);

	// sum the 200Hz HAL-aligned sensor data
	for (i = X; i <= Z; i++)
	{
		thisAccel[mpu9250_num].iSumGpFast[i] += (int32_t) thisAccel[mpu9250_num].iGpFast[i];
		thisMag[mpu9250_num].iSumBpFast[i] += (int32_t) thisMag[mpu9250_num].iBpFast[i];
		thisGyro[mpu9250_num].iSumYpFast[i] += (int32_t) thisGyro[mpu9250_num].iYpFast[iCounter[mpu9250_num]][i];
	}

	// increment the decimation counter for the next iteration
	iCounter[mpu9250_num]++;

	// every OVERSAMPLE_RATIO iterations process the summed over-sampled readings
	if (iCounter[mpu9250_num] == OVERSAMPLE_RATIO)
	{
		// process the HAL-aligned sensor measurements prior to calling the sensor fusion
		for (i = X; i <= Z; i++)
		{
			// calculate the fast accelerometer reading for the Kalman filters (to reduce phase errors)
			thisAccel[mpu9250_num].fGpFast[i] = (float) thisAccel[mpu9250_num].iGpFast[i] * thisAccel[mpu9250_num].fgPerCount;
			// calculate the average (typically 25Hz) accelerometer reading
			thisAccel[mpu9250_num].iGp[i] = (int16_t)(thisAccel[mpu9250_num].iSumGpFast[i] / OVERSAMPLE_RATIO);
			thisAccel[mpu9250_num].fGp[i] = (float) thisAccel[mpu9250_num].iSumGpFast[i] * thisAccel[mpu9250_num].fgPerCount / (float) OVERSAMPLE_RATIO;

			// calculate the fast magnetometer reading for the Kalman filters (to reduce phase errors)
			thisMag[mpu9250_num].fBpFast[i] = (float) thisMag[mpu9250_num].iBpFast[i] * thisMag[mpu9250_num].fuTPerCount;
			// calculate the average (typically 25Hz) magnetometer reading
			thisMag[mpu9250_num].iBp[i] = (int16_t)(thisMag[mpu9250_num].iSumBpFast[i] / OVERSAMPLE_RATIO);
			thisMag[mpu9250_num].fBp[i] = (float) thisMag[mpu9250_num].iSumBpFast[i] * thisMag[mpu9250_num].fuTPerCount / (float) OVERSAMPLE_RATIO;

			// calculate the average (typically 25Hz) gyro reading
			thisGyro[mpu9250_num].iYp[i] = (int16_t)(thisGyro[mpu9250_num].iSumYpFast[i] / OVERSAMPLE_RATIO);
			thisGyro[mpu9250_num].fYp[i] = (float) thisGyro[mpu9250_num].iSumYpFast[i] * thisGyro[mpu9250_num].fDegPerSecPerCount / (float) OVERSAMPLE_RATIO;

			// zero the sensor sums for the next iteration
			thisAccel[mpu9250_num].iSumGpFast[i] = 0;
			thisMag[mpu9250_num].iSumBpFast[i] = 0;
			thisGyro[mpu9250_num].iSumYpFast[i] = 0;
		}

		// zero the counter and set the event flag to start the sensor fusion task
		iCounter[mpu9250_num] = 0;
		globals.RunKF_Event_Flag = 1;

	} // end of over-sampling test
}

// function runs the sensor fusion algorithms
void Fusion_Run(int16_t *accel, int16_t *gyro, int16_t *mag)
{
	int8_t initiatemagcal;				// flag to initiate a new magnetic calibration

	// magnetic DOF: remove hard and soft iron terms from Bp (uT) to get calibrated data Bc (uT)
	fInvertMagCal(&thisMag[mpu9250_num], &thisMagCal[mpu9250_num]);

	// update magnetic buffer checking for i) absence of first all-zero magnetometer output and ii) no calibration in progress
	// an all zero magnetometer reading can occur after power-on at rare intervals but it simply won't be used in the buffer
	if (!((globals.loopcounter < 100) && (thisMag[mpu9250_num].iBpFast[X] == 0) && (thisMag[mpu9250_num].iBpFast[Y] == 0) && (thisMag[mpu9250_num].iBpFast[Z] == 0)) && !thisMagCal[mpu9250_num].iCalInProgress)
	{
		// update the magnetometer measurement buffer integer magnetometer data (typically at 25Hz)
		iUpdateMagnetometerBuffer(&thisMagBuffer[mpu9250_num], &thisAccel[mpu9250_num], &thisMag[mpu9250_num], globals.loopcounter);
	}

	// 9DOF Accel / Mag / Gyro: apply the Kalman filter
	if (PARALLELNOTSEQUENTIAL || (globals.QuaternionPacketType == Q9))
	{
		// XXX thisSV_9DOF_GBY_KALMAN.systick = SYST_CVR & 0x00FFFFFF;
		if (globals.newKF_enabled)
			fRunAccGyrSF(&accGyrKF[mpu9250_num],&accGyrKFpar[mpu9250_num],accel,gyro,&UVRH[mpu9250_num],qbn);
		else
			fRun_9DOF_GBY_KALMAN(&thisSV_9DOF_GBY_KALMAN[mpu9250_num], &thisAccel[mpu9250_num], &thisMag[mpu9250_num], &thisGyro[mpu9250_num], &thisMagCal[mpu9250_num], THISCOORDSYSTEM, OVERSAMPLE_RATIO);
		// XXX thisSV_9DOF_GBY_KALMAN.systick -= SYST_CVR & 0x00FFFFFF;
		// XXX if (thisSV_9DOF_GBY_KALMAN.systick < 0) thisSV_9DOF_GBY_KALMAN.systick += SYST_RVR;
	}

	// 6DOF and 9DOF: decide whether to initiate a magnetic calibration
	// check no magnetic calibration is in progress
	if (!thisMagCal[mpu9250_num].iCalInProgress)
	{
		// do the first 4 element calibration immediately there are a minimum of MINMEASUREMENTS4CAL
		initiatemagcal = (!thisMagCal[mpu9250_num].iMagCalHasRun && (thisMagBuffer[mpu9250_num].iMagBufferCount >= MINMEASUREMENTS4CAL));

		// otherwise initiate a calibration at intervals depending on the number of measurements available
		initiatemagcal |= ((thisMagBuffer[mpu9250_num].iMagBufferCount >= MINMEASUREMENTS4CAL) &&
				(thisMagBuffer[mpu9250_num].iMagBufferCount < MINMEASUREMENTS7CAL) &&
				!(globals.loopcounter % INTERVAL4CAL));
		initiatemagcal |= ((thisMagBuffer[mpu9250_num].iMagBufferCount >= MINMEASUREMENTS7CAL) &&
				(thisMagBuffer[mpu9250_num].iMagBufferCount < MINMEASUREMENTS10CAL) &&
				!(globals.loopcounter % INTERVAL7CAL));
		initiatemagcal |= ((thisMagBuffer[mpu9250_num].iMagBufferCount >= MINMEASUREMENTS10CAL) &&
				!(globals.loopcounter % INTERVAL10CAL));

		// initiate the magnetic calibration if any of the conditions are met
		if (initiatemagcal)
		{
			// set the flags denoting that a calibration is in progress
			thisMagCal[mpu9250_num].iCalInProgress = 1;
			thisMagCal[mpu9250_num].iMagCalHasRun = 1;

			// enable the magnetic calibration task to run
			globals.MagCal_Event_Flag = 1;
		} // end of test whether to call calibration functions
	} // end of test that no calibration is already in progress

	// increment the loopcounter (used for time stamping magnetic data)
	globals.loopcounter++;

	return;
}

// function runs the magnetic calibration
void MagCal_Run(struct MagCalibration *pthisMagCal, struct MagneticBuffer *pthisMagBuffer)
{
	int8_t i, j;			// loop counters
	int8_t isolver;		// magnetic solver used

	// 4 element calibration case
	if (pthisMagBuffer->iMagBufferCount < MINMEASUREMENTS7CAL)
	{
		// age the existing fit error to avoid one good calibration locking out future updates
		if (pthisMagCal->iValidMagCal)
		{
			pthisMagCal->fFitErrorpc *= (1.0F + (float) INTERVAL4CAL * (float) OVERSAMPLE_RATIO / ((float) SENSORFS * FITERRORAGINGSECS));	
		}
		// call the 4 element matrix inversion calibration
		isolver = 4;
		fUpdateCalibration4INV(pthisMagCal, pthisMagBuffer, &thisMag[mpu9250_num]);
	}
	// 7 element calibration case
	else if (pthisMagBuffer->iMagBufferCount < MINMEASUREMENTS10CAL)
	{
		// age the existing fit error to avoid one good calibration locking out future updates
		if (pthisMagCal->iValidMagCal)
		{
			pthisMagCal->fFitErrorpc *= (1.0F + (float) INTERVAL7CAL * (float) OVERSAMPLE_RATIO / ((float) SENSORFS * FITERRORAGINGSECS));	
		}
		// call the 7 element eigenpair calibration
		isolver = 7;
		fUpdateCalibration7EIG(pthisMagCal, pthisMagBuffer, &thisMag[mpu9250_num]);
	}
	// 10 element calibration case
	else
	{
		// age the existing fit error to avoid one good calibration locking out future updates
		if (pthisMagCal->iValidMagCal)
		{
			pthisMagCal->fFitErrorpc *= (1.0F + (float) INTERVAL10CAL * (float) OVERSAMPLE_RATIO / ((float) SENSORFS * FITERRORAGINGSECS));	
		}
		// call the 10 element eigenpair calibration
		isolver = 10;
		fUpdateCalibration10EIG(pthisMagCal, pthisMagBuffer, &thisMag[mpu9250_num]);
	}

	// the trial geomagnetic field must be in range (earth is 22uT to 67uT)
	if ((pthisMagCal->ftrB >= MINBFITUT) && (pthisMagCal->ftrB <= MAXBFITUT))		
	{
		// always accept the calibration if i) no previous calibration exists ii) the calibration fit is reduced or
		// an improved solver was used giving a good trial calibration (4% or under)
		if ((pthisMagCal->iValidMagCal == 0) ||
				(pthisMagCal->ftrFitErrorpc <= pthisMagCal->fFitErrorpc) ||
				((isolver > pthisMagCal->iValidMagCal) && (pthisMagCal->ftrFitErrorpc <= 4.0F)))
		{
			// accept the new calibration solution
			pthisMagCal->iValidMagCal = isolver;
			pthisMagCal->fFitErrorpc = pthisMagCal->ftrFitErrorpc;
			pthisMagCal->fB = pthisMagCal->ftrB;
			pthisMagCal->fFourBsq = 4.0F * pthisMagCal->ftrB * pthisMagCal->ftrB;
			for (i = X; i <= Z; i++)
			{
				pthisMagCal->fV[i] = pthisMagCal->ftrV[i];
				for (j = X; j <= Z; j++)
				{
					pthisMagCal->finvW[i][j] = pthisMagCal->ftrinvW[i][j];
				}
			}
		} // end of test to accept the new calibration 
	} // end of test for geomagenetic field strength in range

	// reset the calibration in progress flag to allow writing to the magnetic buffer
	pthisMagCal->iCalInProgress = 0;

	return;
}

// function applies the hardware abstraction layer to the Fast (typically 200Hz) accelerometer readings
void ApplyAccelHAL(struct AccelSensor *pthisAccel)
{
#if THISCOORDSYSTEM == NED
	int16_t itmp16;
	itmp16 = thisAccel[mpu9250_num].iGpFast[X];
	thisAccel[mpu9250_num].iGpFast[X] = thisAccel[mpu9250_num].iGpFast[Y];
	thisAccel[mpu9250_num].iGpFast[Y] = itmp16;
#endif // NED
#if THISCOORDSYSTEM == ANDROID
	thisAccel[mpu9250_num].iGpFast[X] = -thisAccel[mpu9250_num].iGpFast[X];
	thisAccel[mpu9250_num].iGpFast[Y] = -thisAccel[mpu9250_num].iGpFast[Y];
#endif // Android
#if (THISCOORDSYSTEM == WIN8)
	thisAccel[mpu9250_num].iGpFast[Z] = -thisAccel[mpu9250_num].iGpFast[Z];
#endif // Win8

	return;
}

// function applies the hardware abstraction layer to the Fast (typically 200Hz) magnetometer readings
void ApplyMagHAL(struct MagSensor *pthisMag)
{
#if THISCOORDSYSTEM == NED
	int16_t itmp16;
	itmp16 = thisMag[mpu9250_num].iBpFast[X];
	thisMag[mpu9250_num].iBpFast[X] = -thisMag[mpu9250_num].iBpFast[Y];
	thisMag[mpu9250_num].iBpFast[Y] = -itmp16;
	thisMag[mpu9250_num].iBpFast[Z] = -thisMag[mpu9250_num].iBpFast[Z];
#endif // NED
#if THISCOORDSYSTEM == ANDROID
	thisMag[mpu9250_num].iBpFast[X] = -thisMag[mpu9250_num].iBpFast[X];
	thisMag[mpu9250_num].iBpFast[Y] = -thisMag[mpu9250_num].iBpFast[Y];
#endif // Android
#if THISCOORDSYSTEM == WIN8
	thisMag[mpu9250_num].iBpFast[X] = -thisMag[mpu9250_num].iBpFast[X];
	thisMag[mpu9250_num].iBpFast[Y] = -thisMag[mpu9250_num].iBpFast[Y];
#endif

	// finally correct for the left handed magnetic coordinate system in MAG3110
#if defined USE_MAG3110
	thisMag[mpu9250_num].iBpFast[Z] = -thisMag[mpu9250_num].iBpFast[Z];
#endif

	return;
}

// function applies the hardware abstraction layer to the Fast (typically 200Hz) gyro readings
void ApplyGyroHAL(struct GyroSensor *pthisGyro, int16_t irow)
{
#if THISCOORDSYSTEM == NED
	int16_t itmp16;
	itmp16 = thisGyro[mpu9250_num].iYpFast[irow][X];
	thisGyro[mpu9250_num].iYpFast[irow][X] = -thisGyro[mpu9250_num].iYpFast[irow][Y];
	thisGyro[mpu9250_num].iYpFast[irow][Y] = -itmp16;
	thisGyro[mpu9250_num].iYpFast[irow][Z] = -thisGyro[mpu9250_num].iYpFast[irow][Z];
#endif // NED
#if THISCOORDSYSTEM == ANDROID
	thisGyro[mpu9250_num].iYpFast[irow][X] = -thisGyro[mpu9250_num].iYpFast[irow][X];
	thisGyro[mpu9250_num].iYpFast[irow][Y] = -thisGyro[mpu9250_num].iYpFast[irow][Y];
#endif // Android
#if THISCOORDSYSTEM == WIN8
	thisGyro[mpu9250_num].iYpFast[irow][X] = -thisGyro[mpu9250_num].iYpFast[irow][X];
	thisGyro[mpu9250_num].iYpFast[irow][Y] = -thisGyro[mpu9250_num].iYpFast[irow][Y];
#endif // Win8

	return;
}
