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

struct ProjectGlobals globals;

// sensor data structures
struct AccelSensor thisAccel;				// this accelerometer

struct MagSensor thisMag;					// this magnetometer
struct MagCalibration thisMagCal;			// hard and soft iron magnetic calibration
struct MagneticBuffer thisMagBuffer;		// magnetometer measurement buffer

struct GyroSensor thisGyro;					// this gyro

// 9DOF accelerometer, magnetometer and gyro (Kalman) structure
struct SV_9DOF_GBY_KALMAN thisSV_9DOF_GBY_KALMAN;

// function initializes the sensor data structures
void RdSensData_Init(void)
{
	int8_t i;							// loop counter

	// zero sums of sensor data (typically 200Hz) on first execution
	for (i = X; i <= Z; i++)
	{
		thisAccel.iSumGpFast[i] = 0;
		thisMag.iSumBpFast[i] = 0;
		thisGyro.iSumYpFast[i] = 0;
	}

	return;
}

// function initializes the sensor fusion and magnetic calibration and sets loopcounter to zero
void Fusion_Init(void)
{
	// magnetic DOF: reset magnetic calibration and magnetometer data buffer (not needed for 3DOF)
	fInitMagCalibration(&thisMagCal, &thisMagBuffer);

	// reset the default quaternion type to the simplest Q3 (it will be updated during the initializations)
	globals.DefaultQuaternionPacketType = Q3;
	
	// force a reset of all the algorithms next time they execute
	// the initialization will result in the default and current quaternion being set to the most sophisticated
	// algorithm supported by the build
	thisSV_9DOF_GBY_KALMAN.resetflag = true;

	// reset the loop counter to zero for first iteration
	globals.loopcounter = 0;

	return;
}

// this function is called at (typically 200Hz) to apply sensor data to the HAL
void RdSensData_Run(uint16_t *accel, uint16_t *gyro, uint16_t *mag)
{
	static int8_t iCounter = 0;		// the number of gyro readings summed
	int8_t i;							// loop counter

	memcpy(thisAccel.iGpFast, accel, sizeof(thisAccel.iGpFast));
	memcpy(thisGyro.iYpFast[iCounter], gyro, sizeof(thisGyro.iYpFast[iCounter]));
	memcpy(thisMag.iBpFast, mag, sizeof(thisMag.iBpFast));

	// apply the 200Hz sensor data to the HAL
	//ApplyAccelHAL(&thisAccel);
	//ApplyGyroHAL(&thisGyro, iCounter);
	//ApplyMagHAL(&thisMag);

	// sum the 200Hz HAL-aligned sensor data
	for (i = X; i <= Z; i++)
	{
		thisAccel.iSumGpFast[i] += (int32_t) thisAccel.iGpFast[i];
		thisMag.iSumBpFast[i] += (int32_t) thisMag.iBpFast[i];
		thisGyro.iSumYpFast[i] += (int32_t) thisGyro.iYpFast[iCounter][i];
	}

	// increment the decimation counter for the next iteration
	iCounter++;

	// every OVERSAMPLE_RATIO iterations process the summed over-sampled readings
	if (iCounter == OVERSAMPLE_RATIO)
	{
		// process the HAL-aligned sensor measurements prior to calling the sensor fusion
		for (i = X; i <= Z; i++)
		{
			// calculate the fast accelerometer reading for the Kalman filters (to reduce phase errors)
			thisAccel.fGpFast[i] = (float) thisAccel.iGpFast[i] * thisAccel.fgPerCount;
			// calculate the average (typically 25Hz) accelerometer reading
			thisAccel.iGp[i] = (int16_t)(thisAccel.iSumGpFast[i] / OVERSAMPLE_RATIO);
			thisAccel.fGp[i] = (float) thisAccel.iSumGpFast[i] * thisAccel.fgPerCount / (float) OVERSAMPLE_RATIO;

			// calculate the fast magnetometer reading for the Kalman filters (to reduce phase errors)
			thisMag.fBpFast[i] = (float) thisMag.iBpFast[i] * thisMag.fuTPerCount;
			// calculate the average (typically 25Hz) magnetometer reading
			thisMag.iBp[i] = (int16_t)(thisMag.iSumBpFast[i] / OVERSAMPLE_RATIO);
			thisMag.fBp[i] = (float) thisMag.iSumBpFast[i] * thisMag.fuTPerCount / (float) OVERSAMPLE_RATIO;

			// calculate the average (typically 25Hz) gyro reading
			thisGyro.iYp[i] = (int16_t)(thisGyro.iSumYpFast[i] / OVERSAMPLE_RATIO);
			thisGyro.fYp[i] = (float) thisGyro.iSumYpFast[i] * thisGyro.fDegPerSecPerCount / (float) OVERSAMPLE_RATIO;

			// zero the sensor sums for the next iteration
			thisAccel.iSumGpFast[i] = 0;
			thisMag.iSumBpFast[i] = 0;
			thisGyro.iSumYpFast[i] = 0;
		}

		// zero the counter and set the event flag to start the sensor fusion task
		iCounter = 0;
		globals.RunKF_Event_Flag = 1;

	} // end of over-sampling test
}

// function runs the sensor fusion algorithms
void Fusion_Run(void)
{
	int8_t initiatemagcal;				// flag to initiate a new magnetic calibration

	// magnetic DOF: remove hard and soft iron terms from Bp (uT) to get calibrated data Bc (uT)
	fInvertMagCal(&thisMag, &thisMagCal);

	// update magnetic buffer checking for i) absence of first all-zero magnetometer output and ii) no calibration in progress
	// an all zero magnetometer reading can occur after power-on at rare intervals but it simply won't be used in the buffer
	if (!((globals.loopcounter < 100) && (thisMag.iBpFast[X] == 0) && (thisMag.iBpFast[Y] == 0) && (thisMag.iBpFast[Z] == 0)) && !thisMagCal.iCalInProgress)
	{
		// update the magnetometer measurement buffer integer magnetometer data (typically at 25Hz)
		iUpdateMagnetometerBuffer(&thisMagBuffer, &thisAccel, &thisMag, globals.loopcounter);
	}

	// 9DOF Accel / Mag / Gyro: apply the Kalman filter
	if (PARALLELNOTSEQUENTIAL || (globals.QuaternionPacketType == Q9))
	{
		// XXX thisSV_9DOF_GBY_KALMAN.systick = SYST_CVR & 0x00FFFFFF;
		fRun_9DOF_GBY_KALMAN(&thisSV_9DOF_GBY_KALMAN, &thisAccel, &thisMag, &thisGyro, &thisMagCal, THISCOORDSYSTEM, OVERSAMPLE_RATIO);
		// XXX thisSV_9DOF_GBY_KALMAN.systick -= SYST_CVR & 0x00FFFFFF;
		// XXX if (thisSV_9DOF_GBY_KALMAN.systick < 0) thisSV_9DOF_GBY_KALMAN.systick += SYST_RVR;
	}

	// 6DOF and 9DOF: decide whether to initiate a magnetic calibration
	// check no magnetic calibration is in progress
	if (!thisMagCal.iCalInProgress)
	{
		// do the first 4 element calibration immediately there are a minimum of MINMEASUREMENTS4CAL
		initiatemagcal = (!thisMagCal.iMagCalHasRun && (thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS4CAL));

		// otherwise initiate a calibration at intervals depending on the number of measurements available
		initiatemagcal |= ((thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS4CAL) && 
				(thisMagBuffer.iMagBufferCount < MINMEASUREMENTS7CAL) &&
				!(globals.loopcounter % INTERVAL4CAL));
		initiatemagcal |= ((thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS7CAL) &&
				(thisMagBuffer.iMagBufferCount < MINMEASUREMENTS10CAL) &&
				!(globals.loopcounter % INTERVAL7CAL));
		initiatemagcal |= ((thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS10CAL) &&
				!(globals.loopcounter % INTERVAL10CAL));

		// initiate the magnetic calibration if any of the conditions are met
		if (initiatemagcal)
		{
			// set the flags denoting that a calibration is in progress
			thisMagCal.iCalInProgress = 1;
			thisMagCal.iMagCalHasRun = 1;

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
		fUpdateCalibration4INV(pthisMagCal, pthisMagBuffer, &thisMag);
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
		fUpdateCalibration7EIG(pthisMagCal, pthisMagBuffer, &thisMag);
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
		fUpdateCalibration10EIG(pthisMagCal, pthisMagBuffer, &thisMag);
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
	itmp16 = thisAccel.iGpFast[X];
	thisAccel.iGpFast[X] = thisAccel.iGpFast[Y];
	thisAccel.iGpFast[Y] = itmp16;
#endif // NED
#if THISCOORDSYSTEM == ANDROID
	thisAccel.iGpFast[X] = -thisAccel.iGpFast[X];
	thisAccel.iGpFast[Y] = -thisAccel.iGpFast[Y];
#endif // Android
#if (THISCOORDSYSTEM == WIN8)
	thisAccel.iGpFast[Z] = -thisAccel.iGpFast[Z];
#endif // Win8

	return;
}

// function applies the hardware abstraction layer to the Fast (typically 200Hz) magnetometer readings
void ApplyMagHAL(struct MagSensor *pthisMag)
{
#if THISCOORDSYSTEM == NED
	int16_t itmp16;
	itmp16 = thisMag.iBpFast[X];
	thisMag.iBpFast[X] = -thisMag.iBpFast[Y];
	thisMag.iBpFast[Y] = -itmp16;
	thisMag.iBpFast[Z] = -thisMag.iBpFast[Z];
#endif // NED
#if THISCOORDSYSTEM == ANDROID
	thisMag.iBpFast[X] = -thisMag.iBpFast[X];
	thisMag.iBpFast[Y] = -thisMag.iBpFast[Y];	
#endif // Android
#if THISCOORDSYSTEM == WIN8
	thisMag.iBpFast[X] = -thisMag.iBpFast[X];
	thisMag.iBpFast[Y] = -thisMag.iBpFast[Y];
#endif

	// finally correct for the left handed magnetic coordinate system in MAG3110
#if defined USE_MAG3110
	thisMag.iBpFast[Z] = -thisMag.iBpFast[Z];
#endif

	return;
}

// function applies the hardware abstraction layer to the Fast (typically 200Hz) gyro readings
void ApplyGyroHAL(struct GyroSensor *pthisGyro, int16_t irow)
{
#if THISCOORDSYSTEM == NED
	int16_t itmp16;
	itmp16 = thisGyro.iYpFast[irow][X];
	thisGyro.iYpFast[irow][X] = -thisGyro.iYpFast[irow][Y];
	thisGyro.iYpFast[irow][Y] = -itmp16;
	thisGyro.iYpFast[irow][Z] = -thisGyro.iYpFast[irow][Z];
#endif // NED
#if THISCOORDSYSTEM == ANDROID
	thisGyro.iYpFast[irow][X] = -thisGyro.iYpFast[irow][X];
	thisGyro.iYpFast[irow][Y] = -thisGyro.iYpFast[irow][Y];
#endif // Android
#if THISCOORDSYSTEM == WIN8
	thisGyro.iYpFast[irow][X] = -thisGyro.iYpFast[irow][X];
	thisGyro.iYpFast[irow][Y] = -thisGyro.iYpFast[irow][Y];
#endif // Win8

	return;
}
