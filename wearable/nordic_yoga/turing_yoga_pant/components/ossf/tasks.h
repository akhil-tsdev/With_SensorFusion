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
#ifndef TASKS_H
#define TASKS_H

//CUSTOM
#include "build.h"
#include "common_types.h"
#include "magnetic.h"
//END CUSTOM

// New SF
#include "KF.h"
#include "matrixComputations.h"
#include "orientationComputation.h"
#include "multichipKF.h"

// structure definitions

// project globals structure
struct ProjectGlobals
{
	volatile bool RunKF_Event_Flag[MAX_SENSORS_IN_CS];
	volatile bool MagCal_Event_Flag[MAX_SENSORS_IN_CS];

	// bluetooth quaternion type
	quaternion_type QuaternionPacketType;

	// default quaternion type
	quaternion_type DefaultQuaternionPacketType;

	// global counter incrementing each iteration of sensor fusion (typically 25Hz)
	int32_t loopcounter[MAX_SENSORS_IN_CS];
	//volatile bool enable_MagBiasSending; //CARMELO It sends the calibration values of the magnetometer, rather than the values of the quaternion
	volatile bool disable_magCalibrationAlways; //CARMELO It is used to sends the calibration values of the magnetometer setted by, rather than the values of the quaternion
	volatile bool enable_MagBiasCalibrationAlways; //CARMELO Even if set the fixed values of BIAS, calibration of the magnetometer remains active. In this case the calibration values will be recalculated

	volatile uint16_t outputMode;	// GABRIELE - enables different output modes (gyro and acc and mag bias output);
									// It encompasses also the old enable_magBiasSending

	volatile bool alternateQuaternionMag;
	volatile bool newKF_enabled;
};

// globals defined in mqx_tasks.c
extern struct ProjectGlobals globals;

// New SF
extern KF accGyrKF;
extern KF_params accGyrKFpar;
extern refFrame UVRH;
extern float qbn[4];

extern KF_static accGyrKF_static[MAX_SENSORS_IN_CS];
extern refFrame_static UVRH_static[MAX_SENSORS_IN_CS];




// globals defined in tasks_func.c declared here for use elsewhere
extern struct AccelSensor thisAccel[];
extern struct GyroSensor thisGyro[];

extern uint8_t mpu9250_num;

extern int communicationTestCounter[MAX_SENSORS][2];
extern float adcValue;
#define COMMUNICATIONCOUNTER_BUFF 0
#define COMMUNICATIONCOUNTER_SPIP 1


// function prototypes for functions in tasks_func.c
void ApplyAccelHAL(struct AccelSensor *pthisAccel);
void ApplyMagHAL(struct MagSensor *pthisMag);
void ApplyGyroHAL(struct GyroSensor *pthisGyro, int16_t irow);
void RdSensData_Init(void);
void RdSensData_Run(uint16_t *accel, uint16_t *gyro, uint16_t *mag);

// New SF
void Fusion_Init1(KF * thisKF, KF_params * thisKFpar, refFrame * thisRefFrame, float stdg, float stda, float ca, float cb, float po, float Xdim, float Ydim);
void Fusion_Run(int16_t *accel, int16_t *gyro, int16_t *mag);
void MagCal_Run(struct MagCalibration *pthisMagCal, struct MagneticBuffer *pthisMagBuffer);

#endif   // #ifndef TASKS_H
