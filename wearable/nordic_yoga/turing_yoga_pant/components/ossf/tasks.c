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

//#include "build.h"
#include "magnetic.h"
#include "tasks.h"


// new SF
#include "KF.h"
#include "orientationComputation.h"
#include "matrixComputations.h"
//#include "multichipKF.h"

struct ProjectGlobals globals;

// sensor data structures
struct AccelSensor thisAccel[MAX_SENSORS_IN_CS];				// this accelerometer



struct GyroSensor thisGyro[MAX_SENSORS_IN_CS];					// this gyro

// New KF struct gyr/acc only
KF accGyrKF;
KF_params accGyrKFpar;
refFrame UVRH;

KF_static accGyrKF_static[MAX_SENSORS_IN_CS];
refFrame_static UVRH_static[MAX_SENSORS_IN_CS];

float qbn[4];

uint8_t mpu9250_num = 0;										// The sensor number for PIVOT 3.0



void Fusion_Init1(KF * thisKF, KF_params * thisKFpar, refFrame * thisRefFrame, float stdg, float stda, float ca, float cb, float po, float Xdim, float Ydim)
{
  thisKFpar->stdg = stdg;
  thisKFpar->stds = stda;

  thisKFpar->ca = ca;
  thisKFpar->cb = cb;
  thisKFpar->po = po;

  thisKFpar->Xdim = Xdim;
  thisKFpar->Ydim = Ydim;


  // ref frame (Up Vertical, Right Horizontal)
  thisRefFrame->initialized = false;
  thisRefFrame->allocated = false;
  thisKF->allocated = false;

  // reset the default quaternion type to the simplest Q3 (it will be updated during the initializations)
  globals.DefaultQuaternionPacketType = Q3;

  // reset the loop counter to zero for first iteration
  for (int sn = 0; sn < MAX_SENSORS_IN_CS; sn++)
		globals.loopcounter[sn] = 0;

}

