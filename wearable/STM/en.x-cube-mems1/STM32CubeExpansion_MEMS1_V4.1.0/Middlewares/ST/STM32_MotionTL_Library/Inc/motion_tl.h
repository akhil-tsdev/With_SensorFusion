/**
  ******************************************************************************
  * @file    motion_tl.h
  * @author  MEMS Application Team
  * @version V1.0.0
  * @date    01-September-2017
  * @brief   Header for motion_gr module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ********************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTION_TL_H_
#define _MOTION_TL_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup MOTION_TL MOTION_TL
  * @{
  */

/** @defgroup MOTION_TL_Exported_Types MOTION_TL_Exported_Types
 * @{
 */

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
 typedef struct
{
  float AccX;           /* Acceleration in X axis in [g] */
  float AccY;           /* Acceleration in Y axis in [g] */
  float AccZ;           /* Acceleration in Z axis in [g] */
  float deltatime_s;    /* Time between 2 library calls in [s] */
} MTL_input_t;

typedef struct
{
  float Angles_Array[3];    /* Either pitch, roll and gravity inclination or theta, psi and phi */
} MTL_output_t;

typedef struct
{
  float offset[3];
  float gain[3];
} MTL_AccCal_t;

typedef enum
{
  MODE_PITCH_ROLL_GRAVITY_INCLINATION,  /* pitch, roll and gravity inclination */
  MODE_THETA_PSI_PHI                    /* theta, psi and phi */
} MTL_AngleMode_t;

typedef enum
{
  X_UP,
  X_DOWN,
  Y_UP,
  Y_DOWN,
  Z_UP,
  Z_DOWN
} MTL_CalPosition_t;

typedef enum
{
  CAL_PASS,  /* Calibration passed */
  CAL_NONE,  /* Calibration not finished or not performed at all */
  CAL_FAIL   /* Calibration failed */
} MTL_CalResult_t;

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup MOTION_TL_Exported_Functions MOTION_TL_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize the MotionTL engine
 * @param  none
 * @retval none
 */
void MotionTL_Initialize(void);

/**
 * @brief  Set the MotionTL accelerometer data orientation
 * @param  *acc_orientation: reference system of the accelerometer raw data (for instance: south west up became "swu", north east up became "ned")
 * @retval none
 */
void MotionTL_SetOrientation_Acc(const char *acc_orientation);

/**
 * @brief  Run tilt algorithm
 * @param  data_in  Pointer to accaleration in [g]
 * @retval none
 */
void MotionTL_Update(MTL_input_t *data_in);

/**
 * @brief  Get angles
 * @param  data_out   Pointer to MTL_output_t structure
 * @param  angleMode  Switch mode to return desired angles
 * @retval none
 */
void MotionTL_GetAngles(MTL_output_t *data_out, MTL_AngleMode_t angleMode);

/**
 * @brief  Get the library version
 * @param  version  Pointer to an array of 35 char
 * @retval Number of characters in the version string
 */
uint8_t MotionTL_GetLibVersion(char *version);

/**
 * @brief  Calibrate accelerometer in specific position
 * @param  calData      Pointer to 2D array of calibration data calData[nRecords][3]
 * @param  nRecords     Number of calibration data records (3 axes per each record)
 * @param  calPosition  Calibration position the data belong to
 * @retval none
 */
void MotionTL_CalibratePosition(float calData[][3], uint32_t nRecords, MTL_CalPosition_t calPosition);

/**
 * @brief  Get accelerometer calibration values
 * @param  accCal  Pointer to calibration values structure
 * @retval Enum with calibration result
 */
MTL_CalResult_t MotionTL_GetCalValues(MTL_AccCal_t *accCal);

/**
 * @brief  Set accelerometer calibration values
 * @param  accCal  Pointer to calibration values structure
 * @retval Enum with calibration result
 */
MTL_CalResult_t MotionTL_SetCalValues(MTL_AccCal_t *accCal);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* _MOTION_TL_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
