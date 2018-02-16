/**
 ******************************************************************************
 * @file        MotionTL_Manager.c
 * @author      MEMS Application Team
 * @version     V1.0.0
 * @date        01-September-2017
 * @brief       This file includes Tilt Sensing interface functions
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
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "MotionTL_Manager.h"
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define CAL_DATA_NUM_RECORDS  100  /* Number of accelerometer data (3 axes per data) to be taken */

/** @addtogroup MOTION_TL_Applications
 * @{
 */

/** @addtogroup TILT_SENSING
 * @{
 */

/** @addtogroup MotionTL_Manager MotionTL_Manager
 * @{
 */

/* Public functions prototypes ---------------------------------------------*/

/**
 * @brief  Initialize the MotionTL engine
 * @param  None
 * @retval 1 in case of success,
 * @retval 0 otherwise
 */
uint8_t MotionTL_manager_init(void *handle)
{
  uint8_t instance;
  char acc_orientation[3];
  MTL_AccCal_t accCal;

  MotionTL_Initialize();
  BSP_ACCELERO_Get_Instance(handle, &instance);

  switch (instance)
  {

#if (defined (USE_IKS01A1))
    case LSM6DS0_X_0:
      acc_orientation[0] ='e';
      acc_orientation[1] ='n';
      acc_orientation[2] ='u';
      break;

    case LSM6DS3_X_0:
      acc_orientation[0] ='n';
      acc_orientation[1] ='w';
      acc_orientation[2] ='u';
      break;

    default:
      return 0;

#elif (defined (USE_IKS01A2))
    case LSM6DSL_X_0:
      acc_orientation[0] = 'n';
      acc_orientation[1] = 'w';
      acc_orientation[2] = 'u';
      break;

    case LSM303AGR_X_0:
      acc_orientation[0] = 'n';
      acc_orientation[1] = 'e';
      acc_orientation[2] = 'u';
      break;

    default:
      return 0;

#else
#error Not supported platform
#endif

  }

  MotionTL_SetOrientation_Acc(acc_orientation);

  // Get calibration values from flash memory and set them in library if valid
  MotionTL_manager_LoadCalValuesFromNVM(&accCal);
  MotionTL_SetCalValues(&accCal);

  return 1;
}


/**
 * @brief  Run Tilt Sensing algorithm
 * @param  data_in  Structure containing input data
 * @retval none
 */
void MotionTL_manager_run(MTL_input_t *data_in)
{
  MotionTL_Update(data_in);
}


/**
 * @brief  Get angles
 * @param  data_out   Structure containing output data
 * @param  angleMode  Switch mode to return desired angles
 * @retval none
 */
void MotionTL_manager_getAngles(MTL_output_t *data_out, MTL_AngleMode_t angleMode)
{
  MotionTL_GetAngles(data_out, angleMode);
}


/**
 * @brief  Get the library version
 * @param  version  Library version string (must be array of 35 char)
 * @param  length   Library version string length
 * @retval none
 */
void MotionTL_manager_get_version(char *version, int *length)
{
  *length = MotionTL_GetLibVersion(version);
}


/**
 * @brief  Calibrate accelerometer in specific position - collect data and pass them to library
 * @param  calPosition  Calibration position the data belong to
 * @retval None
 */
void MotionTL_manager_calibratePosition(MTL_CalPosition_t calPosition)
{
  float calData[CAL_DATA_NUM_RECORDS][3];

  if (CollectData(calData, CAL_DATA_NUM_RECORDS) == 0)
  {
    MotionTL_CalibratePosition(calData, CAL_DATA_NUM_RECORDS, calPosition);
  }
}


/**
 * @brief  Get accelerometer calibration values from library
 * @param  accCal  Pointer to calibration values structure
 * @retval Enum with calibration result
 */
MTL_CalResult_t MotionTL_manager_getCalibrationValues(MTL_AccCal_t *accCal)
{
  return MotionTL_GetCalValues(accCal);
}


/**
 * @brief  Set accelerometer calibration values into library
 * @param  accCal  Pointer to calibration values structure
 * @retval None
 */
void MotionTL_manager_setCalibrationValues(MTL_AccCal_t *accCal)
{
  MotionTL_SetCalValues(accCal);
}


/**
 * @brief  Get estimated measurement time
 * @param  time_s  Pointer to time in [s]
 * @retval None
 */
void MotionTL_manager_getEstimatedMeasTime(float *time_s)
{
  GetEstimatedMeasTime(time_s, CAL_DATA_NUM_RECORDS);
}


/**
 * @brief  Load calibration values from memory
 * @param  accCal  Pointer to structure with offset and gain values
 * @retval (1) fail, (0) success
 */
uint8_t MotionTL_manager_LoadCalValuesFromNVM(MTL_AccCal_t *accCal)
{
#if (defined (MOTION_TL_STORE_CALIB_FLASH))
  float data[6] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f};
  RecallCalibrationFromMemory(6, (uint32_t *)data);
  for (int i = 0; i < 3; i++)
  {
    accCal->offset[i] = data[i];
    accCal->gain[i] = data[i + 3];
  }
  return 0;
#else
  return 1;
#endif
}


/**
 * @brief  Save calibration values to memory
 * @param  accCal  Pointer to calibration values structure
 * @retval (1) fail, (0) success
 */
uint8_t MotionTL_manager_SaveCalValuesInNVM(MTL_AccCal_t *accCal)
{
#if (defined (MOTION_TL_STORE_CALIB_FLASH))
  float data[6] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f};
  for (int i = 0; i < 3; i++)
  {
    data[i] = accCal->offset[i];
    data[i + 3] = accCal->gain[i];
  }
  SaveCalibrationToMemory(6, (uint32_t *)data);
  return 0;
#else
  return 1;
#endif
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
