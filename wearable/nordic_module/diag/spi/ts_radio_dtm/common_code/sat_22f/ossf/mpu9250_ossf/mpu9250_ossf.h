/*
 * mpu9250.h
 *
 *  Created on: Mar 27, 2015
 *      Author: eric
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include "mpu9250_freq.h"

// Configuration settings

#define DO_TEMP_COMP        0  /* not doing temperature compensation
                                  due to lack of calibration */

// GYRO_BANDWIDTH also affects temperature sensor bandwidth,
// and delay time for both gyro and temperature sensor
#define GYRO_BANDWIDTH     92  /* Hz, 5, 10, 20, 41, 92, or 184 */

#define GYRO_RANGE_DPS   2000  /* +/- 250, 500, 1000, 2000 */

// ACCEL_BANDWIDTH also affects delay time for accel
#define ACCEL_BANDWIDTH    92  /* Hz, 5, 10, 20, 41, 92, or 184 */

#define ACCEL_RANGE_G      16  /* +/- 2, 4, 8, 16 */

#define GYRO_ACCEL_RATE   TS_GYRO_FREQ  /* Hz, integer submultiple of 1000 */

// End of configuration settings

#if ACCEL_RANGE_G == 2
#  define MPU9250_COUNTSPERG (16384.0F)
#elif ACCEL_RANGE_G == 4
#  define MPU9250_COUNTSPERG (8192.0F)
#elif ACCEL_RANGE_G == 8
#  define MPU9250_COUNTSPERG (4096.0F)
#elif ACCEL_RANGE_G == 16
#  define MPU9250_COUNTSPERG (2048.0F)
#else
#  error "invalid ACCEL_RANGE_G"
#endif

#if GYRO_RANGE_DPS == 250
#  define MPU9250_COUNTSPERDEGPERSEC (131.0F)
#elif GYRO_RANGE_DPS == 500
#  define MPU9250_COUNTSPERDEGPERSEC (65.5F)
#elif GYRO_RANGE_DPS == 1000
#  define MPU9250_COUNTSPERDEGPERSEC (32.8F)
#elif GYRO_RANGE_DPS == 2000
#  define MPU9250_COUNTSPERDEGPERSEC (16.4F)
#else
#  error "invalid GYRO_RANGE_DPS"
#endif

#define MPU9250_UTPERCOUNT (0.15F)     		/* range for AK8963 magnetometer */

#define MPU9250_GPERCOUNT (1.0F / MPU9250_COUNTSPERG)
#define MPU9250_DEGPERSECPERCOUNT (1.0F / MPU9250_COUNTSPERDEGPERSEC)
#define MPU9250_COUNTSPERUT (1.0F / MPU9250_UTPERCOUNT)

#define SENSOR_ACCEL 1
#define SENSOR_GYRO  2
#define SENSOR_MAG   4
#define SENSOR_TEMP  8


#define MPU9250_REG_SELF_TEST_X_GYRO   0x00
#define MPU9250_REG_SELF_TEST_Y_GYRO   0x01
#define MPU9250_REG_SELF_TEST_Z_GYRO   0x02
#define MPU9250_REG_SELF_TEST_X_ACCEL  0x0d
#define MPU9250_REG_SELF_TEST_Y_ACCEL  0x0e
#define MPU9250_REG_SELF_TEST_Z_ACCEL  0x0f
#define MPU9250_REG_XG_OFFSET_H        0x13
#define MPU9250_REG_XG_OFFSET_L        0x14
#define MPU9250_REG_YG_OFFSET_H        0x15
#define MPU9250_REG_YG_OFFSET_L        0x16
#define MPU9250_REG_ZG_OFFSET_H        0x17
#define MPU9250_REG_ZG_OFFSET_L        0x18
#define MPU9250_REG_SMPLRT_DIV         0x19
#define MPU9250_REG_CONFIG             0x1a
#define MPU9250_REG_GYRO_CONFIG        0x1b
#define MPU9250_REG_ACCEL_CONFIG       0x1c
#define MPU9250_REG_ACCEL_CONFIG2      0x1d
#define MPU9250_REG_I2C_MST_CTRL       0x24
#define MPU9250_REG_I2C_SLV0_ADDR      0x25
#define MPU9250_REG_I2C_SLV0_REG       0x26
#define MPU9250_REG_I2C_SLV0_CTRL      0x27
#define MPU9250_REG_I2C_SLV1_CTRL      0x2a
#define MPU9250_REG_I2C_SLV2_CTRL      0x2d
#define MPU9250_REG_I2C_SLV3_CTRL      0x30
#define MPU9250_REG_I2C_SLV4_ADDR      0x31
#define MPU9250_REG_I2C_SLV4_REG       0x32
#define MPU9250_REG_I2C_SLV4_DO        0x33
#define MPU9250_REG_I2C_SLV4_CTRL      0x34
#define MPU9250_REG_I2C_SLV4_DI        0x35
#define MPU9250_REG_INT_PIN_CFG        0x37
#define MPU9250_REG_INT_ENABLE         0x38
#define MPU9250_REG_INT_STATUS         0x3a
#define MPU9250_REG_ACCEL_XOUT_H       0x3b
#define MPU9250_REG_TEMP_OUT_H         0x41
#define MPU9250_REG_GYRO_XOUT_H        0x43
#define MPU9250_REG_EXT_SENSE_DATA     0x49
#define MPU9250_REG_I2C_SLV0_DO        0x63
#define MPU9250_REG_I2C_MST_DELAY_CTRL 0x67
#define MPU9250_REG_SIGNAL_PATH_RESET  0x68
#define MPU9250_REG_USER_CTRL          0x6a
#define MPU9250_REG_PWR_MGMT_1         0x6b
#define MPU9250_REG_PWR_MGMT_2         0x6c
#define MPU9250_REG_WHO_AM_I		   0x75
#define MPU9250_REG_XA_OFFSET_H        0x77
#define MPU9250_REG_XA_OFFSET_L        0x78
#define MPU9250_REG_YA_OFFSET_H        0x7a
#define MPU9250_REG_YA_OFFSET_L        0x7b
#define MPU9250_REG_ZA_OFFSET_H        0x7d
#define MPU9250_REG_ZA_OFFSET_L        0x7e

#define AK8963_I2C_ADDR 0x0c

#define AK8963_REG_WAI             0x00
#define AK8963_REG_DEV_INFO        0x01
#define AK8963_REG_STATUS_1        0x02
#define AK8963_REG_HXL             0x03
#define AK8963_REG_HXH             0x04
#define AK8963_REG_HYL             0x05
#define AK8963_REG_HYH             0x06
#define AK8963_REG_HZL             0x07
#define AK8963_REG_HZH             0x08
#define AK8963_REG_STATUS_2        0x09
#define AK8963_REG_CNTL1           0x0a
#define AK8963_REG_ASAX            0x10
#define AK8963_REG_ASAY            0x11
#define AK8963_REG_ASAZ            0x12

#define AK8963_FIRST_REG           AK8963_REG_WAI
#define AK8963_REG_COUNT           ((AK8963_REG_STATUS_2 + 1) - AK8963_FIRST_REG)

#define MPU9250_FIRST_REG          MPU9250_REG_INT_STATUS
#define MPU9250_REG_COUNT          (MPU9250_REG_EXT_SENSE_DATA + AK8963_REG_COUNT - MPU9250_FIRST_REG)

// in self-test, don't read magnetometer
#define MPU9250_SELF_TEST_REG_COUNT (MPU9250_REG_EXT_SENSE_DATA - MPU9250_FIRST_REG)

#define MPU9250_REG_OFFSET(r) (r - MPU9250_FIRST_REG)
#define AK8963_REG_OFFSET(r) ((r - AK8963_FIRST_REG) + (MPU9250_REG_EXT_SENSE_DATA - MPU9250_FIRST_REG))

#define SMPLRT_DIV (((int)(1000 / GYRO_ACCEL_RATE))-1)
#define SELFTEST_SMPLRT_DIV (((int)(1000 / SELFTEST_GYRO_ACCEL_RATE))-1)


enum sensor_axis
{
	X,
	Y,
	Z
};

typedef struct
{
	int valid_sensors;  // bit field, SENSOR_x above
	uint32_t timestamp;
	int16_t accel[3];
	int16_t gyro[3];
	int16_t mag[3];
	float temp;
} sensor_data_t;

typedef struct
{
	uint8_t gyro[3];
	uint8_t accel[3];
} mpu9250_self_test_results_t;

typedef struct
{
	int16_t gyro[3];
	int16_t accel[3];
} mpu9250_average_data_t;

#endif /* MPU9250_H_ */
