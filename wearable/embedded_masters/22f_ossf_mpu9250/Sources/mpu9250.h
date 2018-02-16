/*
 * mpu9250.h
 *
 *  Created on: Mar 27, 2015
 *      Author: eric
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#define MPU9250_COUNTSPERG (2048.0F)          /* +/- 16g range */
#define MPU9250_GPERCOUNT (1.0F / MPU9250_COUNTSPERG)

#define MPU9250_COUNTSPERDEGPERSEC (16.4F)    /* +/- 2000dps range */
#define MPU9250_DEGPERSECPERCOUNT (1.0F / MPU9250_COUNTSPERDEGPERSEC)

#define MPU9250_UTPERCOUNT (0.15F)     		/* range for AK8963 magnetometer */
#define MPU9250_COUNTSPERUT (1.0F / MPU9250_UTPERCOUNT)

#define SENSOR_ACCEL 1
#define SENSOR_GYRO  2
#define SENSOR_MAG   4
#define SENSOR_TEMP  8

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
	uint16_t temp;
} raw_sensor_t;

bool mpu9250_init(void);

bool mpu9250_data_avail(void);

void mpu9250_clear_data_avail(void);

bool mpu9250_read(int sensors,  // bit field, SENSOR_x above
				  raw_sensor_t *raw_data);

#endif /* MPU9250_H_ */
