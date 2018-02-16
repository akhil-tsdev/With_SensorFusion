/*
 * mpu9250_freq.h
 *
 *  Created on: Jul 24, 2015
 *      Author: cwati
 *
 *  This file defines the main frequencies of the sensors.
 */

#ifndef MPU9250_FREQ_H_
#define MPU9250_FREQ_H_


/* Frequency of Gyro */
#define TS_GYRO_FREQ			200	/* Hz, integer submultiple of 1000 */

#define PACE_SENSOR_OUTPUT		1	/* Pace so that sensor output is 1 every 10ms. Disable during R&D. */
#define TS_INITIAL_SENDING_FREQ	50	/* Hz.  If PACE_SENSOR_OUTPUT is enabled, we will only send this fast even though TS_GYRO_FREQ
								     * is faster. */
#if PIVOT_3_0
#define TS_INITIAL_SENDING_PERIOD  1 /* If satellite has 6 or more sensors, the generated data only comes at 50-60 Hz */
#else
#define TS_INITIAL_SENDING_PERIOD  (TS_GYRO_FREQ / TS_INITIAL_SENDING_FREQ)
#endif

/* Frequency of Mag */
#define TS_MAG_RATE				100	/* Hz, 8 or 100 */

#define TS_VALID_MAG_CAL		4	/* 0, 4, 7, 10.  Must set this to non-zero to enable the use of MAG data */


/* The "real" frequency of sensor */
#define TS_SENSORFS 			200 // int32: 1000Hz: frequency (Hz) of sensor sampling process

/* The oversampling ratio */
#define TS_OVERSAMPLE_RATIO 	2   // int32: 8x: 3DOF, 6DOF, 9DOF run at SENSORFS / OVERSAMPLE_RATIO Hz

#define MAX_OVERSAMPLE_RATIO	30  //Carmelo

#define MAX_COMPASS_SAMPLE_RATE	100
#if (TS_MAG_RATE > MAX_COMPASS_SAMPLE_RATE)
#  error "unsupported magnetometer sample rate"
#endif
#endif /* MPU9250_FREQ_H_ */
