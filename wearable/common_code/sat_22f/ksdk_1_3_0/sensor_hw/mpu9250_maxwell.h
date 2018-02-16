/*
 * Copyright (c) 2017 TuringSense
 *
 * mpu9250_maxwell.h
 *
 * Interface to hw API talking to invensense, sensor mpu9250.
 */

#ifndef __MPU9250_MAXWELL_H__
#define __MPU9250_MAXWELL_H__

#include "fsl_dspi_master_driver.h"
#include "mpu9250_ossf.h"

//extern uint8_t valid_num_sensors_in_cs;
//extern uint8_t valid_cs[];

extern const dspi_device_t mpu9250_dspi_device;
extern const uint8_t invalid_maxwell_num;

void mpu9250_init(void);
bool mpu9250_init_all_pins(void);
bool mpu9250_init_aux_sensor(int);
bool mpu9250_init_regs(void);

bool mpu9250_start(void);

bool mpu9250_stop(void);

bool mpu9250_data_avail(void);

void mpu9250_clear_data_avail(void);
bool mpu9250_enable_sensor(uint8_t num);
bool mpu9250_detect_sensor(uint8_t cs, uint8_t* whoami);

// This only works when the mpu9250 has been initialized and started.
bool mpu9250_read(int sensors,  // bit field, SENSOR_x above
				  sensor_data_t *raw_data);

void mpu9250_sleep(void);

void mpu9250_wakeup(void);

void mpu9250_init_cs_gpio(void);
void update_maxwell_cs(uint8_t uart_num);
bool sat_enable_sensor(uint8_t sensor_num, uint8_t uart_num);

// Flag field in some self-test function can have any of the
// following bits ORed together.

// These flags indicate which devices to self-test
#define ST_FLAG_ACCEL 0x0001
#define ST_FLAG_GYRO  0x0002

#define ST_FLAG_DEBUG_PRINTF 0x0100


// This only works when mpu9250 has been initialized but is not started.
// returns the mpu9250 factory-programmed OTP contents, which are
// a logarithmically scaled version of the offsets measured between
// normal mode and self-test mode.
bool mpu9250_get_factory_self_test_results(mpu9250_self_test_results_t *factory_st);

// This only works when mpu9250 has been initialized but is not started.
// This should only be called when mpu9250 is stationary and has settled.
// Returns the average gyro and accel data in normal (non-self-test) mode,
// and in self-test mode.  The self-test response is defined as
// the selftest average minus the normal average, and should approximately
// match the factory self-test results.
// WARNING: clears any gyro and accelerometer offsets that have previously
// been set.
// WARNING: per the Invensense spec, these averages are collected with
// the gyro and accel sensitivities set to 250 dps and +/-2g.  This
// means that the normal output may not be directly suitable for input
// to the mpu9250_set_gyro_offsets() function.
bool mpu9250_self_test(int flags,
		               mpu9250_average_data_t *normal,
		               mpu9250_average_data_t *selftest);

// Determine whether the self-test results are consistent mpu9250 factory
// selftest OTP data. Returns true if match is acceptable.
uint32_t mpu9250_check_selftest_results(int flags,
		                            mpu9250_self_test_results_t *factory_st,
		                          	mpu9250_average_data_t *normal,
		                          	mpu9250_average_data_t *selftest);

void sensor_spi_set_csn(uint16_t cs, uint16_t val);
void sensor_read_regs(uint8_t reg_addr, size_t count, uint8_t *data);
void sensor_write_reg(uint8_t reg_addr, uint8_t data);

// Self-test gyro and accelerometer bandwidth and range per Invensense
// application note: "MPU-6500 Accelerometer and Gyroscope
// Self-Test Implementation"
// Document Number: AN-MPU-6500A-02
// Revision: 1.1
// Release Date: 5/30/2013

#define SELFTEST_GYRO_ACCEL_RATE  1000
#define SELFTEST_GYRO_BANDWIDTH   92  /* Hz, 5, 10, 20, 41, 92, or 184 */
#define SELFTEST_GYRO_RANGE_DPS   250  /* +/- 250, 500, 1000, 2000 */
#define SELFTEST_ACCEL_BANDWIDTH  92  /* Hz, 5, 10, 20, 41, 92, or 184 */
#define SELFTEST_ACCEL_RANGE_G    2   /* +/- 2, 4, 8, 16 */

#endif /* __MPU9250_MAXWELL_H__ */
