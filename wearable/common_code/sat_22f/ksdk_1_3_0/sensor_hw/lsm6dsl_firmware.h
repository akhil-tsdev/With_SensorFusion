/*
 * Copyright (c) 2017 TuringSense
 *
 * lsm6dsl_firmware.h
 *
 * Interface to hw API talking to to STM LSM6DSL
 */

#ifndef __LS6DSL_FIRMWARE_H__
#define __LS6DSL_FIRMWARE_H__

#include <stdio.h>
#include "mpu9250_maxwell.h"
#include "sat_module.h"

#define LSM6DSL_READ_SIZE				100 /* cwati todo find out exact size we need */

#define LSM6DSL_FIFO_CTRL1				0x06
#define LSM6DSL_FIFO_CTRL5				0x0A
#define LSM6DSL_INT1_CTRL				0x0D
#define LSM6DSL_INT2_CTRL				0x0E
#define LSM6DSL_REG_WHOAMI				0x0F
#define LSM6DSL_CTRL1_XL				0x10
#define LSM6DSL_CTRL2_G					0x11
#define LSM6DSL_CTRL3_C					0x12
#define LSM6DSL_OUTX_L_G				0x22
#define LSM6DSL_TAP_CFG					0x58
#define LSM6DSL_TAP_THS_6D				0x59
#define LSM6DSL_INT_DUR2				0x5A
#define LSM6DSL_FREE_FALL				0x5D
#define LSM6DSL_MD1_CFG					0x5E
#define LSM6DSL_MD2_CFG					0x5F
#define LSM6DSL_WHOAMI					0x6A


bool lsm6dsl_detect_sensor(uint8_t cs, uint8_t* whoami);
bool lsm6dsl_init_regs(void);
bool lsm6dsl_start(void);
bool lsm6dsl_read(int sensors, sensor_data_t *sensor_data);
bool lsm6dsl_spi_test(void);

#endif
