/*
 * Copyright (c) 2017 TuringSense
 *
 * lsm6dsl_firmware.h
 *
 * Interface to hw API talking to STM LSM6DSL sensor
 */

#ifndef __LS6DSL_FIRMWARE_H__
#define __LS6DSL_FIRMWARE_H__


#include "mpu9250_firmware.h"
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
#define LSM6DSL_FUNC_CFG_ACCESS 0x01



// For Magnetometer, this are the registers of the LSM6DSL


#define LSM6DSL_MASTER_CONFIG          0x1A
#define LSM6DSL_CTRL10_C					     0x19
#define LSM6DSL_SENSORHUB1_REG 				 0x2E
#define LSM6DSL_SENSORHUB2_REG 				 0x2F
#define LSM6DSL_SENSORHUB3_REG 				 0x30
#define LSM6DSL_SENSORHUB4_REG 				 0x31
#define LSM6DSL_SENSORHUB5_REG 				 0x32
#define LSM6DSL_SENSORHUB6_REG 				 0x33
#define LSM6DSL_SENSORHUB7_REG 				 0x34
#define LSM6DSL_SENSORHUB8_REG 				 0x35
#define LSM6DSL_SENSORHUB9_REG 				 0x36
#define LSM6DSL_SENSORHUB10_REG 				 0x37
#define LSM6DSL_SENSORHUB11_REG 				 0x38
#define LSM6DSL_SENSORHUB12_REG 				 0x39



#define LSM6DSL_OUT_MAG_RAW_X_L         0x66
#define LSM6DSL_OUT_MAG_RAW_X_H         0x67
#define LSM6DSL_OUT_MAG_RAW_Y_L         0x68
#define LSM6DSL_OUT_MAG_RAW_Y_H         0x69
#define LSM6DSL_OUT_MAG_RAW_Z_L         0x6A
#define LSM6DSL_OUT_MAG_RAW_Z_H         0x6B 


// Embedded functions register mapping for slave sensors 

#define LSM6DSL_SLV0_ADD                         0x02
#define LSM6DSL_SLV0_SUBADD                      0x03
#define LSM6DSL_SLAVE0_CONFIG                    0x04
#define LSM6DSL_DATAWRITE_SRC_MODE_SUB_SLV0      0x0E


// For the status of the Aux I2C bus

#define LSM6DSL_FUNC_SRC2                        0x54
#define LSM6DSL_FUNC_SRC1                        0x53



// Register maps of LIS2MDL

#define LIS2MDL_SLAVE_ADDRESS_READ    0x3D
#define LIS2MDL_SLAVE_ADDRESS_WRITE   0x3C


#define LIS2MDL_WHO_AM_I_REG          0x4F






bool lsm6dsl_detect_sensor(uint8_t cs, uint8_t* whoami);
bool lsm6dsl_init_regs(void);
bool lsm6dsl_start(void);
bool lsm6dsl_read(int sensors, sensor_data_t *sensor_data);


// Functions for LIS2MDL

void write_stm_mag_reg(uint8_t reg_addr, uint8_t data);
uint8_t read_stm_mag_reg(uint8_t reg_addr);
uint8_t  detect_stm_mag_sensor(void);
	

#endif /* __LS6DSL_FIRMWARE_H__ */
