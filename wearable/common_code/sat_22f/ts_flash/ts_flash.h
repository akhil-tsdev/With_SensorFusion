/*
 * Copyright (c) 2016 TuringSense
 *
 * ts_flash.h
 *
 */

#ifndef __TS_FLASH_H__
#define __TS_FLASH_H__

#include <stdint.h>
#include <stdbool.h>
#include "flash_demo.h"
#include "SSD_FTFx.h"
#include "common_err.h"

typedef volatile uint32_t 	vuint32_t;
typedef volatile uint16_t 	vuint16_t;
typedef volatile uint8_t 	vuint8_t;

/* Turingsense data is located at the last sector.
 * Size for flash is 256kb or 0x40000.  One sector is 0x800.
 * So the last sector is from address 0x40000 - 0x800 = 0x3F800.
 */
#define TS_FLASH_STARTING_ADDR	0x3f800
#define TS_FLASH_SAT_ID_ADDR	(TS_FLASH_STARTING_ADDR)		/* 32-bit */
#define TS_FLASH_ACCEL_X_ADDR	(TS_FLASH_STARTING_ADDR + 4)	/* 16-bit but stored in 32-bit */
#define TS_FLASH_ACCEL_Y_ADDR	(TS_FLASH_STARTING_ADDR + 8)
#define TS_FLASH_ACCEL_Z_ADDR	(TS_FLASH_STARTING_ADDR + 12)
#define TS_FLASH_GYRO_X_ADDR	(TS_FLASH_STARTING_ADDR + 16)
#define TS_FLASH_GYRO_Y_ADDR	(TS_FLASH_STARTING_ADDR + 20)
#define TS_FLASH_GYRO_Z_ADDR	(TS_FLASH_STARTING_ADDR + 24)
#define TS_FLASH_MAG_X_ADDR		(TS_FLASH_STARTING_ADDR + 28)
#define TS_FLASH_MAG_Y_ADDR		(TS_FLASH_STARTING_ADDR + 32)
#define TS_FLASH_MAG_Z_ADDR		(TS_FLASH_STARTING_ADDR + 36)
#define TS_FLASH_SENS_ACCEL_X_ADDR (TS_FLASH_STARTING_ADDR + 40)
#define TS_FLASH_SENS_ACCEL_Y_ADDR (TS_FLASH_STARTING_ADDR + 44)
#define TS_FLASH_SENS_ACCEL_Z_ADDR (TS_FLASH_STARTING_ADDR + 48)
#define TS_FLASH_HUBID_ADDR 	(TS_FLASH_STARTING_ADDR + 52)
#define TS_FLASH_MAG_FACTORY_X_ADDR		(TS_FLASH_STARTING_ADDR + 56)
#define TS_FLASH_MAG_FACTORY_Y_ADDR		(TS_FLASH_STARTING_ADDR + 60)
#define TS_FLASH_MAG_FACTORY_Z_ADDR		(TS_FLASH_STARTING_ADDR + 64)
#define TS_FLASH_LCP_ADDR		(TS_FLASH_STARTING_ADDR + 68)

/* The following only works starting from sensor 1!  Sensor 0 uses the
 * legacy allotted addresses above.
 */
#define TS_FLASH_SENS_BASE(a)		(uint32_t)(72 + ((a-1) * 60)) // 72, 132, 192, 252

#define TS_FLASH_ACCEL_X_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 0)
#define TS_FLASH_ACCEL_Y_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 4)
#define TS_FLASH_ACCEL_Z_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 8)
#define TS_FLASH_GYRO_X_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 12)
#define TS_FLASH_GYRO_Y_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 16)
#define TS_FLASH_GYRO_Z_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 20)
#define TS_FLASH_MAG_X_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 24)
#define TS_FLASH_MAG_Y_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 28)
#define TS_FLASH_MAG_Z_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 32)
#define TS_FLASH_SENS_ACCEL_X_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 36)
#define TS_FLASH_SENS_ACCEL_Y_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 40)
#define TS_FLASH_SENS_ACCEL_Z_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 44)
#define TS_FLASH_MAG_FACTORY_X_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 48)
#define TS_FLASH_MAG_FACTORY_Y_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 52)
#define TS_FLASH_MAG_FACTORY_Z_ADDR_N(a)	(TS_FLASH_STARTING_ADDR + a + 56)

err_t ts_flash_init();
err_t ts_write_to_flash(uint32_t destination, uint32_t size, uint8_t* program_buffer);
err_t ts_erase_flash_sector();
uint32_t ts_read_from_flash(uint32_t dest);

#endif /* __TS_FLASH_H__ */
