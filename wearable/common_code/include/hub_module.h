/* Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * hub_module.h
 *
 *  Created on: Mar 7, 2015
 *      Author: cwati
 */

#ifndef HUB_MODULE_H_
#define HUB_MODULE_H_
#include "common_types.h"

/*
 * Size without ENABLE_MAG
 * 4 ts + 6 accel + 6 gyro + 16 quat = 32 bytes
 * With MAG
 * 32 bytes + 6 = 38 bytes
 */
typedef struct cbuf_hub_ {
	uint32_t	timestamp;
	sensor_record_t record;
} cbuf_hub_t;

typedef enum hub_err_ {
	hub_no_err = 0,
	hub_hw_err1,
	hub_hw_err2,
	hub_hw_err3,
	hub_inv_err1,
	hub_inv_err2,
	hub_inv_err3
}hub_err_t ;
#endif /* HUB_MODULE_H_ */
