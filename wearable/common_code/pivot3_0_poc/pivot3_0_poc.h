/*
 * Copyright (c) 2017 TuringSense
 * All rights reserved.
 *
 * pivot3_0_poc.h
 *
 */

#ifndef __PIVOT_3_0_POC_H__
#define __PIVOT_3_0_POC_H__

#include <stdint.h>

#if HUB_22F
#include "hub_main_loop.h"
#endif

extern uint32_t cur_num_of_sensors;
extern uint8_t  cur_sensors_per_sat[];

/* For PIVOT 3.0 POC
 *
 * Sat Index             Sensor Index
 * 0 ------------------- 0
 * 1 ------------------- 1
 *                       2
 *                       3
 *                       4
 * 2 ------------------- 5
 *                       6
 *                       7
 *                       8
 * 3 ------------------- 9
 *                       10
 *                       11
 *                       12
 *                       13
 * 4 ------------------- 14
 * 5 ------------------- 15
 */
 
 
/* Returns how many sensors for a specific sat */
uint8_t get_sensor_num_of_sat_idx(uint8_t sat_idx);

/* Returns the starting index for a specific sat */
uint8_t get_starting_sensor_idx_of_sat_idx(uint8_t sat_idx);

/* Returns sat index given a sensor index */
uint8_t get_sat_idx_of_sensor_idx(uint8_t sensor_idx);

#endif /* __PIVOT_3_0_POC_H__ */
