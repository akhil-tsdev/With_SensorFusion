/*
 * Copyright � 2015 Turingsense, Inc.
 *
 * hub_main.h for Nordic
 *
 */
#ifndef __HUB_MAIN_H__
#define __HUB_MAIN_H__

#include "common_types.h"

// TODO: in a world where multiple hubs might exist, an additional
// "hub id" or "hub serial number" might be needed. But it should be a
// separate variable.
/// Whenever the system uses a device ID, satellites start at 1.
/// The hub is 0 by convention.
#define DEVICE_ID 				0

// Should probably be 15 in production. Or, perhaps, configured by Freescale.
#define TMP_NUM_OF_SAT 		MAX_SAT_SENSORS

// How many times RTC is sent out
#define RTC_SENDING_CNT 	5

extern uint32_t cur_num_of_sat;
extern uint32_t cur_sat_ids[MAX_SAT_SENSORS];

void set_sat_idx(uint32_t num_of_sat, uint32_t satellite_ids[MAX_SAT_SENSORS]);

#endif /* __HUB_MAIN_H__ */
