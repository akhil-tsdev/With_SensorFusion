/*
 * Copyright � 2015 Turingsense, Inc.
 *
 * hub_main.h for Nordic
 *
 */
#ifndef __HUB_MAIN_H__
#define __HUB_MAIN_H__

#include "common_types.h"

// Should probably be 15 in production. Or, perhaps, configured by Freescale.
#define TMP_NUM_OF_SAT 		MAX_SENSORS

// How many times RTC is sent out
#define RTC_SENDING_CNT 	5

// How many times LCP value is sent out
#define LCP_SENDING_CNT		10

extern uint32_t cur_num_of_sat;
extern uint32_t cur_sat_ids[MAX_SENSORS];

void set_sat_idx(uint32_t num_of_sat, uint32_t satellite_ids[MAX_SENSORS]);

#endif /* __HUB_MAIN_H__ */