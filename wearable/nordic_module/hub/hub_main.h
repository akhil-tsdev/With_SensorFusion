/*
 * Copyright © 2015 Turingsense, Inc.
 *
 * hub_main.h
 *
 */
#ifndef __HUB_MAIN_H__
#define __HUB_MAIN_H__

#include "common_types.h"

// Should probably be 15 in production. Or, perhaps, configured by Freescale.
#define TMP_NUM_OF_SAT 		5

// How many times RTC is sent out
#define RTC_SENDING_CNT 	10

// How many times LCP value is sent out
#define LCP_SENDING_CNT		10

// How many times COM/VAL  is sent out
#define COM_SENDING_CNT		3

extern uint32_t cur_num_of_sat;
extern uint32_t cur_sat_ids[MAX_SATELLITES];
extern uint32_t new_lcp;
extern uint32_t my_id_hub;
extern uint32_t max_quiet_wait;

void set_sat_idx(uint32_t satellite_ids[MAX_SATELLITES]);
void update_cmd_to_sat(uint32_t command[SAT_COMMSTRUCT_COMMA]);
void reinit_esb(void);
void resetPacketCounter(void);

#endif /* __HUB_MAIN_H__ */
