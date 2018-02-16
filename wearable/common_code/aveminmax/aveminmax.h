/*
 * Copyright © 2016 Turingsense, Inc.
 *
 * aveminmax.h
 *
 */
#ifndef __AVEMINMAX_H__
#define __AVEMINMAX_H__

#include <stdint.h>

uint32_t get_new_ave (uint32_t* current_total, uint32_t* current_cnt, uint32_t newval);
uint32_t get_new_max (uint32_t current_max, uint32_t newval);
uint32_t get_new_min (uint32_t current_min, uint32_t newval);

#endif /* __AVEMINMAX_H__ */
