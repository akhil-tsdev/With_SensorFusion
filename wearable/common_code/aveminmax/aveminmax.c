/*
 * Copyright © 2016 Turingsense, Inc.
 *
 * aveminmax.c
 *
 * Feb 17, 2016.  This file is simply calculating the ave, max, and min of data.
 *
 */
#include <stdint.h>

uint32_t get_new_ave (uint32_t* current_total, uint32_t* current_cnt, uint32_t newval) {
	uint32_t average = 0;
	*current_total += newval;
	*current_cnt += 1;
	
	if (*current_cnt == 1) {
		average = newval;
	} else {
		average = *current_total / *current_cnt;
	}
	return average;
}

uint32_t get_new_max (uint32_t current_max, uint32_t newval) {
	uint32_t new_max = current_max;
	
	if (newval > current_max) {
		new_max = newval;
	}
	
	return new_max;
}

uint32_t get_new_min (uint32_t current_min, uint32_t newval) {
	uint32_t new_min = current_min;
	
	if (newval < current_min) {
		new_min = newval;
	}
	
	return new_min;
}
