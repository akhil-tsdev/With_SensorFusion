/*
 * Copyright (c) 2017 TuringSense
 * All rights reserved.
 *
 * pivot3_0_poc.c
 *
 * File specific to PIVOT 3.0 POC
 */
#if HUB_NORDIC
#include "hub_main.h"
#endif

#if SAT_NORDIC
#include "satellite_main.h"
#endif

#include "common_types.h"
#include "pivot3_0_poc.h"

uint32_t cur_num_of_sensors = 16; 		/* Total sensors in the system */
uint8_t  cur_sensors_per_sat[MAX_SATELLITES] = {7,5,4,0,0,0};      /* Default Yoga POC */

uint8_t get_sensor_num_of_sat_idx(uint8_t sat_idx) {
  return cur_sensors_per_sat[sat_idx];
}

uint8_t get_starting_sensor_idx_of_sat_idx(uint8_t sat_idx) {
  uint8_t total_sensors_so_far = 0;
  uint8_t sat;
  
  /* If sat_idx is 0, then sensor index starts from 0 for sure. */
  if (sat_idx == 0) {
    return 0;
  }
  
  total_sensors_so_far += cur_sensors_per_sat[0];
  
  for (sat = 1; sat < cur_num_of_sat; sat++) {
    if (sat_idx == sat) {
      break;
    }
  
    total_sensors_so_far += cur_sensors_per_sat[sat];
  }

  return total_sensors_so_far;
}

uint8_t get_sat_idx_of_sensor_idx(uint8_t sensor_idx) {
  uint8_t total_sensors_so_far = 0;
  uint8_t sat_idx;

  for (sat_idx = 0; sat_idx < cur_num_of_sat; sat_idx++) {
    total_sensors_so_far += cur_sensors_per_sat[sat_idx];
    if (sensor_idx < total_sensors_so_far) {
      break;
    }
  }
  return sat_idx;
}
