/*
 * Copyright (c) 2015 TuringSense
 *
 * main_22f_sensor.h
 *
 * Oct 7, 2015 - cwati
 */

#ifndef __MAIN_22F_NORDIC_BL_H__
#define __MAIN_22F_NORDIC_BL_H__

typedef enum {
	offLed,
	redLed,
	greenLed,
	blueLed,
	yellowLed,
	turqoiseLed,
	purpleLed,
	whiteLed
} sat_led_t;

typedef enum {
	bl_start = 0,
	bl_init1,
	bl_init2,
	bl_erase_load,
	bl_stop,
	bl_ready1,
	bl_ready2,
	bl_read,
	bl_num,
}bl_name ;

uint32_t nordic_bl_mode_time[bl_num];
uint32_t nordic_bl_mode_cnt[bl_num];

#endif /* __MAIN_22F_NORDIC_BL_H__ */
