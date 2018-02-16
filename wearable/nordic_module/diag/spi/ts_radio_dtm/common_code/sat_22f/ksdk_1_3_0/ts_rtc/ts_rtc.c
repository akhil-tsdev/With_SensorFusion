/*
 * Copyright (c) 2016 TuringSense
 *
 * ts_rtc.c
 *
 * May 12, 2016 - cwati
 */
#include <string.h>
#include <stdint.h>
#include "board.h"
#include "fsl_os_abstraction.h"
#include "ts_rtc.h"
#if PROTO_BOARD
#include "MK22F12810.h"			/* For RTC registers */
#else
#if PRODUCTION1
#include "MK22F25612.h"			/* For RTC registers */
#endif
#endif
/* We always reset time to 0 when set RTC is requested.
 * However, in the case that hub requests us to set RTC to a number other than 0,
 * we keep the diff here.
 */
static uint32_t hub_timestamp_diff = 0;
static uint32_t MSB = 0;
static bool init_was_requested = false;
static uint32_t last_osa_time = 0;

////debug
//uint32_t ts1, ts2, ts3;
//
//static uint32_t last_ts10 = 0;
//static uint32_t last_sec, last_tpr;
//static uint32_t last_total_ms;	//debug cw todo
//uint32_t ts[500] = { 0 }, ts_cnt = 0;

#if ENABLE_RTC
///*******************************************************************************
// * Enable K22F's RTC registers (TSR - seconds counter, TPR - prescaler counter
// * TSR is incremented every second
// * TPR is incremented every 32.768 kHz
// ******************************************************************************/
//void k22f_enable_rtc() {
//	return;
//}

/*******************************************************************************
 * Set K22F's RTC to reset any drift among satellite modules
 ******************************************************************************/
void k22f_set_rtc(uint32_t hub_rtc_milliseconds) {
	OSA_TimeInit();
//	init_was_requested = true;
	last_osa_time = 0;

	if (hub_rtc_milliseconds > 0xFFFF) {
		MSB = hub_rtc_milliseconds & (0xFFFF0000);
		hub_timestamp_diff = hub_rtc_milliseconds & (0x0000FFFF);
	} else {
		MSB = 0;
		hub_timestamp_diff = hub_rtc_milliseconds;
	}
}

/*******************************************************************************
 * Get K22F's RTC in milliseconds
 ******************************************************************************/
uint32_t k22f_get_rtc(void) {
	/* Despite the return value, this only returns 16-bit */
	uint32_t time_now = OSA_TimeGetMsec();
	uint32_t real_time;

	if (last_osa_time > time_now) {
		MSB += 0x10000;
	}
	last_osa_time = time_now;

	real_time = ((MSB & 0xFFFF0000) | (time_now & 0x0000FFFF));

	return (real_time + hub_timestamp_diff);
}
#else
void k22f_enable_rtc() {
	;
}
void k22f_set_rtc(uint32_t hub_rtc_milliseconds) {
	;
}
uint32_t k22f_get_rtc(void) {
	return 0;
}
#endif /* ENABLE_RTC */
