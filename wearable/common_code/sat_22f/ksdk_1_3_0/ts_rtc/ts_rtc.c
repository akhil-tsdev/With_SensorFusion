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
#if CPU_MK22FN1M0AVMC12
#include "MK22FA12.h"
#elif CPU_MK22FN256VDC12
#include "MK22F25612.h"			/* For RTC registers */
#endif
#endif

#include "common_types.h"

#if ENABLE_RTC
/* We always reset time to 0 when set RTC is requested.
 * However, in the case that hub requests us to set RTC to a number other than 0,
 * we keep the diff here.
 */

static uint32_t last_ts10 = 0;
static uint32_t last_sec, last_tpr;
static uint32_t last_total_ms;	//debug cw todo
uint32_t ts[500] = { 0 }, ts_cnt = 0;

void k22f_enable_rtc() {
	SIM_SCGC6|= SIM_SCGC6_RTC_MASK;

	// Enable 32.768 kHz oscillator
	RTC_BWR_CR_OSCE(RTC, 1);
	// Wait 100ms for oscillator output to settle down
	OSA_TimeDelay(100);

	// Disable time counters TSR and TPR before writing
	RTC_BWR_SR_TCE(RTC, 0);
	// Clear the RTC_TPR counter
	RTC_CLR_TPR(RTC, 0xFFFFFFFFU);
	// Write the RTC_TSR with 0 second value
	RTC_WR_TSR(RTC, 0);
	// Enable time counters TSR and TPR after writing
	RTC_BWR_SR_TCE(RTC, 1);

	last_sec = 0;
	last_tpr = 0;
	last_total_ms = 0;

//	// Enable 32.768 kHz oscillator
//	BW_RTC_CR_OSCE(RTC_BASE, 1);
//	// Wait 100ms for oscillator output to settle down
//	OSA_TimeDelay(100);
//	// Disable time counters TSR and TPR before writing
//	BW_RTC_SR_TCE(RTC_BASE, 0);
//	// Clear the RTC_TPR counter
//	HW_RTC_TPR_CLR(RTC_BASE, 0xFFFFFFFFU);
//	// Write the RTC_TSR with 0 second value
//	HW_RTC_TSR_WR(RTC_BASE, 0);
//	// Enable time counters TSR and TPR after writing
//	BW_RTC_SR_TCE(RTC_BASE, 1);
//
//	last_sec = 0;
//	last_tpr = 0;
//	last_total_ms = 0;

}
void k22f_set_rtc(uint32_t hub_rtc_milliseconds) {
	uint32_t rtc_milliseconds = hub_rtc_milliseconds % 1000;
	last_sec = (hub_rtc_milliseconds - rtc_milliseconds) / 1000;
	// TPR = 1 equals to ~30.5us = 1000000us / 32768Hz = (30.5/1000)ms = (1000/32768)ms
	last_tpr = (uint32_t) ((float) (rtc_milliseconds * 32768) / 1000);
	last_total_ms = (last_sec * 1000) + rtc_milliseconds;

	// Disable time counters TSR and TPR before writing
	RTC_BWR_SR_TCE(RTC, 0);
	// Write the RTC_TPR counter first
	RTC_WR_TPR(RTC, last_tpr);
	// Write the RTC_TSR with Hub's RTC's seconds value
	RTC_WR_TSR(RTC, last_sec);
	// Re-Enable time counters TSR and TPR after writing
	RTC_BWR_SR_TCE(RTC, 1);
}
uint32_t k22f_get_rtc(void) {

	uint32_t dummy_tpr, dummy_tpr1, current_tpr;
	uint32_t rtc_milliseconds;
	uint32_t dummy_rtc, current_sec;
	uint32_t cnt;
	const uint32_t cnt_max = 0xFF;

	//debug

	uint32_t total_ms_diff, current_total_ms;

	/* WARNING: There will be consequences if you change the order of the reading!
	 * Make sure you know what you're doing.
	 *
	 * For example:
	 * At time 0 second, 0x7FFF
	 * Read TPR: get 0x7FFF
	 * Read second: get 1 (already incremented)
	 * So you need to fix it.
	 *
	 * If you read second first
	 * second: get 0,
	 * TPR: get 0 (already incremented).
	 * Must handle these cases.
	 */
	/* We have to read twice or more until we get same values */
	dummy_tpr = RTC_RD_TPR(RTC);
	current_tpr = RTC_RD_TPR(RTC);
	cnt = 0;
	while (dummy_tpr != current_tpr) {
		if (++cnt == cnt_max) {
			/* Put a count to avoid infinite loop.  Shouldn't arrive here unless HW
			 * goes kaput. */
			return UINT32_MAX;
		}
		dummy_tpr = RTC_RD_TPR(RTC);
		current_tpr = RTC_RD_TPR(RTC);
	}

	/* We have to read twice or more until we get same values */
	dummy_rtc = RTC_RD_TSR(RTC);
	current_sec = RTC_RD_TSR(RTC);
	cnt = 0;
	while (dummy_rtc != current_sec) {
		if (++cnt == cnt_max) {
			/* Put a count to avoid infinite loop.  Shouldn't arrive here unless HW
			 * goes kaput. */
			return UINT32_MAX;
		}
		dummy_rtc = RTC_RD_TSR(RTC);
		current_sec = RTC_RD_TSR(RTC);
	}

	/* Check again.
	 * The hardware is bad.
	 *
	 * On our first read, say time was
	 * 0/7FFF
	 *
	 * It is possible that the above read yields:
	 * 7FFF, then 0 (which is correct)
	 * or
	 * 7FFF, then 1  --> when second has incremented.
	 *
	 */
	dummy_tpr = RTC_RD_TPR(RTC);
	dummy_tpr1 = RTC_RD_TPR(RTC);
	while (dummy_tpr != dummy_tpr1) {
		if (++cnt == cnt_max) {
			/* Put a count to avoid infinite loop.  Shouldn't arrive here unless HW
			 * goes kaput. */
			return UINT32_MAX;
		}
		dummy_tpr = RTC_RD_TPR(RTC);
		dummy_tpr1 = RTC_RD_TPR(RTC);
	}

	if (dummy_tpr < current_tpr) {
		current_sec = current_sec - 1;
	}

	// TPR = 1 equals to ~30.5us = 1000000us / 32768Hz = (30.5/1000)ms = (1000/32768)ms
	rtc_milliseconds = current_tpr * 1000 / 32768;
	current_total_ms = (current_sec * 1000) + rtc_milliseconds;

	/* In the case that timer has reached max (max is the same with INVALID_TIMESTAMP,
	 * then use current_total_ms calculation.
	 */
	if ((current_total_ms < last_total_ms)
			&& (last_total_ms != INVALID_TIMESTAMP)) {
		/* Very bad.  HW problem.  Not much we could do.
		 * Let's just return last value. */
		return last_total_ms;
	} else {
		last_sec = current_sec;
		last_tpr = current_tpr;
		last_total_ms = current_total_ms;

		return current_total_ms;
	}
// DEBUG
//	static count = 1;
//	static uint32_t prevtimeMillisecond = 0;
//	uint32_t timeMillisecond = current_sec*1000 + rtc_milliseconds;
//	if ((count % 10) == 0) {
//		count = 1;
//	} else {
//		count++;
//	}
//	prevtimeMillisecond = timeMillisecond;
//	return timeMillisecond;
}

#else
static uint32_t hub_timestamp_diff = 0;
static uint32_t MSB = 0;
static bool init_was_requested = false;
static uint32_t last_osa_time = 0;


/*******************************************************************************
 * Enable K22F's RTC registers (TSR - seconds counter, TPR - prescaler counter
 * TSR is incremented every second
 * TPR is incremented every 32.768 kHz
 ******************************************************************************/
void k22f_enable_rtc() {
	return;
}

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
#endif /* ENABLE_RTC */
