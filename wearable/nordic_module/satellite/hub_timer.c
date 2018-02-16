/*
 * Copyright Turingsense © 2015
 *
 * hub_timer.c
 *
 */
 
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "app_timer.h"			/* RTC */
#include "hub_timer.h"
#include "nrf.h"

/* RTC */	
static app_timer_id_t m_empty_timeout_timer_id;

/* RTC 
 * Dummy timer handler
 */
static void empty_timeout_handler (void *ptr) {
	/* dummy */
}

/* RTC get elapsed ticks */
uint32_t hub_timer_get_elapsed_ticks() {
		uint32_t p_ticks;
	
		app_timer_cnt_get(&p_ticks);
	
		return(p_ticks);
}

/* RTC get elapsed time */
uint32_t hub_timer_get_elapsed_ms() {
		uint32_t p_ticks;
	
		app_timer_cnt_get(&p_ticks);
	
		/* Timer has expired */
		if (p_ticks == 0) {
			hub_timer_stop();
			hub_timer_start();
		}
		
	/* Ticks / (ticks per ms) */
		return (p_ticks / (APP_TIMER_CLOCK_FREQ / 1000));
}

/* RTC stop timer */
uint32_t hub_timer_stop() {
	return app_timer_stop(m_empty_timeout_timer_id);
}

/* RTC start timer */
uint32_t hub_timer_start() {
	uint32_t err_code;
	
	// 0x00FFFFFF is MAX_RTC_COUNTER_VAL from app_timer.c 
	const uint32_t timeout = 	(0x00FFFFFF / 2); 

	err_code = app_timer_start(m_empty_timeout_timer_id, timeout, NULL); 
	return err_code;
}

/* RTC
 * start timer to count ticks
 */
uint32_t hub_timer_init() {
		uint32_t err_code;
			
	  /* Initialize timer module */
		APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS,
			APP_TIMER_OP_QUEUE_SIZE, false);
	
		err_code = app_timer_create(&m_empty_timeout_timer_id, APP_TIMER_MODE_REPEATED, empty_timeout_handler);
		return err_code;
}
	
/* Start RTC */
void hub_timer_start_clock(void)
{
    NRF_CLOCK->LFCLKSRC             = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_LFCLKSTART     = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
}
