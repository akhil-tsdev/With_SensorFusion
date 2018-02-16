/*
 * Copyright Turingsense © 2015
 *
 * hub_timer.h
 *
 */
#ifndef __HUB_TIMER_H__
#define __HUB_TIMER_H__
#include <stdint.h>

#define APP_TIMER_PRESCALER                  0   /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS                 3   /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE              4   /**< Size of timer operation queues. */

/* RTC get elapsed time */
uint32_t hub_timer_get_elapsed_ticks(void);
uint32_t hub_timer_get_elapsed_ms(void);

/* RTC stop timer */
uint32_t hub_timer_stop(void);

/* RTC start timer */
uint32_t hub_timer_start(void);

/* RTC
 * start timer to count ticks
 */
uint32_t hub_timer_init(void);
	
/* Start RTC */
void hub_timer_start_clock(void);
	
#endif  /* __HUB_TIMER_H__ */
