/*
 * Copyright TuringSense, Inc © 2015-2015
 * hub_main_loop_dtm.h
 *
 *  Created on: June 25, 2016
 *      Author: cwati
 *
 *
 */

#ifndef HUB_MAIN_LOOP_DTM_H_
#define HUB_MAIN_LOOP_DTM_H_

#include <ts_io_led.h>
#include "common_types.h"
#include "cbuf.h"

/* DTM */
#include "dtm_common_types.h"

#define TMPDEBUG			1
#define DEBUG				0
#define DEBUG1				0
#define DEBUG2				1
#define DEBUG_DELAY			0

#define NORDIC_RST			(GPIO_PORT_B|GPIO_PIN19) /* PTB19 Nordic soft reset */

#define MAX(a,b) ((a>b)?(a):(b))
#define SPI_COMM_LENGTH MAX(sizeof(hub_to_nordic_dtm_t),sizeof(nordic_to_hub_dtm_t))


/* Some error check */
#if PRODUCTION1 && EVAL_BOARD
#error "You can't choose both production and eval board!"
#endif

#define NORDIC_LOOP_DELAY	5		/* Delay in ms in main loop for each iteration */

#define INITIAL_SENDING_DELAY 250	/* Delay in ms, discard first few data after reset RTC TODO check this*/

//extern uint16_t  sending_delay_cnt;
extern cloud_to_hub_t	cloud_to_hub;
extern hub_to_nordic_t 	hub_to_nordic;

extern nordic_to_hub_t 	nordic_to_hub;
extern hub_to_cloud_t 	hub_to_cloud;
extern cbuf_t cbuf[MAX_SENSORS];
extern uint32_t 		current_state;

extern TIME_STRUCT		latest_rtc;
extern uint32_t 		cur_num_of_sat;
extern uint32_t 		cur_sat_ids[MAX_SENSORS - 1];
extern uint32_t 		cur_hub_id;

extern uint32_t latest_ts[MAX_SENSORS];
extern const uint8_t enable_stats;
  
//cw debug timestamp
extern uint32_t	*ts_tmp;

/* Active or not active is whether we are recording or not */
#define HUB_ACTIVE(c)					(c & WIFI_START)
#define HUB_NOT_ACTIVE(c)				((c & WIFI_START) == 0)		/* Sleep */

/* Can or cannot send is whether we can send to cloud */
#define HUB_CAN_SEND(c)					((c & WIFI_WAIT) == 0)
#define HUB_CANNOT_SEND(c)				(c & WIFI_WAIT)

#define COMMAND_WIFI_INVALID(c) 	(c & WIFI_INVALID)
#define COMMAND_WIFI_START(c) 		(c & WIFI_START)
#define COMMAND_WIFI_WAIT(c) 		(c & WIFI_WAIT)
#define COMMAND_WIFI_CALIBRATE(c) 	(c & WIFI_CALIBRATE)
#define COMMAND_WIFI_SET_RTC(c) 	(c & WIFI_SET_RTC)
#define COMMAND_WIFI_SET_SAT(c) 	(c & WIFI_SET_SATELLITES)
#define COMMAND_WIFI_SET_LCP(c) 	(c & WIFI_SET_LCP)
#define COMMAND_WIFI_SET_CHANNEL(c) (c & WIFI_SET_CHANNEL)

//TODO hardcode IP during dev
//#define default_destIP		"192.168.1.126"
//#define	default_destPort 	"1234"
#define txSize		sizeof(hub_to_cloud)
#define txInterval	TX_DELAY							/* delay between packets in ms.
												 * If you don't put this things might break!! */
#define txType		TCP_TX
#define txMode		PACKET_TEST
#define txPktNum	1
#define UNITY_CONNECT_TOUT			60*1		/* 1 minute(s) to try/retry connection to Unity */

uint32_t max_cbuf_fill_pct();

uint32_t hub_active (uint32_t c);
uint32_t hub_not_active (uint32_t c);
uint32_t hub_can_send (uint32_t c);
uint32_t hub_cannot_send (uint32_t c);

void set_hub_can_send(uint32_t* c);
void set_hub_cannot_send(uint32_t* c);
void set_hub_active(uint32_t* c);
void set_hub_not_active(uint32_t* c);

void blinkLEDerror(hub_led_t color, uint32_t multiplier);
void turn_on_LED(hub_led_t led_color);

void hub_dtm_main_loop(uint32_t);

void assert_nordic_reset();
uint32_t all_cbuf_fill_pct(void);

#endif /* HUB_MAIN_LOOP_DTM_H_ */
