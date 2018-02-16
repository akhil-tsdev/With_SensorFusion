/*
 * Copyright TuringSense, Inc © 2015
 * hub_main_loop.h
 *
 *  Created on: May 11, 2015
 *      Author: cwati
 */

#ifndef HUB_MAIN_LOOP_H_
#define HUB_MAIN_LOOP_H_

//#include "throughput.h"
#include "common_types.h"
#include "cbuf.h"

extern cloud_to_hub_t	cloud_to_hub;
extern hub_to_nordic_t 	hub_to_nordic;

extern nordic_to_hub_t 	nordic_to_hub;
extern hub_to_cloud_t 	hub_to_cloud;
extern cbuf_t cbuf;

extern TIME_STRUCT		latest_rtc;
extern uint32_t 		cur_num_of_sat;
extern uint32_t 		cur_sat_ids[MAX_SENSORS - 1];

/* Active or not active is whether we are recording or not */
#define HUB_NOT_ACTIVE      0		/* Sleep */
#define HUB_SLEEP      		HUB_NOT_ACTIVE
#define HUB_ACTIVE          1

/* Can or cannot send is whether we can send to cloud */
#define HUB_CAN_SEND		1
#define HUB_CANNOT_SEND		0

#define COMMAND_WIFI_INVALID(c) 	(c & WIFI_INVALID)
#define COMMAND_WIFI_START(c) 		(c & WIFI_START)
#define COMMAND_WIFI_STOP(c) 		(c & WIFI_STOP)
#define COMMAND_WIFI_WAIT(c) 		(c & WIFI_WAIT)
#define COMMAND_WIFI_CALIBRATE(c) 	(c & WIFI_CAIBRATE)
#define COMMAND_WIFI_SET_RTC(c) 	(c & WIFI_SET_RTC)
#define COMMAND_WIFI_SET_SAT(c) 	(c & WIFI_SET_SATELLITES)

//TODO hardcode IP during dev
//#define default_destIP		"192.168.1.126"
#define	default_destPort 	"1234"
#define txSize		sizeof(hub_to_cloud)
#define txInterval	0							/* delay between packets in ms*/
#define txType		"TCP"
#define txMode		PACKET_TEST
#define txPktNum	1


uint32_t hub_start_loop(uint32_t argc, char *argv[] );

#endif /* HUB_MAIN_LOOP_H_ */
