/*
 * Copyright TuringSense, Inc © 2015
 * atheros_main_loop.h
 *
 *  Created on: Nov 18, 2015
 *      Author: cwati
 */

#ifndef ATHEROS_MAIN_LOOP_H_
#define ATHEROS_MAIN_LOOP_H_
#include <lwevent.h>
#include <mutex.h>
#include <stdint.h>
#include "mpu9250_freq.h"

#define ENABLE_WIFI		1
#define ENABLE_WIFI_TX		1

#define MAX_WIFI_CHANNEL        11
#define WIFI_START_CHANNEL      10

#define SAT_SENDING_FREQ	TS_INITIAL_SENDING_FREQ

#if (SAT_SENDING_FREQ == 50)
/* The number of times we talk to Nordic consecutively */
#define NORDIC_REPEAT_6         10
#define NORDIC_REPEAT_5         5
#define NORDIC_REPEAT_DEFAULT   NORDIC_REPEAT_5

/* The number of times we talk to Wifi consecutively */
#define WIFI_REPEAT_6           5
#define WIFI_REPEAT_5           5
#define WIFI_REPEAT_DEFAULT     WIFI_REPEAT_5

/* The number of cycles we wait when bitmap to send via Wifi is incomplete */
#define WAIT_FOR_COMPLETE_6     3
#define WAIT_FOR_COMPLETE_5     3
#define WAIT_FOR_COMPLETE_DEF   WAIT_FOR_COMPLETE_5

#else /* Other (faster) than 50 Hz */
#define NORDIC_REPEAT_6         1
#define NORDIC_REPEAT_5         1
#define NORDIC_REPEAT_DEFAULT   NORDIC_REPEAT_5

#define WIFI_REPEAT_6           3
#define WIFI_REPEAT_5           3
#define WIFI_REPEAT_DEFAULT     WIFI_REPEAT_5

#define WAIT_FOR_COMPLETE_6     3
#define WAIT_FOR_COMPLETE_5     3
#define WAIT_FOR_COMPLETE_DEF   WAIT_FOR_COMPLETE_5

#endif /* SAT_SENDING_FREQ == 50 */

#define WIFI_MAX_REPEAT 70

/* If you set FAST_DRAIN to 1, it will not send any more sensor data to hub. */
#define FAST_DRAIN              1

extern char ssid_name[];

/* When we use WIFI DIRECT, the PORT NUMBER will be 0.  This means we will use the
 * default MY_PORT_STR.  For anything beside 0, this will be the port number used. */
extern uint32_t hubType_comPort;
#define HUB_TYPE_DIRECT     0     /* Wifi direct */

/* This IP Address field only matters if we are connecting to a router.
 * This will be the requested static IP Address.
 * If we are using WIFI DIRECT (ie *HUB_TYP_OFF = 0) then the IP will be the default
 * set by Atheros, i.e.:
 *  #define				MY_IP			"192.168.1.10"
 *  #define				MY_NETMASK		"255.255.255.0"
 *  #define				MY_GATEWAY		"192.168.1.1"
 */
extern uint32_t hubType_IPAdress;

//extern uint8_t hvac_init;
extern MUTEX_STRUCT cbuf_mutex;
extern MUTEX_STRUCT ack_to_client_mutex;
extern LWEVENT_STRUCT atheros_task_event;
extern uint32_t ack_to_client;

extern uint8_t wait_for_complete_max;
extern uint32_t nordic_repeat;
extern uint32_t wifi_repeat;

extern uint32_t communicationTestCounter_fromNordic[]; //CF: communication analysis
extern uint32_t communicationTestCounter_toClient[]; //CF: communication analysis
extern uint32_t communicationTestCounter_1[]; //CF: communication analysis
extern uint32_t communicationTestCounter_2[]; //CF: communication analysis

void atheros_loop(uint32_t dummy);
uint32_t init_wmiconfig(uint32_t);

uint32_t readSatIdList(int satn);
void onlyTest(bool isCommand);

#endif /* ATHEROS_MAIN_LOOP_H_ */
