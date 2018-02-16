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


#define ENABLE_WIFI		1
#define ENABLE_WIFI_TX		1

#define MAX_WIFI_CHANNEL        11

#define SAT_SENDING_FREQ	50

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
#define NORDIC_REPEAT_6         6
#define NORDIC_REPEAT_5         5
#define NORDIC_REPEAT_DEFAULT   NORDIC_REPEAT_5

#define WIFI_REPEAT_6           6
#define WIFI_REPEAT_5           5
#define WIFI_REPEAT_DEFAULT     WIFI_REPEAT_5

#define WAIT_FOR_COMPLETE_6     3
#define WAIT_FOR_COMPLETE_5     3
#define WAIT_FOR_COMPLETE_DEF   WAIT_FOR_COMPLETE_5

#endif /* SAT_SENDING_FREQ == 50 */

extern uint8_t hvac_init;
extern MUTEX_STRUCT cbuf_mutex;
extern MUTEX_STRUCT ack_to_client_mutex;
extern LWEVENT_STRUCT atheros_task_event;
extern uint32_t ack_to_client;

extern uint8_t wait_for_complete_max;
extern uint32_t nordic_repeat;
extern uint32_t wifi_repeat;

void atheros_loop(uint32_t dummy);
uint32_t init_wmiconfig(void);

#endif /* ATHEROS_MAIN_LOOP_H_ */
