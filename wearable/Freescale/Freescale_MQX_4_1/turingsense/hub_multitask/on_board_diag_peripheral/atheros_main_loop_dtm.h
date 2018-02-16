/*
 * Copyright TuringSense, Inc © 2015-2015
 * atheros_main_loop_dtm.h
 *
 *  Created on: June 25, 2016
 *      Author: cwati
 *
 *
 */

#ifndef ATHEROS_MAIN_LOOP_DTM_H_
#define ATHEROS_MAIN_LOOP_DTM_H_
#include <lwevent.h>
#include <mutex.h>
#include <stdint.h>

//cwati tmp
#define ENABLE_WIFI			1
#define ENABLE_WIFI_TX		1

#define MAX_WIFI_CHANNEL                11

#define SAT_SENDING_FREQ	50

extern uint8_t hvac_init;
extern MUTEX_STRUCT cbuf_mutex;
extern LWEVENT_STRUCT atheros_task_event;
extern uint32_t ack_to_client;

void atheros_loop(uint32_t dummy);
void setWiFiParameters();

#endif /* ATHEROS_MAIN_LOOP_DTM_H_ */
