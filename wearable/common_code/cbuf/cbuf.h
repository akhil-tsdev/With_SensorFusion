/* Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * cbuf.h
 *
 *  Created on: Mar 7, 2015
 *      Author: cwati
 */

#ifndef CBUF_H_
#define CBUF_H_

#include <stdint.h>
#include "hub_module.h"
#include "sat_module.h"

#if HUB_22F
/* Perfect size
 * For 10 sensors, no mag, cb_size 200,
 * 10 * 200 * 32 = 64,000 = 64 KB
 *
 * For 10 sensors, with mag
 * 64,000 / 10 = 6400 per sensor
 * 6400 / 38 = 168 (CB_SIZE)
 *
 * For 15 sensors, no mag
 * 64,000 / 15 = 4266 per sensor
 * 4266 / 32 = 133 (CB_SIZE)
 *
 * This size is sensitive.  Make it bigger and you're risking
 * the code running erratically.
 */
#define MAX_CBUF_SIZE	        6000

#define CB_TYPE 		cbuf_hub_t
#define CB_SIZE 		((MAX_CBUF_SIZE/MAX_SENSORS) / sizeof(cbuf_hub_t))

#elif HUB_NORDIC
#define CB_SIZE								80
#define CB_TYPE								sat_sensor_record_t 
#elif SAT_NORDIC
#include "net_common.h"
#if PIVOT_3_0
#define CB_SIZE								(SENSOR_RECORDS_PER_NET_PACKET*20)
#else
#define CB_SIZE								(SENSOR_RECORDS_PER_NET_PACKET*2)
#endif
#define CB_TYPE								sat_sensor_record_t 
#elif SAT_22F
#define CB_SIZE								(MAX_SENSORS_IN_CS * 5)
#define CB_TYPE								sat_sensor_record_t
#endif /* HUB_22F */

typedef struct {
	CB_TYPE A[CB_SIZE+1];	// Circular array of integers */
	uint32_t head;		// FIFO head position/index where entry is popped out
	uint32_t tail;		// FIFO tail position/index pointing to entry to push in
	uint8_t  full;		// If 1 then FIFO is full, else it's non-full
	uint32_t num_pushed;		// Number of element in cbuf
	uint32_t num_popped;		// Number of element in cbuf
} cbuf_t;

err_t cbufPush(cbuf_t *cb, CB_TYPE val);
uint8_t cbufIsFull(cbuf_t *cb);
uint8_t cbufIsEmpty(cbuf_t *cb);
err_t cbufPop(cbuf_t *cb, CB_TYPE *pkeyval);
err_t cbufPopDiscard(cbuf_t *cb);
err_t cbufFindKey(cbuf_t *cb, CB_TYPE keyval, uint32_t *index_found);
err_t cbufInit(cbuf_t *cb);
uint32_t cbufNum(cbuf_t *cb);
err_t cbufPeek(cbuf_t *cb, CB_TYPE *pkeyval);

#endif /* CBUF_H_ */
