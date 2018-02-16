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
#include "common_types.h"

/* Must define the following
#define CB_SIZE <size in number>
#define CB_TYPE <data type, such as int>
*/
#define CB_SIZE 	MAX_SENSORS
#define CB_TYPE 	data_to_wifi_t

typedef uint8_t (*cbuf_callback_t)(CB_TYPE cbuf_data, CB_TYPE user_data);

typedef struct {
CB_TYPE 	A[CB_SIZE]; /* Circular array of integers */
uint32_t 	head;  		/* FIFO head position/index where entry is popped out */
uint32_t 	tail;  		/* FIFO tail position/index pointing to entry to push in */
uint8_t  	full;   	/* If 1 then FIFO is full (8 entries), else it's non-full (0-7 entries) */
cbuf_callback_t	foundMatch;	/* callback function for comparison */
} cbuf_t;

dspi_status_t cbuf_push(cbuf_t *cb, CB_TYPE val);
uint8_t cbuf_queueIsFull(cbuf_t *cb);
uint8_t cbuf_queueIsEmpty(cbuf_t *cb);
dspi_status_t cbuf_pop(cbuf_t *cb, CB_TYPE *pkeyval);
dspi_status_t cbuf_findKey(cbuf_t *cb, CB_TYPE keyval, uint32_t *index_found);
dspi_status_t cbuf_init(cbuf_t *cb, cbuf_callback_t callback);

#endif /* CBUF_H_ */
