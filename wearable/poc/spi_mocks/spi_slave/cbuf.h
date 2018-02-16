/* Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * cbuf.h
 *
 *  Created on: Mar 7, 2015
 *      Author: cwati/cwajh (forked)
 */

#ifndef CBUF_H_
#define CBUF_H_

#include <stdint.h>
#include "common_types.h"

/* Must define the following
#define CB_SIZE <size in number>
#define CB_TYPE <data type, such as int>
*/

// TODO: factor these out.
#define DEBUG		1
#define CB_SIZE 	10		/* A number greater than 3 */
#define CB_TYPE 	uint32_t
typedef dspi_status_t err_t;
	
typedef struct {
CB_TYPE A[CB_SIZE];   /* Circular array of integers */
uint32_t head;  /* FIFO head position/index where entry is popped out */
uint32_t tail;  /* FIFO tail position/index pointing to entry to push in */
uint8_t full;  /* If 1 then FIFO is full (8 entries), else it's non-full (0-7 entries) */
} cbuf_t;

err_t push(cbuf_t *cb, CB_TYPE val);
uint8_t queueIsFull(cbuf_t *cb);
uint8_t queueIsEmpty(cbuf_t *cb);
err_t pop(cbuf_t *cb, CB_TYPE *pkeyval);
err_t findKey(cbuf_t *cb, CB_TYPE keyval, uint32_t *index_found);
err_t cbuf_init(cbuf_t *cb);

#endif /* CBUF_H_ */
