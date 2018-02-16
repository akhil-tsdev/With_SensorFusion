/* Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * cbuf.h
 *
 *  Created on: Mar 7, 2015
 *      Author: cwati
 */

#ifndef __RFCBUF_H_
#define __RFCBUF_H_

#include <stdint.h>
#include "common_err.h"
#include "common_types.h"
#include "net_common.h"

typedef union rf_rx_ {
	hts_packet_t hts;
	sth_packet_t sth;
} rf_rx_t;

typedef struct rx_packet_ {
	net_packet_type_t packet_type;
	rf_rx_t packet;
} rx_packet_t;

#define RFCB_TYPE 		sth_packet_t
#if HUB_NORDIC
#define RFCB_SIZE 		8	/* cwati todo ssc uses 8 sats */
#endif

typedef struct {
	RFCB_TYPE A[RFCB_SIZE+1];	// Circular array of integers */
	uint32_t head;		// FIFO head position/index where entry is popped out
	uint32_t tail;		// FIFO tail position/index pointing to entry to push in
	uint8_t  full;		// If 1 then FIFO is full, else it's non-full
	uint32_t num_pushed;		// Number of element in cbuf
	uint32_t num_popped;		// Number of element in cbuf
} rfcbuf_t;

err_t rfcbufPush(rfcbuf_t *cb, RFCB_TYPE val);
uint8_t rfcbufIsFull(rfcbuf_t *cb);
uint8_t rfcbufIsEmpty(rfcbuf_t *cb);
err_t rfcbufPop(rfcbuf_t *cb, RFCB_TYPE *pkeyval);
err_t rfcbufPopDiscard(rfcbuf_t *cb);
err_t rfcbufFindKey(rfcbuf_t *cb, RFCB_TYPE keyval, uint32_t *index_found);
err_t rfcbufInit(rfcbuf_t *cb);
uint32_t rfcbufNum(rfcbuf_t *cb);
err_t rfcbufPeek(rfcbuf_t *cb, RFCB_TYPE *pkeyval);

#endif /* __RFCBUF_H_ */
