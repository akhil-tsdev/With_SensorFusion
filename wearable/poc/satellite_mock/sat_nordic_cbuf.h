#include "net_common.h"
#define CB_TYPE sat_sensor_record_t
#define CB_SIZE 50

#define PRINTF_UNSAFE

#ifndef CBUF_H_
#define CBUF_H_

#include <stdint.h>

/* Must define the following before calling cbuf.h
#define CB_SIZE <size in number>
#define CB_TYPE <data type, such as int>
*/

typedef struct {
	CB_TYPE A[CB_SIZE];	// Circular array of integers */
	uint32_t head;		// FIFO head position/index where entry is popped out
	uint32_t tail;		// FIFO tail position/index pointing to entry to push in
	uint8_t  full;		// If 1 then FIFO is full, else it's non-full
} cbuf_t;

err_t push(cbuf_t *cb, CB_TYPE val);
uint8_t queueIsFull(cbuf_t *cb);
uint8_t queueIsEmpty(cbuf_t *cb);
err_t pop(cbuf_t *cb, CB_TYPE *pkeyval);
err_t peek(cbuf_t *cb, CB_TYPE *pkeyval);
err_t findKey(cbuf_t *cb, CB_TYPE keyval, uint32_t *index_found);
err_t cbuf_init(cbuf_t *cb);

#endif /* CBUF_H_ */
