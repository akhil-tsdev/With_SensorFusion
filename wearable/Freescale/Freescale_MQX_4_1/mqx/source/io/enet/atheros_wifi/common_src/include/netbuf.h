//------------------------------------------------------------------------------
// Copyright (c) 2011 Qualcomm Atheros, Inc.
// All Rights Reserved.
// Qualcomm Atheros Confidential and Proprietary.
// Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is
// hereby granted, provided that the above copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE
// INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
// USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
//------------------------------------------------------------------------------
//==============================================================================
// Author(s): ="Atheros"
//==============================================================================
#ifndef _NETBUF_H_
#define _NETBUF_H_

#include <a_types.h>
#include <a_config.h>
#include <athdefs.h>

/*
 * Network buffer queue support
 */
typedef struct {
    pointer head;
    pointer tail;
    A_UINT16 count;
} A_NETBUF_QUEUE_T;


typedef A_VOID (*A_TRANS_CB)(A_VOID*, A_VOID *);

/* A_TRANSPORT_OBJ - the structure used to encapsulate any/all bus requests */
typedef struct _transport_obj {   
    A_UINT32 lookahead; /* expected lookahead value for sanity checking */
    A_UINT16 transferLength; /* total transfer length including any pad bytes */
    A_UINT16 address; /* register address from which the transfer will begin */    
    A_UINT16 cmd; /* bit field containing command values */
    A_UINT8 credits; /* num credits consumed by this request (TX only) */
    A_UINT8 epid; /* endpoint ID */   
    A_STATUS status; /* transport status */    
    A_TRANS_CB cb; /* callback to be used upon completion <cannot be NULL> */
}A_TRANSPORT_OBJ;


#define A_NETBUF_QUEUE_INIT(q)  \
    a_netbuf_queue_init(q)

#define A_NETBUF_ENQUEUE(q, pReq) \
    a_netbuf_enqueue((q), (pReq))
#define A_NETBUF_PREQUEUE(q, pReq) \
    a_netbuf_prequeue((q), (pReq))
#define A_NETBUF_DEQUEUE(q) \
    (a_netbuf_dequeue((q)))
#define A_NETBUF_PEEK_QUEUE(q) ((A_NETBUF_QUEUE_T*)(q))->head            
#define A_NETBUF_QUEUE_SIZE(q)  ((A_NETBUF_QUEUE_T*)(q))->count
#define A_NETBUF_QUEUE_EMPTY(q) (!((A_NETBUF_QUEUE_T*)(q))->count)
   
/*
 *   receiver buffer
 */

#define A_RXBUF_ENQUEUE(q, pReq) \
    a_rxbuf_enqueue((q), (pReq))
#define A_RXBUF_DEQUEUE(q) \
    (a_rxbuf_dequeue((q)))

A_VOID a_netbuf_enqueue(A_NETBUF_QUEUE_T *q, A_VOID *pReq);
A_VOID a_netbuf_prequeue(A_NETBUF_QUEUE_T *q, A_VOID *pReq);
A_VOID *a_netbuf_dequeue(A_NETBUF_QUEUE_T *q);

A_VOID a_rxbuf_enqueue(A_NETBUF_QUEUE_T *q, A_VOID *pReq);
A_VOID *a_rxbuf_dequeue(A_NETBUF_QUEUE_T *q);

//A_INT32 a_netbuf_queue_size(A_NETBUF_QUEUE_T *q);
//A_INT32 a_netbuf_queue_empty(A_NETBUF_QUEUE_T *q);
A_VOID a_netbuf_queue_init(A_NETBUF_QUEUE_T *q);
//A_VOID *a_netbuf_peek_queue(A_NETBUF_QUEUE_T *q);

#define     MAX_NUM_FREE_QUEUES     8
#define     AR6000_DATA_OFFSET      64

typedef struct mgmt_credits {
        A_UINT16    rxMaxBufferSizeInQueues;
        A_UINT8     rxMaxBuffersInQueues;
        A_UINT8     rxQueueMask;
        A_UINT8     credit_count;
        A_UINT8     init_credits;
} MGMT_CREDITS;

extern A_UINT8    send_reverse_credits_flag;
extern MGMT_CREDITS host_credits[MAX_NUM_FREE_QUEUES];

extern A_NETBUF_QUEUE_T    rxFreeQueues[MAX_NUM_FREE_QUEUES];

#if defined(DRIVER_CONFIG_IMPLEMENT_RX_FREE_MULTIPLE_QUEUE)
extern A_UINT8 credits_test;
#endif

//extern A_UINT16    rxMaxBuffersInQueues[MAX_NUM_FREE_QUEUES];
//extern A_UINT16    rxMaxBufferSizeInQueues[MAX_NUM_FREE_QUEUES];
//extern A_UINT8     rxQueueMask[MAX_NUM_FREE_QUEUES];

#define GET_NUM_CREDIT_QUEUES()  (MAX_NUM_FREE_QUEUES)

#define GET_MAX_BUFFERS_IN_QUEUE(i)  (MAX_BUFFERS_IN_QUEUE_##i)

#define GET_INIT_CREDITS_IN_QUEUES(a)  \
    a[0].init_credits = GET_MAX_BUFFERS_IN_QUEUE(0); \
    a[1].init_credits = GET_MAX_BUFFERS_IN_QUEUE(1);\
    a[2].init_credits = GET_MAX_BUFFERS_IN_QUEUE(2);\
    a[3].init_credits = GET_MAX_BUFFERS_IN_QUEUE(3);\
    a[4].init_credits = GET_MAX_BUFFERS_IN_QUEUE(4);\
    a[5].init_credits = GET_MAX_BUFFERS_IN_QUEUE(5);\
    a[6].init_credits = GET_MAX_BUFFERS_IN_QUEUE(6);\
    a[7].init_credits = GET_MAX_BUFFERS_IN_QUEUE(7);

#define GET_MAX_BUFFERS_IN_QUEUES(a)  \
    a[0].rxMaxBuffersInQueues = GET_MAX_BUFFERS_IN_QUEUE(0); \
    a[1].rxMaxBuffersInQueues = GET_MAX_BUFFERS_IN_QUEUE(1);\
    a[2].rxMaxBuffersInQueues = GET_MAX_BUFFERS_IN_QUEUE(2);\
    a[3].rxMaxBuffersInQueues = GET_MAX_BUFFERS_IN_QUEUE(3);\
    a[4].rxMaxBuffersInQueues = GET_MAX_BUFFERS_IN_QUEUE(4);\
    a[5].rxMaxBuffersInQueues = GET_MAX_BUFFERS_IN_QUEUE(5);\
    a[6].rxMaxBuffersInQueues = GET_MAX_BUFFERS_IN_QUEUE(6);\
    a[7].rxMaxBuffersInQueues = GET_MAX_BUFFERS_IN_QUEUE(7);

#define GET_BUFFER_SIZE_IN_QUEUE(i) (BUFFER_SIZE_IN_QUEUE_##i)

#define GET_BUFFER_SIZE_IN_QUEUES(a)  \
    a[0].rxMaxBufferSizeInQueues = GET_BUFFER_SIZE_IN_QUEUE(0);\
    a[1].rxMaxBufferSizeInQueues = GET_BUFFER_SIZE_IN_QUEUE(1);\
    a[2].rxMaxBufferSizeInQueues = GET_BUFFER_SIZE_IN_QUEUE(2);\
    a[3].rxMaxBufferSizeInQueues = GET_BUFFER_SIZE_IN_QUEUE(3);\
    a[4].rxMaxBufferSizeInQueues = GET_BUFFER_SIZE_IN_QUEUE(4);\
    a[5].rxMaxBufferSizeInQueues = GET_BUFFER_SIZE_IN_QUEUE(5);\
    a[6].rxMaxBufferSizeInQueues = GET_BUFFER_SIZE_IN_QUEUE(6);\
    a[7].rxMaxBufferSizeInQueues = GET_BUFFER_SIZE_IN_QUEUE(7);

#define QUEUE_MASK(i) (QUEUE_MASK_##i)

#define GET_MASK_ALL_QUEUES(a)  \
    a[0].rxQueueMask = QUEUE_MASK(0);\
    a[1].rxQueueMask = QUEUE_MASK(1);\
    a[2].rxQueueMask = QUEUE_MASK(2);\
    a[3].rxQueueMask = QUEUE_MASK(3);\
    a[4].rxQueueMask = QUEUE_MASK(4);\
    a[5].rxQueueMask = QUEUE_MASK(5);\
    a[6].rxQueueMask = QUEUE_MASK(6);\
    a[7].rxQueueMask = QUEUE_MASK(7);

#define GET_FREE_QUEUE(i) (rxFreeQueues[i])

#define GET_QUEUE_BUFFER_SIZE(i) (host_credits[i].rxMaxBufferSizeInQueues + AR6000_DATA_OFFSET)
#define GET_QUEUE_DATA_BUFFER_SIZE(i) (host_credits[i].rxMaxBufferSizeInQueues)
#define GET_MAX_NUM_BUFFERS(i)  (host_credits[i].rxMaxBuffersInQueues)
#define GET_QUEUE_MASK(i)  (host_credits[i].rxQueueMask)
#define CREDIT_INC(i)      {if (credits_test == 0) host_credits[i].credit_count++;}
#define CREDIT_ADD(i,c)    {host_credits[i].credit_count += c;}
#define CREDIT_DEC(i)      (host_credits[i].credit_count--)
#define GET_CREDIT(i)      (host_credits[i].credit_count)
#define GET_INIT_CREDITS(i)  (host_credits[i].init_credits)
#define CLEAR_CREDIT(i)    (host_credits[i].credit_count = 0)

#endif /* _NETBUF_H_ */