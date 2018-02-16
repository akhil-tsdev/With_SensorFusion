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
#include <a_config.h>
#include <a_types.h>
#include <a_osapi.h>
#include <netbuf.h>
#include <cust_netbuf.h>
#include <htc_api.h>

void Driver_ReportReverseCredits(A_VOID *pReq);

#if defined(DRIVER_CONFIG_IMPLEMENT_RX_FREE_MULTIPLE_QUEUE)
A_UINT8 GET_QUEUE_INDEX(A_NETBUF_QUEUE_T *q)
{
   A_UINT8  i;

   for (i=0; i < 8; i++)
   {
       if (&GET_FREE_QUEUE(i) == q)
          break;
   }

   return i;
}
#endif

A_VOID a_netbuf_enqueue(A_NETBUF_QUEUE_T *q, A_VOID *pReq)
{
//    A_UINT8   i;

    _int_disable(); 
    if (q->head == NULL) {
        q->head = pReq;
    } else {
    	A_ASSIGN_QUEUE_LINK(q->tail, pReq);
        //((A_NETBUF*)q->tail)->queueLink = (A_NETBUF*)pReq;
    }

    q->tail = pReq;
    A_CLEAR_QUEUE_LINK(pReq);
    //((A_NETBUF*)pkt)->queueLink = NULL;
    q->count++;
    _int_enable();
}

A_VOID a_netbuf_prequeue(A_NETBUF_QUEUE_T *q, A_VOID *pReq)
{
    if(q->head == NULL){
        q->tail = pReq;
    }
    A_ASSIGN_QUEUE_LINK(pReq, q->head);
    //((A_NETBUF*)pkt)->queueLink = q->head;
    q->head = pReq;
    q->count++;
}

A_VOID *a_netbuf_dequeue(A_NETBUF_QUEUE_T *q)
{
    A_VOID* pReq;
    

    if(q->head == NULL) return (A_VOID*)NULL;
    _int_disable();
    pReq = q->head;

    if(q->tail == q->head){
        q->tail = q->head = NULL;
    }else{
    	q->head = A_GET_QUEUE_LINK(pReq);
        //q->head = (A_VOID*)(curr->queueLink);
    }

    q->count--;
    A_CLEAR_QUEUE_LINK(pReq);
    //curr->queueLink = NULL;
    _int_enable();
    return (A_VOID*)pReq;
}

#if defined(DRIVER_CONFIG_IMPLEMENT_RX_FREE_MULTIPLE_QUEUE)

extern A_UINT8  reverse_credits_init;
A_UINT8 credits_test = 0;

A_VOID a_rxbuf_enqueue(A_NETBUF_QUEUE_T *q, A_VOID *pReq)
{
    A_UINT8   epid;
    A_UINT32  bufCtrlNdx;

    a_netbuf_enqueue(q, pReq);

    if (reverse_credits_init == 0)
        return;

    epid = A_NETBUF_GET_ELEM(pReq, A_REQ_EPID);
    if (epid == ENDPOINT_0)
        return;

//    i = GET_QUEUE_INDEX(q);
    bufCtrlNdx = GetQueueCtrlIndexByEPID(epid);
    if ( bufCtrlNdx < 8 && bufCtrlNdx != 7)
    {
        CREDIT_INC(bufCtrlNdx);
//        Driver_ReportReverseCredits(pReq);
    }
    else
    {
//       printf("wrong\n");
    }
}

A_VOID *a_rxbuf_dequeue(A_NETBUF_QUEUE_T *q)
{
    A_VOID* pReq;
//    A_UINT8     i;

    pReq = a_netbuf_dequeue(q);
    if (pReq == NULL)
    {
        return  pReq;
    }

/*    i = GET_QUEUE_INDEX(q);

    if ( i < 8)
        CREDIT_DEC(i);
*/
    return  pReq;
}
#endif

#if 0
A_VOID *a_netbuf_peek_queue(A_NETBUF_QUEUE_T *q)
{
    return q->head;
}


A_INT32 a_netbuf_queue_size(A_NETBUF_QUEUE_T *q)
{
    return q->count;
}

A_INT32 a_netbuf_queue_empty(A_NETBUF_QUEUE_T *q)
{
    return((q->count == 0)? 1:0);
}
#endif

A_VOID a_netbuf_queue_init(A_NETBUF_QUEUE_T *q)
{
    q->head = q->tail = NULL;
    q->count = 0;
}

/* EOF */