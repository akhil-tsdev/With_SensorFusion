/*
 * Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * Circular Buffer. FIFO.
 *
 * cbuf.c
 *
 *  Created on: Mar 7, 2015
 *      Author: cwati
 */

#include <stdio.h>
#include <string.h>
#include "cbuf.h"
#include "common_err.h"
/* WARNING: You must define CB_SIZE and CB_TYPE before using this library!!!
 * And then you must call cbufInit to initialize. */

#ifdef PRINTF_UNSAFE
#define printf(...) do{}while(0)
#endif
	
static uint32_t next_after(uint32_t pos) {
	return (pos+1)%(CB_SIZE+1);
}
	
/* Function cbufInit: initialize cbuf
 *
 * WARNING: YOU MUST CALL THIS TO INITIALIZE!!
 *
 * Parameters:
 * 		cb			the circular buffer
 * 		keyval		the value we're looking for
 * 		index_found	the index, if found
 *
 * Return val:
 * 		E_OK		if keyval is found, then we return E_OK,
 * 				    and the index_found contains valid data
 *
 * 		E_NOT_FOUND	if returns is NOT E_OK, then don't check
 * 					index_found.
 */
err_t cbufInit(cbuf_t *cb) {
	if (cb == NULL) {
		return E_NO_RESOURCE;
	}
	cb->full = 0;
	cb->head = 0;
	cb->tail = 0;
	cb->num_pushed = 0;
	cb->num_popped = 0;

	return E_OK;
}

/* Calculate number of items cbuf currently has */
uint32_t cbufNum(cbuf_t *cb) {
	return cb->num_pushed - cb->num_popped;
}


/* Function to push a new value into the queue
 * Input val : element to be inserted into the queue
 * Output    : -1 if insertion fails, 0 for successful insertion
 */
err_t cbufPush(cbuf_t *cb, CB_TYPE val)
{
	if (cb == NULL) {
		return E_NO_RESOURCE;
	}
	
 	if(cbufIsFull(cb)) /* check if queue is full */
 	{
 		printf("queue full no space");
        return E_NO_RESOURCE; /* return failure */
 	}

 	//cb->A[cb->tail] = val;  /* Push the new value in the queue */
 	memcpy(&(cb->A[cb->tail]),&val,sizeof(CB_TYPE));  /* Push the new value in the queue */

 	cb->tail = next_after(cb->tail);
	//(cb->tail+1) % CB_SIZE; /* This will take care of wrapping the tail to 0 */

 	//if(cb->tail == cb->head) /*check if queue is full after this insertion */
	//{
  //      printf("queue full after insertion");
	//	cb->full = TRUE;
 	//}

 	cb->num_pushed++;
 	return E_OK; /* return success */
}


/* Function to check if the queue is full
 * Returns 1 if queue is full otherwise 0
 */
uint8_t cbufIsFull(cbuf_t *cb)
{
	if (cb == NULL) {
		return 0;
	}
	return (cbufNum(cb) == CB_SIZE);
	//return cb->full;
}

/* Function to check if queue is empty.
 * Returns 1 if empty, else 0
 */
uint8_t cbufIsEmpty(cbuf_t *cb)
{
	if (cb == NULL) {
		return 1;
	}
	return (cbufNum(cb) == 0);
	//if((cb->head == cb->tail) && (cb->full != 1))
	//{
	//	return TRUE; /* Queue is empty */
	//}
	//return FALSE;
}

/* Function to pop an element from the queue, and discard the value.
 */
err_t cbufPopDiscard(cbuf_t *cb)
{
	if (cb == NULL) {
		return E_NO_RESOURCE;
	}
    if(cbufIsEmpty(cb) == FALSE) /* make sure queue is NOT empty */
    {
		cb->head = next_after(cb->head);
	 	cb->num_popped++;
		return E_OK;   		/* success */
  	}
	return E_NO_RESOURCE;
}

/* Function to pop an element from the queue.
 * Return value indicates success/failure of the operation, using
 * input parameter as reference to return the key value since using
 * -1 or any other integer value can be present as value in array.
 */
err_t cbufPop(cbuf_t *cb, CB_TYPE *pkeyval)
{
	if (cb == NULL) {
		return E_NO_RESOURCE;
	}
    if(cbufIsEmpty(cb) == FALSE) /* make sure queue is NOT empty */
    {
		memcpy(pkeyval,&(cb->A[cb->head]),sizeof(CB_TYPE)); /* pop oldest element */
		cb->head = next_after(cb->head);
		//cb->head = (cb->head+1) % CB_SIZE; /* adjust head index */
		//cb->full = FALSE;	/* since the element is successfully removed, full has to be 0. */
	 	cb->num_popped++;
		return E_OK;   		/* success */
  	}
	return E_NO_RESOURCE;
}
/* Function to get the next element from the queue without popping.
 * Return value indicates success/failure of the operation, using
 * input parameter as reference to return the key value since using
 * -1 or any other integer value can be present as value in array.
 */
err_t cbufPeek(cbuf_t *cb, CB_TYPE *pkeyval)
{
	if (cb == NULL) {
		return E_NO_RESOURCE;
	}
    if(cbufIsEmpty(cb) == FALSE) /* make sure queue is NOT empty */
    {
		memcpy(pkeyval,&(cb->A[cb->head]),sizeof(CB_TYPE)); /* copy oldest element */
		return E_OK;   		/* success */
  	}
	return E_NO_RESOURCE;
}

/* Function cbufFindKey: given a key value, find the oldest matching valid entry.
 *
 * Parameters:
 * 		cb			the circular buffer
 * 		keyval		the value we're looking for
 * 		index_found	the index, if found
 *
 * Return val:
 * 		E_OK		if keyval is found, then we return E_OK,
 * 				    and the index_found contains valid data
 *
 * 		E_NOT_FOUND	if returns is NOT E_OK, then don't check
 * 					index_found.
 */
err_t cbufFindKey(cbuf_t *cb, CB_TYPE keyval, uint32_t *index_found)
{
	if (cb == NULL) {
		return E_NO_RESOURCE;
	}
   /* Exit if queue is empty */
   if(cbufIsEmpty(cb) == TRUE)
   {
   	return E_NOT_FOUND;
   }

   /* traverse from head to tail to look for the value */
   uint32_t index = cb->head;
   do {
	 /* cwati test this memcmp TODO, in case of struct */
	 if(!memcmp(&cb->A[index], &keyval, sizeof(CB_TYPE)))
   	 {
	   /* If found match, return the index */
	   *index_found = index;
   	   return E_OK;
   	 }
   	 index = next_after(index);
			// = (index + 1) % CB_SIZE;  /* wrap the queue */
   } while(index != cb->tail);

   return E_NOT_FOUND;  /*Keyval is not present in the array */
}

