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
/* WARNING: You must define CB_SIZE and CB_TYPE before using this library!!! */


/* Function cbuf_init: initialize cbuf
 *
 * WARNING: YOU MUST CALL THIS TO INITIALIZE!!
 *
 * Parameters:
 * 		cb			the circular buffer
 * 		keyval		the value we're looking for
 * 		index_found	the index, if found
 *		callback	callback function for comparison,
 *					if set to NULL, then memcmp for the whole struct will be used.
 * Return val:
 * 		E_OK		if keyval is found, then we return E_OK,
 * 				    and the index_found contains valid data
 *
 * 		E_NOT_FOUND	if returns is NOT E_OK, then don't check
 * 					index_found.
 */
dspi_status_t cbuf_init(cbuf_t* cb, cbuf_callback_t callback) {
	if (cb == NULL) {
		return E_NO_RESOURCE;
	}
	cb->full = 0;
	cb->head = 0;
	cb->tail = 0;
	cb->foundMatch = callback;

	return E_OK;
}

/* Function to cbuf_push a new value into the queue
 * Input val : element to be inserted into the queue
 * Output    : -1 if insertion fails, 0 for successful insertion
 */
dspi_status_t cbuf_push(cbuf_t *cb, CB_TYPE val)
{
	if (cb == NULL) {
		return E_NO_RESOURCE;
	}

 	if(cbuf_queueIsFull(cb)) /* check if queue is full */
 	{
 		printf("queue full no space");
        return E_NO_RESOURCE; /* return failure */
 	}

 	cb->A[cb->tail] = val;  /* Push the new value in the queue */

 	cb->tail = (cb->tail+1) % CB_SIZE; /* This will take care of wrapping the tail to 0 */

 	if(cb->tail == cb->head) /*check if queue is full after this insertion */
	{
        printf("queue full after insertion");
		cb->full = TRUE;
 	}

 	return E_OK; /* return success */
}


/* Function to check if the queue is full
 * Returns 1 if queue is full otherwise 0
 */
uint8_t cbuf_queueIsFull(cbuf_t *cb)
{
	if (cb == NULL) {
		return E_NO_RESOURCE;
	}
	return cb->full;
}

/* Function to check if queue is empty.
 * Returns 1 if empty, else 0
 */
uint8_t cbuf_queueIsEmpty(cbuf_t *cb)
{
	if (cb == NULL) {
		return E_NO_RESOURCE;
	}
	if((cb->head == cb->tail) && (cb->full != 1))
	{
		return TRUE; /* Queue is empty */
	}
	return FALSE;
}

/* Function to cbuf_pop an element from the queue.
 * Return value indicates success/failure of the operation, using
 * input parameter as reference to return the key value since using
 * -1 or any other integer value can be present as value in array.
 */
dspi_status_t cbuf_pop(cbuf_t *cb, CB_TYPE *pkeyval)
{
	if (cb == NULL) {
		return E_NO_RESOURCE;
	}
    if(queueIsEmpty(cb) == FALSE) /* make sure queue is NOT empty */
    {
		memcpy(pkeyval,&(cb->A[cb->head]),sizeof(CB_TYPE)); /* pop oldest element */
		cb->head = (cb->head+1) % CB_SIZE; /* adjust head index */
		cb->full = FALSE;	/* since the element is successfully removed, full has to be 0. */
		return E_OK;   		/* success */
  	}
	return E_NO_RESOURCE;
}

/* Function cbuf_findKey: given a key value, find the oldest matching valid entry.
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
dspi_status_t cbuf_findKey(cbuf_t *cb, CB_TYPE keyval, uint32_t *index_found)
{
	uint8_t	found = FALSE;

	if (cb == NULL) {
		return E_NO_RESOURCE;
	}
   /* Exit if queue is empty */
   if(cbuf_queueIsEmpty(cb) == TRUE)
   {
   	return E_NOT_FOUND;
   }

   /* traverse from head to tail to look for the value */
   uint32_t index = cb->head;
   do {
	 if(cb->foundMatch) {
		 if (cb->foundMatch(cb->A[index]), keyval) {
			 found = TRUE;
		 }
	 }
	 else {
		 if (!memcmp(&cb->A[index], &keyval, sizeof(CB_TYPE))) {
			 found = TRUE;
		 }
	 }

	 if (found) {
	   /* If found match, return the index */
	   *index_found = index;
   	   return E_OK;
	 }
   	 index = (index + 1) % CB_SIZE;  /* wrap the queue */
   } while(index != cb->tail);

   return E_NOT_FOUND;  /*Keyval is not present in the array */
}

