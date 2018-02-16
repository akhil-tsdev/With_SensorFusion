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
#include "a_config.h"
#include <a_types.h>
#include <a_osapi.h>
#include <driver_cxt.h>
#include <common_api.h>
#include <custom_api.h>
#include "wmi_api.h"
#include "a_drv_api.h"
#include "cust_netbuf.h"



#if ENABLE_STACK_OFFLOAD

#include "atheros_stack_offload.h"
#include "common_stack_offload.h"
#include "custom_stack_offload.h"

#define MAX_CERT_DATA_LENGTH 1400
#define SSL_MAX_CERT_NAME_LEN 31
extern A_CONST A_UINT8 max_performance_power_param;
ATH_SOCKET_CONTEXT* ath_sock_context[MAX_SOCKETS_SUPPORTED + 1];

#if ENABLE_SSL
A_VOID *sslhandle = NULL; // Driver context ptr needed by the SSL_xxx() API functions

A_INT32 find_socket_context_from_ssl(SSL *ssl)
{
    A_INT32 index = 0;

    for (index = 0; index < MAX_SOCKETS_SUPPORTED; index++)
    {
        if (ath_sock_context[index]->ssl == ssl)
            return index;
    }
    /*Index not found*/
    return SOCKET_NOT_FOUND;
}
#endif


/*****************************************************************************/
/*  find_socket_context - Finds socket context based on socket handle. "retrieve"
 *                flag is used to identify if a new context is requested or an
 *                exiting context is retrieved.
 *      A_UINT32 handle - socket handle.
 * 		A_UINT8 retrieve - see description
 * Returns- index in context array
 *****************************************************************************/
A_INT32 find_socket_context(A_UINT32 handle, A_UINT8 retrieve)
{
    A_INT32 index = 0;
    if(retrieve)
    {
        if(handle == 0)
        return SOCKET_NOT_FOUND;
    }

    /*If handle is 0, we want to find first empty context and return the index.
    If handle is non zero, return the index that matches */	
    for (index = 0; index < MAX_SOCKETS_SUPPORTED; index++)
    {
        if(ath_sock_context[index]->handle == handle)
        return index;
    }
    /*Index not found*/
    return SOCKET_NOT_FOUND;
}



/*****************************************************************************/
/*  socket_context_init - Initializes common socket context, called during 
 *                 driver init.
 *      
 * Returns- A_OK in case of successful init, A_ERROR otherwise
 *****************************************************************************/
A_STATUS socket_context_init()
{
	A_UINT32 index = 0;
	
	for (index = 0; index < MAX_SOCKETS_SUPPORTED + 1; index++)	
	{
		/*Allocate socket contexts*/
		if((ath_sock_context[index] = A_MALLOC(sizeof(ATH_SOCKET_CONTEXT),MALLOC_ID_CONTEXT)) == NULL)
			return A_NO_MEMORY;

        memset(ath_sock_context[index], 0, sizeof(ATH_SOCKET_CONTEXT));
	}
	
	/*initialize custom socket context*/
	if(CUSTOM_SOCKET_CONTEXT_INIT() != A_OK)
		return A_ERROR;

		
	return A_OK;
}




/*****************************************************************************/
/*  socket_context_deinit - De-initializes common socket context, called during 
 *                 driver deinit.
 *****************************************************************************/
A_VOID socket_context_deinit()
{
	A_UINT32 index = 0;
	for (index = 0; index < MAX_SOCKETS_SUPPORTED + 1; index++)	
	{
		A_FREE(ath_sock_context[index],MALLOC_ID_CONTEXT);
	}
        /*Free the custom context as well*/
        CUSTOM_SOCKET_CONTEXT_DEINIT();
}



/*****************************************************************************/
/*  Api_SockResponseEventRx - Handler for WMI socket receive events.
 *  A_VOID *pCxt  - driver context
 *  A_UINT8 *datap - pointer to incoming event data
 *  A_UINT32 len   - length of event data 
 *****************************************************************************/
A_STATUS Api_SockResponseEventRx(A_VOID *pCxt, A_UINT8 *datap, A_UINT32 len, A_VOID *pReq)
{
    A_INT32 index = 0;
    A_UINT8 unblock_flag=0;
    WMI_SOCK_RESPONSE_EVENT *response = (WMI_SOCK_RESPONSE_EVENT*)datap;
    A_UINT32 resp_type = A_CPU2LE32(response->resp_type);
    A_STATUS status = A_OK;
    A_UINT8 freeBuf = 1;
    
    switch(resp_type)
    {		
    case SOCK_OPEN:                
        index = find_socket_context(SOCKET_HANDLE_PLACEHOLDER, TRUE);

        if(index < 0 || index > MAX_SOCKETS_SUPPORTED)
        {
            last_driver_error = A_SOCKCXT_NOT_FOUND;
            status = A_ERROR;
            break;
        }   
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {
            /*Store the newly created socket handle*/
            ath_sock_context[index]->handle = A_CPU2LE32(response->sock_handle);        
            /* unlock the application thread*/
            unblock_flag = 1;                      
        }     
        break;            
        
    case SOCK_ACCEPT:
        index = find_socket_context(A_CPU2LE32(response->sock_handle), TRUE);
        
        if(index < 0 || index > MAX_SOCKETS_SUPPORTED)
        {
            last_driver_error = A_SOCKCXT_NOT_FOUND;
            status = A_ERROR;
            break;
        }	
        
        /*Check if a socket is waiting on the response*/
        //if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {
            /*Copy incoming socket related information*/
            ath_sock_context[index]->data = response->data; 
            ath_sock_context[index]->pReq = pReq;            
            
            /*Store the response in socket context result field*/
            ath_sock_context[index]->result = A_CPU2LE32(response->error);        
            /*unlock the thread*/
            if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
            {
                unblock_flag=1;
            }
            freeBuf = 0;
            CUSTOM_UNBLOCK_SELECT(pCxt);
        }
        break;		
    case SOCK_CLOSE:
        /* A socket close event may be received under two scenarios-
          1. In response to explicit socket close request from host.
          2. An unsolicited socket close to close a socket created earlier for
             an incoming TCP connection
          In the first scenario, the application thread is waiting for the response, 
          so unblock the thread. 
          In the second case, just cleanup the socket context*/
        
        index = find_socket_context(A_CPU2LE32(response->sock_handle), TRUE);
        
        if(index < 0 || index > MAX_SOCKETS_SUPPORTED)
        {
            last_driver_error = A_SOCKCXT_NOT_FOUND;         
            status = A_ERROR;
            break;
        }		
       
        
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type)){
            /* the blocked flag will clear the context. clearing 
             * the context here will erase the mask which would 
             * break up the task synchronization. */
            /*unlock the thread*/
            unblock_flag=1;     
            /*Store the response in socket context result field*/
            ath_sock_context[index]->result = A_CPU2LE32(response->error);
        }else{
            /* Case 2: clear the context here, no one is waiting */
            // clear_socket_context(index);
            ath_sock_context[index]->TCPCtrFlag = TCP_FIN;
#if T_SELECT_VER1
            //Asynchronous sock_close event. 
            //printf("Asynch SOCK Close for socket index %d\n", index);
            //SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_CLOSE);
            CUSTOM_UNBLOCK_SELECT(pCxt);
#endif //T_SELECT_VER1
        }   
        break;	
    case SOCK_CONNECT:
        index = find_socket_context(A_CPU2LE32(response->sock_handle), TRUE);
        
        if(index < 0 || index > MAX_SOCKETS_SUPPORTED)
        {
            last_driver_error = A_SOCKCXT_NOT_FOUND;
            status = A_ERROR;
            break;
        }	
        /*Store the response in socket context result field*/
        ath_sock_context[index]->result = A_CPU2LE32(response->error);        
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {	
            /*unlock the thread*/
            unblock_flag=1;    
        }else{
            if(A_CPU2LE32(response->error) == TCP_CONNECTION_AVAILABLE)
            {
                /*Special case: async listen event has been received indicating a new
                TCP connection is available. The User thread may be blocked with a select
                call, set the unblock_flag so the user thread can be unlocked*/
                unblock_flag=1;
            }  
        }
        CUSTOM_UNBLOCK_SELECT(pCxt);
        break;	        

    case SOCK_BIND:
    case SOCK_LISTEN:
      /*Listen Event may be received in two cases:
      Case 1: In response to a listen request from host, that puts TCP socket in
              Listen state.
      Case 2: An unsolicited listen with a result value TCP_CONNECTION_AVAILABLE,
              that informs an application thread that a new TCP connection is 
              available. This allows the application to call accept.*/
    case SOCK_ERRNO:
    case SOCK_SETSOCKOPT:
        index = find_socket_context(A_CPU2LE32(response->sock_handle), TRUE);
        
        if(index < 0 || index > MAX_SOCKETS_SUPPORTED)
        {
            last_driver_error = A_SOCKCXT_NOT_FOUND;
            status = A_ERROR;
            break;
        }	
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {	
            /*Store the response in socket context result field*/
            ath_sock_context[index]->result = A_CPU2LE32(response->error);        
            /*unlock the thread*/
            unblock_flag=1;    
        }else{
            if(A_CPU2LE32(response->error) == TCP_CONNECTION_AVAILABLE)
            {
                /*Special case: async listen event has been received indicating a new
                TCP connection is available. The User thread may be blocked with a select
                call, set the unblock_flag so the user thread can be unlocked*/
                ath_sock_context[index]->result = A_CPU2LE32(response->error);
                unblock_flag=1;
            }  
        }
        break;	        
    case SOCK_GETSOCKOPT:
        index = find_socket_context(A_CPU2LE32(response->sock_handle), TRUE);
        
        if(index < 0 || index > MAX_SOCKETS_SUPPORTED)
        {
            last_driver_error = A_SOCKCXT_NOT_FOUND;
            status = A_ERROR;
            break;
        }

        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {
            ath_sock_context[index]->result = A_CPU2LE32(response->error);        
            /*Copy select related information*/
            ath_sock_context[index]->data = response->data; 
            ath_sock_context[index]->pReq = pReq;            
            /* unlock the application thread*/
            unblock_flag=1;        
            freeBuf = 0;
        }
        break;       
    case SOCK_IPCONFIG:
        index = GLOBAL_SOCK_INDEX;
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {
            /*Copy ipconfig related information*/
            ath_sock_context[index]->data = response->data; 
            ath_sock_context[index]->pReq = pReq;                        
            ath_sock_context[index]->result = A_CPU2LE32(response->error);        
            /*unlock the thread*/
            unblock_flag=1;  
            freeBuf = 0;
        }
        break;        
    case SOCK_PING:
    case SOCK_PING6:
        index = GLOBAL_SOCK_INDEX;
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {
            /*Copy ipconfig related information*/		
            ath_sock_context[index]->result = A_CPU2LE32(response->error);		
            /*unlock the thread*/
            unblock_flag=1;		
        }
       
        break;
    case SOCK_IP6CONFIG:
        index = GLOBAL_SOCK_INDEX;
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {
            /*Copy ipconfig related information*/
            ath_sock_context[index]->data = response->data;             
            ath_sock_context[index]->result = A_CPU2LE32(response->error);	
            ath_sock_context[index]->pReq = pReq;
            /*unlock the thread*/
            unblock_flag=1;
            freeBuf = 0;
        }
        break;	
#if ENABLE_HTTP_SERVER
    case SOCK_HTTP_SERVER:
        index = GLOBAL_SOCK_INDEX;
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {
            ath_sock_context[index]->result = A_CPU2LE32(response->error);
            /*unlock the thread*/
            unblock_flag=1;
        }
        break;
    case SOCK_HTTP_SERVER_CMD:
        index = GLOBAL_SOCK_INDEX;
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {
            ath_sock_context[index]->pReq = pReq;
            ath_sock_context[index]->result = A_CPU2LE32(response->error);
            /*Copy select related information*/
            ath_sock_context[index]->data = response->data; 
            ath_sock_context[index]->pReq = pReq;				
            /*unlock the thread*/
            unblock_flag=1;
            freeBuf = 0;
        }
        break;
#endif
#if ENABLE_HTTP_CLIENT
    case SOCK_HTTPC:
        index = GLOBAL_SOCK_INDEX;
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {
            ath_sock_context[index]->pReq = pReq;
            ath_sock_context[index]->result = A_CPU2LE32(response->error);
            /*Copy select related information*/
            ath_sock_context[index]->data = response->data; 
            ath_sock_context[index]->pReq = pReq;			
            /*unlock the thread*/
            unblock_flag=1;
            freeBuf = 0;
        }
        //printf("Received http client resp :%d req %x\n", ath_sock_context[index]->result, ath_sock_context[index]->pReq);
        break;
#endif /* ENABLE_HTTP_CLIENT */
    case SOCK_DNC_CMD:
        index = GLOBAL_SOCK_INDEX;
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {
            /*Copy ipconfig related information*/
            ath_sock_context[index]->data = response->data;             
            ath_sock_context[index]->result = A_CPU2LE32(response->error);	
            ath_sock_context[index]->pReq = pReq;
            /*unlock the thread*/
            unblock_flag=1;
            freeBuf = 0;
        }
        break;
    case SOCK_DNC_ENABLE:
        index = GLOBAL_SOCK_INDEX;
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {
            ath_sock_context[index]->result = A_CPU2LE32(response->error);
            /*unlock the thread*/
            unblock_flag=1;
        }
#if TURINGSENSE
        break;
#endif
     case SOCK_DNS_SRVR_CFG_ADDR:
          index =  GLOBAL_SOCK_INDEX; 	
          /*Check if a socket is waiting on the response*/
          if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
          {
             ath_sock_context[index]->result = A_CPU2LE32(response->error);
             /*unlock the thread*/
             unblock_flag=1;
          }
          break;
    case SOCK_IP_HOST_NAME:
        index = GLOBAL_SOCK_INDEX;
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {
            ath_sock_context[index]->result = A_CPU2LE32(response->error);
            /*unlock the thread*/
            unblock_flag=1;
        }
        break;
    case SOCK_IP_DNS:
        index = GLOBAL_SOCK_INDEX;
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type))
        {
            ath_sock_context[index]->result = A_CPU2LE32(response->error);
            /*unlock the thread*/
            unblock_flag=1;
        }
#if TURINGSENSE
        break;
#endif
    case SOCK_IP_SNTP_GET_TIME:
        index = GLOBAL_SOCK_INDEX;
        //printf("resp from sntp \r\n");
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK2_TEST(ath_sock_context[index], resp_type))
        {
            /*Copy ipconfig related information*/
            ath_sock_context[index]->data = response->data;             
            ath_sock_context[index]->result = A_CPU2LE32(response->error);	
            ath_sock_context[index]->pReq = pReq;
            /*unlock the thread*/
            //("unblock\r\n");
            unblock_flag=1;
            freeBuf = 0;
        }
	break;
    case SOCK_IP_SNTP_GET_TIME_OF_DAY:
        index = GLOBAL_SOCK_INDEX;
                /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK2_TEST(ath_sock_context[index], resp_type))
        {
            /*Copy ipconfig related information*/
            ath_sock_context[index]->data = response->data;             
            ath_sock_context[index]->result = A_CPU2LE32(response->error);	
            ath_sock_context[index]->pReq = pReq;
            /*unlock the thread*/
            //("unblock\r\n");
            unblock_flag=1;
            freeBuf = 0;
        }
        break;
    case SOCK_IP_SNTP_QUERY_SNTP_ADDRESS:
        index = GLOBAL_SOCK_INDEX;
        /*Check if a socket is waiting on the response*/
        if(SOCK_EV_MASK2_TEST(ath_sock_context[index], resp_type))
        {
            /*Copy ipconfig related information*/
            ath_sock_context[index]->data = response->data;             
            ath_sock_context[index]->result = A_CPU2LE32(response->error);	
            ath_sock_context[index]->pReq = pReq;
            /*unlock the thread*/
            //("unblock\r\n");
            unblock_flag=1;
            freeBuf = 0;
        }
        break;
#if ENABLE_SSL
    case SOCK_SSL_SET_FD:
    case SOCK_SSL_ACCEPT:
    case SOCK_SSL_CONNECT:
        index = find_socket_context_from_ssl((SSL*) response->sock_handle); // ssl context
        if(SOCK_EV_MASK2_TEST(ath_sock_context[index], resp_type))
        {
            ath_sock_context[index]->result = A_CPU2LE32(response->error);
            /*unlock the thread*/
            unblock_flag=1;
        }
        break;

    case SOCK_SSL_CONFIGURE:
    case SOCK_SSL_SHUTDOWN:
        index = find_socket_context_from_ssl((SSL*) response->sock_handle); // ssl context
        if (index == SOCKET_NOT_FOUND)
        {
            index = GLOBAL_SOCK_INDEX;
        }
        if(SOCK_EV_MASK2_TEST(ath_sock_context[index], resp_type))
        {
            ath_sock_context[index]->result = A_CPU2LE32(response->error);
            /*unlock the thread*/
            unblock_flag=1;
        }
        break;

    case SOCK_SSL_CTX_NEW:
    case SOCK_SSL_CTX_FREE:
    case SOCK_SSL_NEW:
    case SOCK_SSL_ADD_CERT:
    case SOCK_SSL_STORE_CERT:
    case SOCK_SSL_LOAD_CERT:
        index = GLOBAL_SOCK_INDEX;
        if(SOCK_EV_MASK2_TEST(ath_sock_context[index], resp_type))
        {
            ath_sock_context[index]->result = A_CPU2LE32(response->error);
            /*unlock the thread*/
            unblock_flag=1;
        }
        break;

    case SOCK_SSL_LIST_CERT:
        index = GLOBAL_SOCK_INDEX;
        if(SOCK_EV_MASK2_TEST(ath_sock_context[index], resp_type))
        {
            ath_sock_context[index]->data = response->data;
            ath_sock_context[index]->result = A_CPU2LE32(response->error);
            /*unlock the thread*/
            unblock_flag=1;
        }
        break;
#endif

    default:
        last_driver_error = A_UNKNOWN_CMD;
        status = A_ERROR;
        break;	
    }

    /* if a user task is blocked on this event then unblock it. */
    if(unblock_flag /* && SOCK_EV_MASK_TEST(ath_sock_context[index], resp_type)*/){
        if (resp_type < 32) 
        {
            SOCK_EV_MASK_CLEAR(ath_sock_context[index], resp_type);
        }
        else 
        {
            SOCK_EV_MASK2_CLEAR(ath_sock_context[index], resp_type); 
        }
        CUSTOM_UNBLOCK(ath_sock_context[index], RX_DIRECTION);    
    }
    
    if(freeBuf){
        /*Free the netbuf*/
        A_NETBUF_FREE(pReq);   
    }    
    return status;
}



/*****************************************************************************/
/*  getIPLength - Calculates total IP header length.
 *  A_UINT8 version - IP version    
 * Returns- IP header length
 *****************************************************************************/
A_UINT32 getIPLength(A_UINT8 version)
{
	A_UINT32 length = 0;
	switch(version)
	{
		case ATH_AF_INET:
			length = IPV4_HEADER_LENGTH;
		break;
		
		case ATH_AF_INET6:
			length = IPV6_HEADER_LENGTH;
		break;
		default:
		break;		
	}
	
	return length;
	
}


/*****************************************************************************/
/*  getTransportLength - Calculates Transport layer header length
 *  A_UINT8 proto - UDP or TCP    
 * Returns- Transport layer header length
 *****************************************************************************/
A_UINT32 getTransportLength(A_UINT8 proto)
{
	A_UINT32 length = 0;
	switch(proto)
	{
		case SOCK_STREAM_TYPE:
			length = TCP_HEADER_LENGTH;
			break;
		
		case SOCK_DGRAM_TYPE:
			length = UDP_HEADER_LENGTH;
			break;
		default:
			break;		
	}
	
	return length;
	
}



/*****************************************************************************/
/*  Api_socket - API to create new socket
 *  A_VOID *pCxt- driver context
 *  A_UINT32 domain- IPv4, IPv6 
 *  A_UINT32 type- TCP, UDP
 *  A_UINT32 protocol-    
 * Returns- socket handle in case of success, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_socket(A_VOID *pCxt, A_UINT32 domain, A_UINT32 type, A_UINT32 protocol)
{
    A_DRIVER_CONTEXT *pDCxt;
    SOCK_OPEN_T sock_open;
    A_INT32 index = 0;
    A_INT32 result = A_OK;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);
    
    do{
        /*Create new context*/
        if((index = find_socket_context(EMPTY_SOCKET_CONTEXT,FALSE)) != SOCKET_NOT_FOUND)
        {
            ath_sock_context[index]->handle = SOCKET_HANDLE_PLACEHOLDER;
            ath_sock_context[index]->domain = domain;
            ath_sock_context[index]->type = type;
        }
        else
        {
            last_driver_error = A_SOCK_UNAVAILABLE;
            result = A_ERROR;
            break;
        }
    
        /*Create socket open wmi message*/
        sock_open.domain = A_CPU2LE32(domain);
        sock_open.type = A_CPU2LE32(type);
        sock_open.protocol = A_CPU2LE32(protocol);

        SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_OPEN);    

        if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_OPEN, (void *)(&sock_open), sizeof(SOCK_OPEN_T)) != A_OK){
            SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_OPEN);
            /*Command failed, clear socket context and return error*/
            clear_socket_context(index);
            result = A_ERROR;
            break;
        }
    
        /*Wait for response from target*/
        do{
            if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
                A_ASSERT(0);
            }       
        }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_OPEN));    
    
        if(ath_sock_context[index]->handle == A_ERROR)
        {
            /*Socket not created, return error*/
            clear_socket_context(index);            
            result = A_ERROR;
            break;
        }
        else{
            result = ath_sock_context[index]->handle;
        }
    }while(0);  
    
    return result;
}


/*****************************************************************************/
/*  Api_shutdown - Close a previously opened socket
 *  A_VOID *pCxt- driver context 
 *  A_UINT32 handle- socket handle    
 * Returns- 0 in case of successful shutdown, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_shutdown(A_VOID *pCxt, A_UINT32 handle)
{
    SOCK_CLOSE_T sock_close;
    A_INT32 index = 0;
    A_DRIVER_CONTEXT *pDCxt;
    A_INT32 result = A_OK;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);  
    
    do{    
        /*Find context*/
        if((index = find_socket_context(handle,TRUE)) == SOCKET_NOT_FOUND){
            result = A_ERROR;
            break;
        }
    
        /*Check if some other thread is wating on this socket*/
        if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
            result = A_ERROR;
            break;
        }
    
        /*Delete all pending packets in the receive queue*/
        CUSTOM_PURGE_QUEUE(index);
    
        /*Create a socket close wmi message*/
        sock_close.handle = A_CPU2LE32(handle);
        /* set the sock_st_flags before calling wmi_ to avoid possible race conditions */
        //SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_CLOSE);
        
        if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_CLOSE, (void *)(&sock_close), sizeof(SOCK_CLOSE_T)) != A_OK){
            /* clear the flag that would have been cleared by receiving the event */
            SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_CLOSE);
            result = A_ERROR;
            break;	
        }

#if 0 // For shutdown, we are not gonna wait for response. Fire and forgot 
        /*Wait for response from target*/		
        do{
            if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
                A_ASSERT(0);
            }  
        }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_CLOSE));
#endif
        
        result = ath_sock_context[index]->result;
        /* Delete all pending packets */
        CUSTOM_PURGE_QUEUE(index);

        /* clear the socket to make it available for re-use */
        clear_socket_context(index);
        
        restore_power_state(pDCxt, 3); /* POWER_STATE_MOVED_FOR_TCPRX */
        
    }while(0);
    
    return result;	
}



/*****************************************************************************/
/*  Api_connect - API to connect to a peer
 *  A_VOID *pCxt- driver context 
 *  A_UINT32 handle- socket handle    
 *  A_VOID* name- sock addr structure 
 *  A_UINT16 length- sock add length
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_connect(A_VOID *pCxt, A_UINT32 handle, A_VOID* name, A_UINT16 length)
{
    A_DRIVER_CONTEXT *pDCxt;
    SOCK_CONNECT_CMD_T sock_connect;
    A_INT32 index = 0;
    A_INT32 result = A_OK;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);  
    
    do{
        /*Retrieve context*/
        if((index = find_socket_context(handle,TRUE)) == SOCKET_NOT_FOUND)
        {
            result = A_ERROR;
            break;
        }
    
        if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
            result = A_ERROR;
            break;
        }
    
        sock_connect.handle = A_CPU2LE32(handle);
        sock_connect.length = A_CPU2LE16(length);
    
        if(ath_sock_context[index]->domain == ATH_AF_INET)
        {		
            sock_connect.addr.name.sin_port = A_CPU2LE16(((SOCKADDR_T*)name)->sin_port);
            sock_connect.addr.name.sin_family = A_CPU2LE16(((SOCKADDR_T*)name)->sin_family);
            sock_connect.addr.name.sin_addr = A_CPU2LE32(((SOCKADDR_T*)name)->sin_addr);			
        }
        else
        {
            sock_connect.addr.name6.sin6_port = A_CPU2LE16(((SOCKADDR_6_T*)name)->sin6_port);
            sock_connect.addr.name6.sin6_family = A_CPU2LE16(((SOCKADDR_6_T*)name)->sin6_family);
            sock_connect.addr.name6.sin6_flowinfo = A_CPU2LE32(((SOCKADDR_6_T*)name)->sin6_flowinfo);	
            A_MEMCPY((A_UINT8*)&(sock_connect.addr.name6.sin6_addr),(A_UINT8*)&((SOCKADDR_6_T*)name)->sin6_addr,sizeof(IP6_ADDR_T));
        }

        SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_CONNECT);

        if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_CONNECT, (void *)(&sock_connect), sizeof(SOCK_CONNECT_CMD_T)) != A_OK){
            SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_CONNECT);
            result = A_ERROR;
            break;
        }
    
        do{
            if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
                A_ASSERT(0);
            }      
        }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_CONNECT));
        result = ath_sock_context[index]->result;
    }while(0);
    
    return result;
	
}


//Non blocking version of connect
A_INT32 Api_connect_nb(A_VOID *pCxt, A_UINT32 handle, A_VOID* name, A_UINT16 length)
{
    A_DRIVER_CONTEXT *pDCxt;
    SOCK_CONNECT_CMD_T sock_connect;
    A_INT32 index = 0;
    A_INT32 result = A_OK;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);  
    
    do{
        /*Retrieve context*/
        if((index = find_socket_context(handle,TRUE)) == SOCKET_NOT_FOUND)
        {
            result = A_ERROR;
            break;
        }
    
        if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
            result = A_ERROR;
            break;
        }
    
        sock_connect.handle = A_CPU2LE32(handle);
        sock_connect.length = A_CPU2LE16(length);
    
        if(ath_sock_context[index]->domain == ATH_AF_INET)
        {
            sock_connect.addr.name.sin_port = A_CPU2LE16(((SOCKADDR_T*)name)->sin_port);
            sock_connect.addr.name.sin_family = A_CPU2LE16(((SOCKADDR_T*)name)->sin_family);
            sock_connect.addr.name.sin_addr = A_CPU2LE32(((SOCKADDR_T*)name)->sin_addr);
        }
        else
        {
            sock_connect.addr.name6.sin6_port = A_CPU2LE16(((SOCKADDR_6_T*)name)->sin6_port);
            sock_connect.addr.name6.sin6_family = A_CPU2LE16(((SOCKADDR_6_T*)name)->sin6_family);
            sock_connect.addr.name6.sin6_flowinfo = A_CPU2LE32(((SOCKADDR_6_T*)name)->sin6_flowinfo);
            A_MEMCPY((A_UINT8*)&(sock_connect.addr.name6.sin6_addr),(A_UINT8*)&((SOCKADDR_6_T*)name)->sin6_addr,sizeof(IP6_ADDR_T));
        }


        if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_CONNECT, (void *)(&sock_connect), sizeof(SOCK_CONNECT_CMD_T)) != A_OK){
            SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_CONNECT);
            result = A_ERROR;
            break;
        }
    
    }while(0);
    
    return result;
}



/*****************************************************************************/
/*  Api_bind - API to bind to an interface. Works for both IPv4 and v6, name 
 *            field must be populated accordingly. 
 *  A_VOID *pCxt- driver context 
 *  A_UINT32 handle- socket handle    
 *  A_VOID* name- sock addr structure 
 *  A_UINT16 length- sock add length
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_bind(A_VOID *pCxt, A_UINT32 handle, A_VOID* name, A_UINT16 length)
{
    A_DRIVER_CONTEXT *pDCxt;
    SOCK_BIND_CMD_T sock_bind;
    A_INT32 index = 0;
    A_INT32 result = A_OK;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    
    do{
        /*Find context*/
        if((index = find_socket_context(handle,TRUE)) == SOCKET_NOT_FOUND)
        {
            result = A_ERROR;
            break;
        }
    
        if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
            result = A_ERROR;
            break;
        }
        sock_bind.handle = A_CPU2LE32(handle);
        sock_bind.length = A_CPU2LE16(length);
    
        if(ath_sock_context[index]->domain == ATH_AF_INET)
        {		
            sock_bind.addr.name.sin_port = A_CPU2LE16(((SOCKADDR_T*)name)->sin_port);
            sock_bind.addr.name.sin_family = A_CPU2LE16(((SOCKADDR_T*)name)->sin_family);
            sock_bind.addr.name.sin_addr = A_CPU2LE32(((SOCKADDR_T*)name)->sin_addr);			
        }
        else
        {
            sock_bind.addr.name6.sin6_port = A_CPU2LE16(((SOCKADDR_6_T*)name)->sin6_port);
            sock_bind.addr.name6.sin6_family = A_CPU2LE16(((SOCKADDR_6_T*)name)->sin6_family);
            sock_bind.addr.name6.sin6_flowinfo = A_CPU2LE32(((SOCKADDR_6_T*)name)->sin6_flowinfo);	
            A_MEMCPY((A_UINT8*)&(sock_bind.addr.name6.sin6_addr),(A_UINT8*)&((SOCKADDR_6_T*)name)->sin6_addr,sizeof(IP6_ADDR_T));
        }
    
        SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_BIND);

        if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_BIND, (void *)(&sock_bind), sizeof(SOCK_BIND_CMD_T)) != A_OK){
            SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_BIND);
            result = A_ERROR;
            break;
        }
    
        do{
            if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
                A_ASSERT(0);
            }   
        }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_BIND));
        result = ath_sock_context[index]->result;
    }while(0);
    
    return result;	
}





/*****************************************************************************/
/*  Api_listen - API to Listen for incoming connections. Only used on stream sockets
 *               Works for both IPv4 and v6.
 *  A_VOID *pCxt- driver context 
 *  A_UINT32 handle- socket handle    
 *  A_UINT32 backlog- backlog of pending connections. The backlog parameter defines 
 *                    the maximum length for the queue of pending connections
 * Returns- 0 in case of success, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_listen(A_VOID *pCxt, A_UINT32 handle, A_UINT32 backlog)
{
    A_DRIVER_CONTEXT *pDCxt;
    SOCK_LISTEN_T sock_listen;
    A_INT32 index = 0;
    A_INT32 result = A_OK;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    
    do{
        /*Find context*/
        if((index = find_socket_context(handle,TRUE)) == SOCKET_NOT_FOUND)
        {
            result = A_ERROR;
            break;
        }
    
        if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
            result = A_ERROR;
            break;
        }
    
        if (A_OK != move_power_state_to_maxperf(pDCxt, 3)) /* POWER_STATE_MOVED_FOR_TCPRX */
        {
            result = A_ERROR;
            break;
        }
        
        sock_listen.handle = A_CPU2LE32(handle);
        sock_listen.backlog = A_CPU2LE16(backlog);
    
       // SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_LISTEN);    

        if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_LISTEN, (void *)(&sock_listen), sizeof(SOCK_LISTEN_T)) != A_OK){
         //   SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_LISTEN);
            result = A_ERROR;
            break;
        }
    
        /*Block until stack provides a response*/
#if 0
        do{
            if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
                A_ASSERT(0);
            }
        }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_LISTEN));
#endif
        
       // result = ath_sock_context[index]->result;
    }while(0);
    ath_sock_context[index]->TCPCtrFlag = TCP_LISTEN; 
    return result;	
}




/*****************************************************************************/
/*  Api_accept - API to accept incoming inconnections. Works for both IPv4 and v6,  
 *            name field must be populated accordingly. Must call select to wait 
 *            for incoming connection before calling this API.
 *  A_VOID *pCxt- driver context 
 *  A_UINT32 handle- socket handle    
 *  A_VOID* name- sock addr structure 
 *  A_UINT16 length- sock add length
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_accept(A_VOID *pCxt, A_UINT32 handle, A_VOID* name, A_UINT16 length)
{
    A_DRIVER_CONTEXT *pDCxt;
    SOCK_ACCEPT_CMD_T sock_accept;
    A_INT32 index = 0;	
    A_INT32 accept_index = 0;
    A_INT32 result = A_OK;
    A_UINT8 free_buf = 1;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    
    do{
        /*Find context*/
        if((index = find_socket_context(handle,TRUE)) == SOCKET_NOT_FOUND)
        {
            result = A_ERROR;
            free_buf = 0;
            break;
        }
        if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
            result = A_ERROR;
            free_buf = 0;
            break;
        }
        /* prepare wmi accept structure*/
        sock_accept.handle = A_CPU2LE32(handle);
        sock_accept.length = A_CPU2LE16(length);		
    
        SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_ACCEPT);

        if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_ACCEPT, (void *)(&sock_accept), sizeof(SOCK_ACCEPT_CMD_T))!= A_OK){
            SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_ACCEPT);
            result = A_ERROR;
            free_buf = 0;
            break;
        }
    
        /*Block until stack provides a response*/
        do{
            if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
                A_ASSERT(0);
            }
        }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_ACCEPT));
    
        if(ath_sock_context[index]->result == TCP_CONNECTION_AVAILABLE)
        {
            /*Special case: Received TCP_CONN_AVAILABLE from target in response to accept, call
            Accept again to create new connection, free the previos netbuf*/
            if(free_buf){   
                /*Free the netbuf*/
                A_NETBUF_FREE(ath_sock_context[index]->pReq); 
            }
            /*Target stack indicates that a new TCP connection is available, accept it*/      
            SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_ACCEPT);

            if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_ACCEPT, (void *)(&sock_accept), sizeof(SOCK_ACCEPT_CMD_T))!=A_OK){
                SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_ACCEPT);
                result = A_ERROR;
                free_buf = 0;
                break;
            }
        
            /*Block until stack provides a response*/
            do{
                if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
                    A_ASSERT(0);
                }
            }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_ACCEPT));
        }

        if(ath_sock_context[index]->result != A_ERROR)
        {
            /*Create new context*/
            if((accept_index = find_socket_context(EMPTY_SOCKET_CONTEXT,FALSE)) != SOCKET_NOT_FOUND)
            {
                ath_sock_context[accept_index]->handle = ath_sock_context[index]->result;
                ath_sock_context[accept_index]->domain = ath_sock_context[index]->domain;
                ath_sock_context[accept_index]->type = ath_sock_context[index]->type;
            }
            else
            {
                last_driver_error = A_SOCK_UNAVAILABLE;                
                result = A_ERROR;
                break;
            }
        
            if(ath_sock_context[index]->data != NULL)
            {
                /*Based on IPv4 vs IPv6, fill in name fields*/
                if(ath_sock_context[index]->domain == ATH_AF_INET)
                {		
                        A_MEMCPY(name, &((SOCK_ACCEPT_RECV_T*)(ath_sock_context[index]->data))->addr.name, sizeof(SOCKADDR_T));
                        ((SOCKADDR_T*)name)->sin_port = A_CPU2LE16(((SOCKADDR_T*)name)->sin_port);
                        ((SOCKADDR_T*)name)->sin_family = A_CPU2LE16(((SOCKADDR_T*)name)->sin_family);
                        ((SOCKADDR_T*)name)->sin_addr = A_CPU2LE32(((SOCKADDR_T*)name)->sin_addr);			
                }
                else
                {
                        A_MEMCPY(name, &((SOCK_ACCEPT_RECV_T*)(ath_sock_context[index]->data))->addr.name6, sizeof(SOCKADDR_6_T));
                        ((SOCKADDR_6_T*)name)->sin6_port = A_CPU2LE16(((SOCKADDR_6_T*)name)->sin6_port);
                        ((SOCKADDR_6_T*)name)->sin6_family = A_CPU2LE16(((SOCKADDR_6_T*)name)->sin6_family);
                }			
                //A_FREE(ath_sock_context[index]->data, MALLOC_ID_CONTEXT);
               
                ath_sock_context[index]->data = NULL;
            }	       	
        }	
        result = ath_sock_context[index]->result;
    }while(0);
    
    if(free_buf){
        /*Free the netbuf*/
        A_NETBUF_FREE(ath_sock_context[index]->pReq);
    }
    return result;
}



#if T_SELECT_VER1

//Non blocking accept
A_INT32 Api_accept_ver1(A_VOID *pCxt, A_UINT32 handle, A_VOID* name, A_UINT16 length)
{
    A_DRIVER_CONTEXT *pDCxt;
    SOCK_ACCEPT_CMD_T sock_accept;
    A_INT32 index = 0;
    A_INT32 accept_index = 0;
    A_INT32 result = A_OK;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    
    do{
        /*Find context*/
        if((index = find_socket_context(handle,TRUE)) == SOCKET_NOT_FOUND)
        {
            result = A_ERROR;
            break;
        }
        if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
            result = A_ERROR;
            break;
        }


        //The accept was issued earlier and the result of accept is avaible here
        if((ath_sock_context[index]->result != TCP_CONNECTION_AVAILABLE) && (ath_sock_context[index]->data != NULL))
        {
            //printf("Got the result of accept command\n");
            /*Create new context*/
            if((accept_index = find_socket_context(EMPTY_SOCKET_CONTEXT,FALSE)) != SOCKET_NOT_FOUND)
            {
                ath_sock_context[accept_index]->handle = ath_sock_context[index]->result;
                ath_sock_context[accept_index]->domain = ath_sock_context[index]->domain;
                ath_sock_context[accept_index]->type = ath_sock_context[index]->type;
            }
            else
            {
                last_driver_error = A_SOCK_UNAVAILABLE;                
                result = A_ERROR;
                break;
            }
        
            if(ath_sock_context[index]->data != NULL)
            {
                /*Based on IPv4 vs IPv6, fill in name fields*/
                if(ath_sock_context[index]->domain == ATH_AF_INET)
                {
                        A_MEMCPY(name, &((SOCK_ACCEPT_RECV_T*)(ath_sock_context[index]->data))->addr.name, sizeof(SOCKADDR_T));
                        ((SOCKADDR_T*)name)->sin_port = A_CPU2LE16(((SOCKADDR_T*)name)->sin_port);
                        ((SOCKADDR_T*)name)->sin_family = A_CPU2LE16(((SOCKADDR_T*)name)->sin_family);
                        ((SOCKADDR_T*)name)->sin_addr = A_CPU2LE32(((SOCKADDR_T*)name)->sin_addr);
                }
                else
                {
                        A_MEMCPY(name, &((SOCK_ACCEPT_RECV_T*)(ath_sock_context[index]->data))->addr.name6, sizeof(SOCKADDR_6_T));
                        ((SOCKADDR_6_T*)name)->sin6_port = A_CPU2LE16(((SOCKADDR_6_T*)name)->sin6_port);
                        ((SOCKADDR_6_T*)name)->sin6_family = A_CPU2LE16(((SOCKADDR_6_T*)name)->sin6_family);
                }
                //A_FREE(ath_sock_context[index]->data, MALLOC_ID_CONTEXT);
               
                ath_sock_context[index]->data = NULL;
            }
            result = ath_sock_context[index]->result;
            A_NETBUF_FREE(ath_sock_context[index]->pReq); 
        }
        else
        {
            /* prepare wmi accept structure*/
            sock_accept.handle = A_CPU2LE32(handle);
            sock_accept.length = A_CPU2LE16(length);

            SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_ACCEPT);

            //This is a non-blocking accept
            if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_ACCEPT, (void *)(&sock_accept), sizeof(SOCK_ACCEPT_CMD_T))!= A_OK){
                SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_ACCEPT);
                result = A_ERROR;
                break;
            }
        }
    }while(0);
    
    return result;
}

#endif //T_SELECT_VER1



/*****************************************************************************/
/*  Api_errno - API to fetch last error code from target  
 *            
 *  A_VOID *pCxt- driver context 
 *  A_UINT32 handle- socket handle    
 * Returns- error code in case of success, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_errno(A_VOID *pCxt, A_UINT32 handle)
{
    A_DRIVER_CONTEXT *pDCxt;
    A_INT32 index = 0;
    A_INT32 result = A_OK;
    SOCK_ERRNO_T sock_errno;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    
    do{
        /*Find context*/
        if((index = find_socket_context(handle,TRUE)) == SOCKET_NOT_FOUND)
        {
            result = A_ERROR;
            break;
        }
        if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
            result = A_ERROR;
            break;
        }
        sock_errno.errno = A_CPU2LE32(handle);
        SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_ERRNO);

        if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_ERRNO, (A_UINT8*)&sock_errno, sizeof(SOCK_ERRNO_T)) != A_OK){
            SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_ERRNO);
            result = A_ERROR;
            break;
        }
    
        do{
            if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
                A_ASSERT(0);
            }        
        }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_ERRNO));
        result = ath_sock_context[index]->result;
    }while(0);
    
    return result;	
}


/*****************************************************************************/
/*  Api_select -   
 *            
 *  A_VOID *pCxt- driver context 
 *  A_UINT32 handle- socket handle    
 *  A_UINT32 tv - time to wait in milliseconds
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *  Select can be called to check for incoming connections or for incoming data.
 *  It waits for a specified time period before returning. This call does not 
 *  propagate to the target, it is consumed in the host.
 *****************************************************************************/
A_INT32 Api_select(A_VOID *pCxt, A_UINT32 handle, A_UINT32 tv)
{
    A_INT32 index = 0;
    A_INT32 result = A_OK;
    
    do{
        /*Find context*/
        if((index = find_socket_context(handle,TRUE)) == SOCKET_NOT_FOUND){
            result = A_SOCK_INVALID;
            break;
        }
    
        if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
            result = A_ERROR;
            break;
        }
        if((ath_sock_context[index]->type == SOCK_STREAM_TYPE) && 
           (ath_sock_context[index]->remaining_bytes != 0) &&
            (ath_sock_context[index]->old_netbuf != NULL))
        {
             result =  A_OK;
             break;
        }   
        /*Packet is available, return*/
        if(!CUSTOM_QUEUE_EMPTY(index)){
            result = A_OK;
            break;
        }

	if(ath_sock_context[index]->TCPCtrFlag == TCP_FIN)
	{		
		if(CUSTOM_QUEUE_EMPTY(index)) 
		{
			clear_socket_context(index);
			result =  A_SOCK_INVALID;
			CUSTOM_OS_INT_ENABLE(0);
			break;
		}
	}	
   
        if(ath_sock_context[index]->TCPCtrFlag ==  TCP_LISTEN)
	{
        /*Wait for specified time*/
        if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], tv, RX_DIRECTION) != A_OK)
        {
            /*Check if Peer closed socket while we were waiting*/
            if(ath_sock_context[index]->handle == 0){
                result = A_SOCK_INVALID;
                break;
            }
            /*Timeout, no activity detected*/
            result = A_ERROR;
        }
        else
        {
            /*Something is available, it may be a new incoming connection or data*/ 
            if((!CUSTOM_QUEUE_EMPTY(index)) || (ath_sock_context[index]->result == TCP_CONNECTION_AVAILABLE))
            {
                ath_sock_context[index]->result = 0;
                result = A_OK;
            }
            else{
                result = A_ERROR;
            }
        } 
	}
        else
	{	
        /*Wait for specified time*/
        if(CUSTOM_BLOCK_FOR_DATA(pCxt, ath_sock_context[index], tv, RX_DIRECTION) != A_OK)
        {
            /*Check if Peer closed socket while we were waiting*/
            if(ath_sock_context[index]->handle == 0){
                result = A_SOCK_INVALID;
                break;
            }
            /*Timeout, no activity detected*/
            result = A_ERROR;
        }
        else
        {
            /*Something is available, it may be a new incoming connection or data*/ 
            if((!CUSTOM_QUEUE_EMPTY(index)) || (ath_sock_context[index]->result == TCP_CONNECTION_AVAILABLE))
            {
                ath_sock_context[index]->result = 0;
                result = A_OK;
            }
            else{
                result = A_ERROR;
            }
        }
	}
    }while (0);
    return result;
}




#if T_SELECT_VER1
/*****************************************************************************/
/*  Api_select -   
 *            
 *  A_VOID *pCxt- driver context 
 *  A_UINT32 handle- socket handle    
 *  A_UINT32 tv - time to wait in milliseconds
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *  Select can be called to check for incoming connections or for incoming data.
 *  It waits for a specified time period before returning. This call does not 
 *  propagate to the target, it is consumed in the host.
 *****************************************************************************/

A_INT32 Api_select_ver1(A_VOID* pCxt, A_INT32 num, A_UINT32 *r_fd, A_UINT32 *w_fd, A_UINT32 *e_fd, A_UINT32 tv)
{
    A_INT32 index = 0;
    A_INT32 result = 0;
    A_INT32 first_time = 1;
    A_UINT32 r_mask, w_mask;
    HTC_ENDPOINT_ID    eid;
    A_ENDPOINT_T *pEp; 
	static A_UINT32 next_txsock = 0;
	A_UINT32 startsock;
	A_UINT8 credits;

    w_mask = r_mask = 0;

    // w_fd and r_fd are used to return the socket indexes which have some
    // activity. Take a backup and initialize them to zero
    if(w_fd)
    {
	w_mask = *w_fd;
	*w_fd = 0;
    }
    if(r_fd)
    {
	r_mask = *r_fd;
	*r_fd = 0;
    }

    if((w_mask ==0) && (r_mask==0))
    {
	//Nothing to do 
	return result;
    }

        
AGAIN:
    //Check if we have enough credits
    if(w_mask)
    {
		eid = Util_AC2EndpointID (pCxt, WMM_AC_BE);  
		pEp = Util_GetEndpoint(pCxt, eid);
		credits = pEp->credits;
		
		if(credits > 0) /* Credits Available */
		{
			/* Save the last serviced socket and start from the next one to avoid
			 * servicing the first socket alone in case of only one credit available
			 */
			startsock = next_txsock;
			for(index = startsock; index < (MAX_SOCKETS_SUPPORTED + startsock); index++)
			{  
				index %= MAX_SOCKETS_SUPPORTED;
				if(w_mask & (1<<index))
				{
					if(ath_sock_context[index]->handle != 0)
					{
						*w_fd |= 1<<index;
						result+=1;
						
						next_txsock = index + 1;
						credits--;
					}
					else
					{
						//STALE socket. Update the mask for this handle. The next
						//read will return 0/-1
						*(w_fd) |= (1<<index);
						result += 1;
					}
				}
				/* If there are no credits, break */
				if(!credits)
					break;
			}
		} /* credits > 0 */
    } /* w_mask */
    
    //Go through all read handles
    if(r_mask)
    {
	for(index=0; index<MAX_SOCKETS_SUPPORTED; index++)
	{
	    if(r_mask & (1<<index)) 
	    {
		if(ath_sock_context[index]->handle != 0)
		{
#if 0
		    if(/*(ath_sock_context[index]->type == SOCK_STREAM_TYPE) && */
			    (ath_sock_context[index]->remaining_bytes != 0) &&
			    (ath_sock_context[index]->old_netbuf != NULL))
		    {
			//Update the mask for this handle
			*(r_fd) |= (1<<index);
			result += 1;
			continue;
		    }   
#endif
		    /*Packet is available*/
		    if(!CUSTOM_QUEUE_EMPTY(index) || (ath_sock_context[index]->result == TCP_CONNECTION_AVAILABLE) || (ath_sock_context[index]->data != NULL)){
			//Update the mask for this handle
			*(r_fd) |= (1<<index);
			result += 1;
		    }
		}
		else //if( (r_mask & (1<<index)) && (ath_sock_context[index]->handle == 0) )
		{
		    //STALE socket. Update the mask for this handle. The next
		    //read will return 0/-1
		    *(r_fd) |= (1<<index);
		    result += 1;
		}
	    }
	}
    }

    //If no pending activity, wait for the time requested and again check for
    //activity
    if( (result ==0) && (first_time))
    {
	first_time = 0;
	//Wait for the specified time
	if(CUSTOM_BLOCK_SELECT(pCxt, tv) == A_OK)
	{ 
	    //Some activity happened because of which we were woken up. Figure
	    //out the activity and set the mask accordingly
	    goto AGAIN;
	}
    }

    return result;
}

#endif


/*****************************************************************************/
/*  Api_setsockopt - API to set specified socket option   
 *            name field must be populated accordingly. 
 *  A_VOID *pCxt- driver context 
 *  A_UINT32 handle- socket handle    
 *  A_UINT32 level- option level 
 *  A_UINT32 optname- option name
 *  A_UINT32 optlen- option length
 * Returns- 0 in case of success, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_setsockopt(A_VOID *pCxt, A_UINT32 handle, A_UINT32 level, A_UINT32 optname, A_UINT8* optval, A_UINT32 optlen)
{
    A_DRIVER_CONTEXT *pDCxt;
    SOCK_OPT_T* sockSetopt;
    //	A_UINT8* data = NULL;
    A_INT32 index = 0, total_length = 0;
    A_INT32 result = A_OK;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    
    do{
        /*Find context*/
        if((index = find_socket_context(handle,TRUE)) == SOCKET_NOT_FOUND)
        {
            result = A_ERROR;
            break;
        }
        if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
            result = A_ERROR;
            break;
        }    
        total_length = sizeof(SOCK_OPT_T);
    
        if(optval != NULL && optlen != 0)
        {
            total_length += optlen - sizeof(A_UINT8);						
        }
    
        /*Allocate space for option*/
        if((sockSetopt = A_MALLOC(total_length,MALLOC_ID_CONTEXT)) == NULL){
            result = A_NO_MEMORY;            
            break;
        }
    
        if(optval != NULL && optlen != 0)
        {
            A_MEMCPY(sockSetopt->optval, optval, optlen);					
        }		
            
        sockSetopt->handle = A_CPU2LE32(handle);
        sockSetopt->level = A_CPU2LE32(level);
        sockSetopt->optname = A_CPU2LE32(optname);
        sockSetopt->optlen = A_CPU2LE32(optlen);	

        SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_SETSOCKOPT);
                
        if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_SETSOCKOPT, (A_UINT8*)sockSetopt, total_length)!=A_OK){
            SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_SETSOCKOPT);
            result = A_ERROR;
            break;
        }
            
        do{
            if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
                A_ASSERT(0);
            }
        }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_SETSOCKOPT));
    
        A_FREE(sockSetopt, MALLOC_ID_CONTEXT);	
        result = ath_sock_context[index]->result;
    }while(0);	
    
    return result;	
}


/*****************************************************************************/
/*  Api_getsockopt - API to fetch specified socket option   
 *            name field must be populated accordingly. 
 *  A_VOID *pCxt- driver context 
 *  A_UINT32 handle- socket handle    
 *  A_UINT32 level- option level 
 *  A_UINT32 optname- option name
 *  A_UINT32 optlen- option length
 * Returns- 0 in case of success, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_getsockopt(A_VOID *pCxt, A_UINT32 handle, A_UINT32 level, A_UINT32 optname, A_UINT8* optval, A_UINT32 optlen)
{
    A_DRIVER_CONTEXT *pDCxt;
    SOCK_OPT_T* sock_getopt;
    A_UINT32 index = 0, total_length = 0;
    A_INT32 result = A_OK;
    A_UINT8 free_buf = 1;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   

    do{    
        /*Find context*/
        if((index = find_socket_context(handle,TRUE)) == SOCKET_NOT_FOUND)
        {
            result = A_ERROR;
            free_buf = 0;
            break;
        }
    
        /*Check if socket is blocked for a previous command*/
        if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
            result = A_ERROR;
            free_buf = 0;            
            break;
        }
        /*Cannot call getsockopt with NULL*/
        if(optval == NULL || optlen == 0){
            result = A_ERROR;
            free_buf = 0;            
            break;
        }
        /*Total length depends upon the type of option*/
        total_length = sizeof(SOCK_OPT_T) + optlen - sizeof(A_UINT8);	
    
        /*Allocate buffer for option*/
        if((sock_getopt = A_MALLOC(total_length,MALLOC_ID_CONTEXT)) == NULL){
            result = A_NO_MEMORY;
            free_buf = 0;            
            break;
        }
           
        sock_getopt->handle = A_CPU2LE32(handle);
        sock_getopt->level = A_CPU2LE32(level);
        sock_getopt->optname = A_CPU2LE32(optname);
        sock_getopt->optlen = A_CPU2LE32(optlen);	
        
        SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_GETSOCKOPT);            
        /*Send the packet*/	
        if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_GETSOCKOPT, (A_UINT8*)sock_getopt, total_length) != A_OK){
            SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_GETSOCKOPT);
            result = A_ERROR;
            free_buf = 0;            
            break;
        }
    
        /*Wait for response from stack*/
        do{
            if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
                A_ASSERT(0);
            }
        }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_GETSOCKOPT));	
    
        if(ath_sock_context[index]->data != NULL)
        {
            A_MEMCPY(optval,((SOCK_OPT_T*)ath_sock_context[index]->data)->optval, optlen);
            //A_FREE(ath_sock_context[index]->data,MALLOC_ID_CONTEXT);
            ath_sock_context[index]->data = NULL;
        }
        result = ath_sock_context[index]->result;
    }while(0);
    
    A_FREE(sock_getopt,MALLOC_ID_CONTEXT);
    
    if(free_buf){
        /*Free the netbuf*/
        A_NETBUF_FREE(ath_sock_context[index]->pReq);
    }
    
    return result;	
}



/*****************************************************************************/
/*  Api_ipconfig - API to obtain IP address information from target 
 *  A_VOID *pCxt- driver context 
 *  A_UINT32 mode- query vs set    
 *  A_UINT32* ipv4_addrA_UINT32* subnetMask, A_UINT32* gateway4
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_ipconfig(A_VOID *pCxt, A_UINT32 mode,A_UINT32* ipv4_addr, A_UINT32* subnetMask, A_UINT32* gateway4,IP46ADDR *dnsaddr,A_CHAR *hostname)
{
    A_DRIVER_CONTEXT *pDCxt;
    IPCONFIG_CMD_T ipcfg;
    IPCONFIG_RECV_T *result;
    A_UINT32 index = GLOBAL_SOCK_INDEX; //reserved for global commands ToDo- cleanup later    
    A_INT32 res = A_OK;
    A_UINT8 free_buf = 1;
    
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    
    do{
        /*Check if socket is blocked for a previous command*/
        if(IS_SOCKET_BLOCKED(ath_sock_context[GLOBAL_SOCK_INDEX])){
            res = A_ERROR;
            free_buf = 0;
            break;
        }
        if(mode == 1)
        {
            /*This is not a query or dhcp command*/
            ipcfg.mode = A_CPU2LE32(mode);
            ipcfg.ipv4 = A_CPU2LE32(*ipv4_addr);
            ipcfg.subnetMask = A_CPU2LE32(*subnetMask);
            ipcfg.gateway4 = A_CPU2LE32(*gateway4);	
        }
        ipcfg.mode = A_CPU2LE32(mode);	
        SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_IPCONFIG);
        
        if(mode == IPCFG_DHCP)
        {
            if(A_OK != move_power_state_to_maxperf(pDCxt, 2))/* refer POWER_STATE_MOVED_FOR_IPDHCP */
            {
                res = A_ERROR;
                free_buf = 0;
                break;
            }
        }

        if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_IPCONFIG, (void *)(&ipcfg), sizeof(IPCONFIG_CMD_T))!=A_OK){
            SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_IPCONFIG);
            res = A_ERROR;
            free_buf = 0;
            break;
        }
    
        do{
            if(mode == 2)
            {
                if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], DHCP_WAIT_TIME, RX_DIRECTION) != A_OK){
                    if(mode == IPCFG_DHCP)
                    {
                        restore_power_state(pDCxt, 2);/* refer POWER_STATE_MOVED_FOR_IPDHCP */
                    }
                    return (-1);
                }	
            }
             else
             {    
                if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
                    A_ASSERT(0);
                }
             } 
      
        }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_IPCONFIG));	
    
        if(mode == IPCFG_DHCP)
        {
            restore_power_state(pDCxt, 2);/* refer POWER_STATE_MOVED_FOR_IPDHCP */
        }
        /*Got a response*/
        if(ath_sock_context[index]->result != -1)
        {
            result = (IPCONFIG_RECV_T*)(ath_sock_context[index]->data);
            if(mode != 1)
            {
                *ipv4_addr = A_CPU2LE32(result->ipv4);
                *subnetMask = A_CPU2LE32(result->subnetMask);
                *gateway4 = A_CPU2LE32(result->gateway4);
                if(dnsaddr != NULL) 
                    memcpy(dnsaddr,&result->dnsaddr,sizeof(result->dnsaddr));
                if(hostname != NULL)
                    strcpy(hostname,result->hostname);	 
            }		
        }
        res = ath_sock_context[index]->result;
    }while(0);
    
    if(free_buf){
        /*Free the netbuf*/
        A_NETBUF_FREE(ath_sock_context[index]->pReq);
    }
    return res;	
}

/*****************************************************************************/
/*  Api_ip6config - API to obtain IPv6 address information from target 
 *  A_VOID *pCxt- driver context 
 *  A_UINT32 mode- query vs set    
 *  
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_ip6config(A_VOID *pCxt, A_UINT32 mode,IP6_ADDR_T *v6Global, IP6_ADDR_T *v6Link, IP6_ADDR_T *v6DefGw,IP6_ADDR_T *v6GlobalExtd, A_INT32 *LinkPrefix,
		      A_INT32 *GlbPrefix, A_INT32 *DefgwPrefix, A_INT32 *GlbPrefixExtd)
{
    A_DRIVER_CONTEXT *pDCxt;
    IPCONFIG_CMD_T ipcfg;
    IPCONFIG_RECV_T *result;
    A_UINT32 index = GLOBAL_SOCK_INDEX; //reserved for global commands ToDo- cleanup later
    A_INT32 res = A_OK;
    A_UINT8 free_buf = 1;

    pDCxt = GET_DRIVER_COMMON(pCxt);   

    do{    
        /*Check if socket is blocked for a previous command*/
        if(IS_SOCKET_BLOCKED(ath_sock_context[GLOBAL_SOCK_INDEX])){
            res = A_ERROR;
            free_buf = 0;
            break;
        }
            
        ipcfg.mode = A_CPU2LE32(mode);	

        SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_IP6CONFIG);

        if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_IP6CONFIG, (void *)(&ipcfg), sizeof(IPCONFIG_CMD_T))!= A_OK){
            SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_IP6CONFIG);
            res = A_ERROR;
            free_buf = 0;
            break;
        }
    
        do{
            if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
                A_ASSERT(0);
            }
   
        }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_IP6CONFIG));
    
        /*Got a response*/
        if(ath_sock_context[index]->result != -1)
        {
            
            result = (IPCONFIG_RECV_T*)(ath_sock_context[index]->data);
    
             A_MEMCPY(v6Global,&result->ipv6GlobalAddr,sizeof(IP6_ADDR_T));
             A_MEMCPY(v6Link,&result->ipv6LinkAddr,sizeof(IP6_ADDR_T));
             A_MEMCPY(v6DefGw,&result->ipv6DefGw,sizeof(IP6_ADDR_T));
             A_MEMCPY(v6GlobalExtd,&result->ipv6LinkAddrExtd,sizeof(IP6_ADDR_T));
             *LinkPrefix = A_CPU2LE32(result->LinkPrefix);
             *GlbPrefix = A_CPU2LE32(result->GlbPrefix);
             *DefgwPrefix = A_CPU2LE32 (result->DefGwPrefix);
             *GlbPrefixExtd = A_CPU2LE32 (result->GlbPrefixExtd);
                                             
        }
        res = ath_sock_context[index]->result;
    }while(0);

    if(free_buf){ 
        /*Free the netbuf*/
        A_NETBUF_FREE(ath_sock_context[index]->pReq);
    }
    
    return res;	
}


/*****************************************************************************/
/*  Api_ipconfig_dhcp_pool - API to configure dhcp pool 
 *  A_VOID *pCxt- driver context 
 *  A_UINT32 *start_ipv4_addr -   Start ip address    
 *  A_UINT32* end_ipv4_addr A_INT32 leasetime
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_ipconfig_dhcp_pool(A_VOID *pCxt,A_UINT32* start_ipv4_addr, A_UINT32* end_ipv4_addr, A_INT32 leasetime)
{
    A_DRIVER_CONTEXT *pDCxt;
    IPCONFIG_DHCP_POOL_T ipcfg;
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[GLOBAL_SOCK_INDEX])){
            return A_ERROR;
    }
            ipcfg.startaddr = A_CPU2LE32(*start_ipv4_addr);
            ipcfg.endaddr = A_CPU2LE32(*end_ipv4_addr);
            ipcfg.leasetime = A_CPU2LE32(leasetime);	

    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_IPCONFIG_DHCP_POOL, (void *)(&ipcfg), sizeof(IPCONFIG_DHCP_POOL_T))!=A_OK){
        return A_ERROR;
    }
    
    return (A_OK);	
}

/*****************************************************************************/
/*  Api_ip6config_router_prefix - API to configure router prefix 
 *  A_VOID *pCxt- driver context 
 *  A_UINT8 *v6addr -   v6prefix    
 *  A_INT32  prefixlen A_INT32 prefix_lifetime ,A_INT32 valid_lifetime 
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_ip6config_router_prefix(A_VOID *pCxt,A_UINT8 *v6addr,A_INT32 prefixlen,A_INT32 prefix_lifetime,A_INT32 valid_lifetime)
{
    A_DRIVER_CONTEXT *pDCxt;
    IP6CONFIG_ROUTER_PREFIX_T ip6cfg;
    pDCxt = GET_DRIVER_COMMON(pCxt);   

    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[GLOBAL_SOCK_INDEX])){
            return A_ERROR;
    }
    A_MEMCPY(ip6cfg.v6addr,v6addr,16);
    ip6cfg.prefixlen = A_CPU2LE32(prefixlen);
    ip6cfg.prefix_lifetime = A_CPU2LE32(prefix_lifetime);
    ip6cfg.valid_lifetime = A_CPU2LE32(valid_lifetime);

    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_IP6CONFIG_ROUTER_PREFIX, (void *)(&ip6cfg), sizeof(IP6CONFIG_ROUTER_PREFIX_T))!=A_OK){
        return A_ERROR;
    }
    return (A_OK);
}
	
/*****************************************************************************/
/*  Api_ipconfig_set_tcp_exponential_backoff_retry - API to tcp exponential backoff retry 
 *  A_VOID *pCxt- driver context 
 *  A_INT32 retry -   No of MAX Retries
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_ipconfig_set_tcp_exponential_backoff_retry(A_VOID *pCxt,A_INT32 retry)
{
    A_DRIVER_CONTEXT *pDCxt = NULL;
    SOCK_IP_BACKOFF_T sock_backoff;

    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[GLOBAL_SOCK_INDEX])){
            return A_ERROR;
    }
    sock_backoff.max_retry = A_CPU2LE32(retry);
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_IP_SET_TCP_EXP_BACKOFF_RETRY,(void *)(&sock_backoff), sizeof(SOCK_IP_BACKOFF_T))!=A_OK){
        return A_ERROR;
    }
    return (A_OK);
}	

/*****************************************************************************/
/*  Api_ipconfig_set_ip6_status - API to set ip6 status 
 *  A_VOID *pCxt- driver context 
 *  A_UINT16 status -   enable or diable
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_ipconfig_set_ip6_status(A_VOID *pCxt,A_UINT16 status)
{
    A_DRIVER_CONTEXT *pDCxt = NULL;
    SOCK_IPv6_STATUS_T sock_ip6status;

    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[GLOBAL_SOCK_INDEX])){
            return A_ERROR;
    }
    sock_ip6status.ipv6_status = A_CPU2LE16(status);
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_IP_SET_IP6_STATUS,(void *)(&sock_ip6status),sizeof(sock_ip6status))!=A_OK){
        return A_ERROR;
    }
    return (A_OK);
}

/*****************************************************************************/
/*  Api_ipconfig_dhcp_release - API to release dhcp ip address 
 *  A_VOID *pCxt- driver context 
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_ipconfig_dhcp_release(A_VOID *pCxt)
{

    A_DRIVER_CONTEXT *pDCxt = NULL;
    SOCK_IP_DHCP_RELEASE_T release;
    A_UINT16 ifindx = 0;

    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[GLOBAL_SOCK_INDEX])){
            return A_ERROR;
    }
    release.ifIndex = A_CPU2LE16(ifindx);
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_IP_DHCP_RELEASE,(void *)(&release),sizeof(SOCK_IP_DHCP_RELEASE_T))!=A_OK){
        return A_ERROR;
    }
    return (A_OK);
}

A_INT32 Api_ipconfig_set_tcp_rx_buffer(A_VOID *pCxt,A_INT32 rxbuf)
{

    A_DRIVER_CONTEXT *pDCxt = NULL;
    SOCK_IP_TCP_RX_BUF_T sock_tcp_rx_buf;

    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[GLOBAL_SOCK_INDEX])){
            return A_ERROR;
    }
    sock_tcp_rx_buf.rxbuf = A_CPU2LE32(rxbuf);
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_IP_SET_TCP_RX_BUF,(void *)(&sock_tcp_rx_buf), sizeof(SOCK_IP_TCP_RX_BUF_T))!=A_OK){
        return A_ERROR;
    }
    return (A_OK);

}

#if ENABLE_HTTP_SERVER
A_INT32 Api_ip_http_server(A_VOID *pCxt,A_INT32 command)
{

    A_DRIVER_CONTEXT *pDCxt = NULL;
    SOCK_IP_HTTP_SERVER_T sock_http_server;
    int index = GLOBAL_SOCK_INDEX;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
        return A_ERROR;
    }
    sock_http_server.enable = A_CPU2LE32(command);
    
    SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_HTTP_SERVER);
    
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_HTTP_SERVER,(void *)(&sock_http_server), sizeof(SOCK_IP_HTTP_SERVER_T))!=A_OK){
        SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_HTTP_SERVER);
        return A_ERROR;
    }
    
    do{
        if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
            A_ASSERT(0);
        }
    }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_HTTP_SERVER));
    
    return (ath_sock_context[index]->result);
}

A_INT32 Api_ip_http_server_method(A_VOID *pCxt, A_INT32 command, A_UINT8 *pagename, A_UINT8 *objname, A_INT32 objtype, A_INT32 objlen, A_UINT8 *value)
{
    A_DRIVER_CONTEXT *pDCxt = NULL;
    SOCK_IP_HTTP_SERVER_CMD_T sock_http_server;
    int index = GLOBAL_SOCK_INDEX;
    A_UINT32 *length;
    A_UINT8 *buf;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
        return A_ERROR;
    }
    sock_http_server.command = A_CPU2LE32(command);
    strcpy((char*)sock_http_server.pagename, (char*)pagename);
    strcpy((char*)sock_http_server.objname, (char*)objname);
    sock_http_server.objlen = A_CPU2LE32(objlen);
    if (command == 0) /* For POST command */
        strcpy((char*)sock_http_server.value, (char*)value);
    
    SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_HTTP_SERVER_CMD);
    
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_HTTP_SERVER_CMD,(void *)(&sock_http_server), sizeof(SOCK_IP_HTTP_SERVER_CMD_T))!=A_OK){
        SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_HTTP_SERVER_CMD);
        return A_ERROR;
    }
    
    do{
        if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
            A_ASSERT(0);
        }
    }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_HTTP_SERVER_CMD));
    
    if((ath_sock_context[index]->result != -1) && (command == 1))
    {
        length = (A_UINT32 *)ath_sock_context[index]->data;
        if (*length > 512)
            A_ASSERT(0);
        buf = (A_UINT8 *)(length + 1);
        A_MEMCPY(value, buf, *length);
        value[*length] = '\0'; //NULL terminating the string
    }
    
    A_NETBUF_FREE(ath_sock_context[index]->pReq);
    
    return (ath_sock_context[index]->result);   
}
#endif

#if ENABLE_HTTP_CLIENT
/*****************************************************************************/
/*  Api_httpc_method - API to use http client methods
 *                      It is a non blocking call. Just sends the HTTP command and return
 *  A_VOID *pCxt- driver context 
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *****************************************************************************/

A_INT32 Api_httpc_method(A_VOID* pCxt, A_UINT32 command, A_UINT8 *url, A_UINT8 *data, A_UINT8 **output)
{
    A_DRIVER_CONTEXT *pDCxt = NULL;
    SOCK_HTTPC_T httpc;
    A_INT32 index = GLOBAL_SOCK_INDEX;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);
    
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
        return A_ERROR;
    }

    memset(&httpc , 0, sizeof(httpc));
    
    httpc.command = A_CPU2LE32(command);  
    if (url)
        A_MEMCPY(httpc.url, url, strlen((char*)url));
    if (data)
        A_MEMCPY(httpc.data, data, strlen((char*)data));
    
    SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_HTTPC);
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_HTTPC,(void *)(&httpc),sizeof(SOCK_HTTPC_T))!=A_OK){
        SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_HTTPC);
        return A_ERROR;
    }
    
    do{
        if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
            A_ASSERT(0);
        }      
    }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_HTTPC));

    if((ath_sock_context[index]->result != A_ERROR) && (output != NULL))
    {
        *output = ath_sock_context[index]->data;
       // printf("Adding Q %p %p\n", ath_sock_context[index]->pReq, ath_sock_context[index]->data);
        A_NETBUF_ENQUEUE(&zero_copy_free_queue, ath_sock_context[index]->pReq);
    }
    else
        A_NETBUF_FREE(ath_sock_context[index]->pReq);
    
    return ath_sock_context[index]->result;
}

#endif /* ENABLE_HTTP_CLIENT */

#if ENABLE_DNS_CLIENT
A_INT32 Api_ip_resolve_host_name(A_VOID *pCxt,DNC_CFG_CMD *DncCfg,DNC_RESP_INFO *DncRespInfo)
{
    A_DRIVER_CONTEXT *pDCxt = NULL;
    int index = GLOBAL_SOCK_INDEX;
    DNC_RESP_INFO *result = NULL;
    A_INT32 res = A_OK;
    A_UINT8 free_buf = 1;

    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
        free_buf = 0;
        return A_ERROR;
    }
    DncCfg->mode = A_CPU2LE32(DncCfg->mode); 
    DncCfg->domain = A_CPU2LE32(DncCfg->domain);    
    SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_DNC_CMD);
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_DNC_CMD,(void *)DncCfg, sizeof(DNC_CFG_CMD))!=A_OK){
        SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_DNC_CMD);
	free_buf = 0;
        return A_ERROR;
    }

    do{
        if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], (DHCP_WAIT_TIME *2), RX_DIRECTION) != A_OK){
            //printf("dnsclient timed out \r\n");
            return -1;
        }
    }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_DNC_CMD));

   if(ath_sock_context[index]->result != -1)
   {
       result = (DNC_RESP_INFO *)(ath_sock_context[index]->data);
       A_MEMCPY(DncRespInfo,result,sizeof(DNC_RESP_INFO));
   }
   res = ath_sock_context[index]->result;
   /*Free the netbuf*/
   if(free_buf)
   {  
     A_NETBUF_FREE(ath_sock_context[index]->pReq);
   }  
   return res;
}

A_INT32 Api_ip_dns_client(A_VOID *pCxt,A_INT32 command)
{

    A_DRIVER_CONTEXT *pDCxt = NULL;
    SOCK_IP_DNS_CLIENT_T sock_dns_client;
    int index = GLOBAL_SOCK_INDEX;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
        return A_ERROR;
    }
    sock_dns_client.command = A_CPU2LE32(command);
    
    SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_DNC_ENABLE);
    
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_DNC_ENABLE,(void *)(&sock_dns_client), sizeof(SOCK_IP_DNS_CLIENT_T))!=A_OK){
        SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_DNC_ENABLE);
        return A_ERROR;
    }
    
    do{
        if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
            A_ASSERT(0);
        }
    }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_DNC_ENABLE));
    
    return (ath_sock_context[index]->result);
}

A_INT32 Api_ip_dns_server_addr(A_VOID *pCxt, IP46ADDR *addr)
{

    A_DRIVER_CONTEXT *pDCxt = NULL;
    SOCK_IP_CFG_DNS_SRVR_ADDR  sock_dns_srvr_addr;
    int index = GLOBAL_SOCK_INDEX;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
        return A_ERROR;
    }
   
    if(addr->type == ATH_AF_INET)
    {	    
      sock_dns_srvr_addr.addr.type = ATH_AF_INET;
      sock_dns_srvr_addr.addr.addr4 = addr->addr4;
      sock_dns_srvr_addr.addr.au1Rsvd[0] = addr->au1Rsvd[0];
    }	     
    if(addr->type == ATH_AF_INET6)
    {
      sock_dns_srvr_addr.addr.type = ATH_AF_INET6;
      sock_dns_srvr_addr.addr.au1Rsvd[0] = addr->au1Rsvd[0];
      memcpy(&sock_dns_srvr_addr.addr.addr6,&addr->addr6,sizeof(IP6_ADDR_T));
    }	    
    
    SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_DNS_SRVR_CFG_ADDR);
    
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_DNS_SRVR_CFG_ADDR,(void *)(&sock_dns_srvr_addr), sizeof(SOCK_IP_CFG_DNS_SRVR_ADDR))!=A_OK){
        SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_DNS_SRVR_CFG_ADDR);
        return A_ERROR;
    }
    
    do{
        if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
            A_ASSERT(0);
        }
    }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_DNS_SRVR_CFG_ADDR));
    
    return (ath_sock_context[index]->result);
}
#endif /* ENABLE_DNS_CLIENT */

A_INT32 Api_ip_hostname(A_VOID *pCxt, A_CHAR *domain_name)
{
    A_DRIVER_CONTEXT *pDCxt = NULL;
    SOCK_IP_CFG_HOST_NAME sock_ip_host_name;
    int index = GLOBAL_SOCK_INDEX;

    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
        return A_ERROR;
    }
    strcpy(sock_ip_host_name.domain_name,domain_name);  
    SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_IP_HOST_NAME);
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_IP_HOST_NAME,(void *)(&sock_ip_host_name), sizeof(SOCK_IP_CFG_HOST_NAME))!=A_OK){
        SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_IP_HOST_NAME);
        return A_ERROR;
    }

    do{
        if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
            A_ASSERT(0);
        }
    }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_IP_HOST_NAME));
    
    return (ath_sock_context[index]->result);

}

#if ENABLE_DNS_SERVER

A_INT32 Api_ip_dns_local_domain(A_VOID *pCxt,A_CHAR *domain_name)
{

    A_DRIVER_CONTEXT *pDCxt = NULL;
    SOCK_IP_CFG_DNS_LOCAL_DOMAIN  sock_dns_local_domain;
    int index = GLOBAL_SOCK_INDEX;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
        return A_ERROR;
    }
    strcpy(sock_dns_local_domain.domain_name,domain_name);  
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_DNS_LOCAL_DOMAIN,(void *)(&sock_dns_local_domain), sizeof(SOCK_IP_CFG_DNS_LOCAL_DOMAIN))!=A_OK){
        return A_ERROR;
    }
    return (A_OK);
}

A_INT32 Api_ipdns(A_VOID *pCxt,A_INT32 command,A_CHAR *domain_name, IP46ADDR *dnsaddr)
{

    A_DRIVER_CONTEXT *pDCxt = NULL;
    SOCK_IP_DNS_T sock_ip_dns;
    int index = GLOBAL_SOCK_INDEX;

    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
        return A_ERROR;
    }
    memset(&sock_ip_dns,0,sizeof(SOCK_IP_DNS_T));
    strcpy(sock_ip_dns.domain_name,domain_name);  
    sock_ip_dns.command = A_CPU2LE32(command);
     //printf("api_dns addr type %d \r\n",dnsaddr->type);    
     if(dnsaddr->type == ATH_AF_INET)
    {	    
      sock_ip_dns.addr.type = ATH_AF_INET;
      sock_ip_dns.addr.addr4 = dnsaddr->addr4;
      sock_ip_dns.addr.au1Rsvd[0] = dnsaddr->au1Rsvd[0];
    }	     
    if(dnsaddr->type == ATH_AF_INET6)
    {
      sock_ip_dns.addr.type = ATH_AF_INET6;
      sock_ip_dns.addr.au1Rsvd[0] = dnsaddr->au1Rsvd[0];
      memcpy(&sock_ip_dns.addr.addr6,&dnsaddr->addr6,sizeof(IP6_ADDR_T));
    }	 
    SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_IP_DNS);
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_IP_DNS,(void *)(&sock_ip_dns), sizeof(SOCK_IP_DNS_T))!=A_OK){
        SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_IP_DNS);
        return A_ERROR;
    }
    do{
        if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
            A_ASSERT(0);
        }
    }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_IP_DNS));

    return (ath_sock_context[index]->result);
}	
#endif /* ENABLE_DNS_SERVER */

#if ENABLE_SNTP_CLIENT
A_INT32 Api_ip_sntp_srvr_addr(A_VOID *pCxt,A_INT32 command,A_CHAR *sntp_srvr_addr)
{

    A_DRIVER_CONTEXT *pDCxt = NULL;
    SOCK_IP_CFG_SNTP_SRVR_ADDR  sock_ip_sntp_srvr_addr;
    int index = GLOBAL_SOCK_INDEX;
    
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
        return A_ERROR;
    }
    strcpy(sock_ip_sntp_srvr_addr.addr,sntp_srvr_addr);  
    sock_ip_sntp_srvr_addr.command = A_CPU2LE32(command);
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_IP_SNTP_SRVR_ADDR,(void *)(&sock_ip_sntp_srvr_addr), sizeof(SOCK_IP_CFG_SNTP_SRVR_ADDR))!=A_OK){
        return A_ERROR;
    }
    return (A_OK);
}

A_INT32 Api_ip_sntp_get_time(A_VOID *pCxt,tSntpTime *SntpTime)
{	
    A_DRIVER_CONTEXT *pDCxt = NULL;
    int index = GLOBAL_SOCK_INDEX;
    A_INT32 res = A_OK;
    A_UINT8 free_buf = 1;
    tSntpTime *result = NULL;
    A_UINT8 buffer[sizeof(WMI_SOCKET_CMD)+4];
    WMI_SOCKET_CMD *sock_cmd = (WMI_SOCKET_CMD *)&buffer[0];

    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
        free_buf = 0;
        return A_ERROR;
    }
    SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_IP_SNTP_GET_TIME);
    if(wmi_socket_cmd(pDCxt->pWmiCxt,SOCK_IP_SNTP_GET_TIME,(void *)&sock_cmd,(sizeof(WMI_SOCKET_CMD) + 4))!=A_OK){
        SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_IP_SNTP_GET_TIME);
	free_buf = 0;
        return A_ERROR;
    }
    do{
        if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], (DHCP_WAIT_TIME *2), RX_DIRECTION) != A_OK){
            return -1;
        }
    }while(SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_IP_SNTP_GET_TIME));

   if(ath_sock_context[index]->result != -1)
   {
       result = (tSntpTime *)(ath_sock_context[index]->data);
       A_MEMCPY(SntpTime,result,sizeof(tSntpTime));
   }
   res = ath_sock_context[index]->result;
   /*Free the netbuf*/
   if(free_buf)
   {  
     A_NETBUF_FREE(ath_sock_context[index]->pReq);
   }  
   return res;
}  

A_INT32 Api_ip_sntp_get_time_of_day(A_VOID *pCxt,tSntpTM *SntpTm)
{	
    A_DRIVER_CONTEXT *pDCxt = NULL;
    int index = GLOBAL_SOCK_INDEX;
    A_INT32 res = A_OK;
    A_UINT8 free_buf = 1;
    tSntpTM *result = NULL;
    A_UINT8 buffer[sizeof(WMI_SOCKET_CMD)+4];
    WMI_SOCKET_CMD *sock_cmd = (WMI_SOCKET_CMD *)&buffer[0];

    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
        free_buf = 0;
        return A_ERROR;
    }
    
    SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_IP_SNTP_GET_TIME_OF_DAY);
    if(wmi_socket_cmd(pDCxt->pWmiCxt,SOCK_IP_SNTP_GET_TIME_OF_DAY,(void *)&sock_cmd,(sizeof(WMI_SOCKET_CMD) + 4))!=A_OK){
        SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_IP_SNTP_GET_TIME_OF_DAY);
	free_buf = 0;
        return A_ERROR;
    }
    do{
        if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], (DHCP_WAIT_TIME *2), RX_DIRECTION) != A_OK){
            return -1;
        }
    }while(SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_IP_SNTP_GET_TIME_OF_DAY));

   
   if(ath_sock_context[index]->result != -1)
   {
       result = (tSntpTM *)(ath_sock_context[index]->data);
       A_MEMCPY(SntpTm,result,sizeof(tSntpTM));
   }
   res = ath_sock_context[index]->result;
   /*Free the netbuf*/
   if(free_buf)
   {  
     A_NETBUF_FREE(ath_sock_context[index]->pReq);
   }  
   return res;
}  




A_INT32 Api_ip_sntp_modify_zone_dse(A_VOID *pCxt,A_UINT8 hr,A_UINT8 mn,A_UINT8 zone_cal,A_UINT8 dse_en_dis)
{
    A_DRIVER_CONTEXT *pDCxt = NULL;
    int index = GLOBAL_SOCK_INDEX;
    A_INT32 res = A_OK;
    SOCK_SNTP_MODIFY_TIMEZONE sock_ip_sntp_time_zone;
   
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
        return A_ERROR;
    }
    sock_ip_sntp_time_zone.hour    = hr;  
    sock_ip_sntp_time_zone.min     = mn;  
    sock_ip_sntp_time_zone.add_sub = zone_cal;  
    sock_ip_sntp_time_zone.dse     = dse_en_dis;  

    SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_IP_SNTP_CONFIG_TIMEZONE_DSE);
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_IP_SNTP_CONFIG_TIMEZONE_DSE,(void *)(&sock_ip_sntp_time_zone), sizeof(SOCK_SNTP_MODIFY_TIMEZONE))!=A_OK){
        SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_IP_SNTP_CONFIG_TIMEZONE_DSE);
        return A_ERROR;
    }
    
   return res; 
}

A_INT32 Api_ip_sntp_query_srvr_address(A_VOID *pCxt,tSntpDnsAddr SntpDnsAddr[MAX_SNTP_SERVERS])
{
    A_DRIVER_CONTEXT *pDCxt = NULL;
    int index = GLOBAL_SOCK_INDEX;
    A_INT32 res = A_OK;
    A_UINT8 free_buf = 1;
    tSntpDnsAddr *result = NULL;
    SOCK_IP_QUERY_SNTP_CONFIG sock_ip_query_sntp_config;
   
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
	free_buf = 0;
        return A_ERROR;
    }

    SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_IP_SNTP_QUERY_SNTP_ADDRESS);
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_IP_SNTP_QUERY_SNTP_ADDRESS,(void *)(&sock_ip_query_sntp_config), sizeof(SOCK_IP_QUERY_SNTP_CONFIG))!=A_OK){
        SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_IP_SNTP_QUERY_SNTP_ADDRESS);
	free_buf = 0;
        return A_ERROR;
    }
    
    do{
	//printf("block on reso for sntp query \r\n");     
        if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], (DHCP_WAIT_TIME *2), RX_DIRECTION) != A_OK){
            return -1;
        }
    }while(SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_IP_SNTP_QUERY_SNTP_ADDRESS));

   if(ath_sock_context[index]->result != -1)
   {
       result = (tSntpDnsAddr *)(ath_sock_context[index]->data);
       A_MEMCPY(SntpDnsAddr,result,(sizeof(tSntpDnsAddr) * MAX_SNTP_SERVERS));
   }
   res = ath_sock_context[index]->result;
   /*Free the netbuf*/
   if(free_buf)
   {  
     A_NETBUF_FREE(ath_sock_context[index]->pReq);
   }  
   return res;
}

A_INT32 Api_ip_sntp_client(A_VOID *pCxt,A_INT32 command)
{

    A_DRIVER_CONTEXT *pDCxt = NULL;
    SOCK_IP_SNTP_CLIENT_T sock_sntp_client;
    int index = GLOBAL_SOCK_INDEX;
    A_INT32 res = A_OK;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    /*Check if socket is blocked for a previous command*/
    if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
        return A_ERROR;
    }
    sock_sntp_client.command = A_CPU2LE32(command);
    
    SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_IP_SNTP_CLIENT_ENABLE);
    
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_IP_SNTP_CLIENT_ENABLE,(void *)(&sock_sntp_client), sizeof(SOCK_IP_SNTP_CLIENT_T))!=A_OK){
        SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_IP_SNTP_CLIENT_ENABLE);
        return A_ERROR;
    }
     
    return (res);
}

#endif /* ENABLE_SNTP_CLIENT */
/*****************************************************************************/
/*  Api_ping - API to obtain IP address information from target 
 *  A_VOID *pCxt- driver context   
 *  A_UINT32* ipv4_addr
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_ping(A_VOID *pCxt, A_UINT32 ipv4_addr, A_UINT32 size)
{
    A_DRIVER_CONTEXT *pDCxt;
    PING_T ping;
    A_UINT32 index = GLOBAL_SOCK_INDEX; //reserved for global commands ToDo- cleanup later
    A_INT32 result = A_OK;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    
    do{
        /*Check if socket is blocked for a previous command*/
        if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
            result = A_ERROR;
            break;
        }
        ping.ip_addr = A_CPU2LE32(ipv4_addr);
        ping.size = A_CPU2LE32(size);	

        SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_PING);

        if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_PING, (void *)(&ping), sizeof(PING_T))!=A_OK){
            SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_PING);
            result = A_ERROR;
            break;
        }
    
        do{
            if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
                A_ASSERT(0);
            }
 
        }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_PING));	
        result = ath_sock_context[index]->result;
    }while(0);
    
    return result;	
}

/*****************************************************************************/
/*  Api_ping6 - API to obtain IP address information from target 
 *  A_VOID *pCxt- driver context   
 *  A_UINT32* ipv4_addr
 * Returns- 0 in case of successful connect, A_ERROR otherwise
 *****************************************************************************/
A_INT32 Api_ping6(A_VOID *pCxt, A_UINT8 *ip6addr, A_UINT32 size)
{
    A_DRIVER_CONTEXT *pDCxt;
    PING_6_T ping6;
    A_UINT32 index = GLOBAL_SOCK_INDEX; //reserved for global commands ToDo- cleanup later
    A_INT32 result = A_OK;
    
    pDCxt = GET_DRIVER_COMMON(pCxt);   
    
    do{
        /*Check if socket is blocked for a previous command*/
        if(IS_SOCKET_BLOCKED(ath_sock_context[index])){
          result = A_ERROR;
          break;
        }
        A_MEMCPY(ping6.ip6addr,ip6addr,sizeof(ping6.ip6addr));
        ping6.size = A_CPU2LE32(size);

        SOCK_EV_MASK_SET(ath_sock_context[index], SOCK_PING6);

        if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_PING6, (void *)(&ping6), sizeof(PING_6_T))!=A_OK){
            SOCK_EV_MASK_CLEAR(ath_sock_context[index], SOCK_PING6);
            result = A_ERROR;
            break;
        }
    
        do{
            if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK){
                A_ASSERT(0);
            }
        }while(SOCK_EV_MASK_TEST(ath_sock_context[index], SOCK_PING6));	
        result = ath_sock_context[index]->result;
    }while(0);
    
    return result;	
}

/*****************************************************************************/
/*  clear_socket_context - clears all elements of socket context 
 *  A_INT32 - index to socket context 
 * Returns- None
 *****************************************************************************/
A_VOID clear_socket_context(A_INT32 index)
{		
	ath_sock_context[index]->handle = 0;
	ath_sock_context[index]->domain = 0;
	ath_sock_context[index]->type = 0;
	ath_sock_context[index]->sock_st_mask = 0;
	ath_sock_context[index]->TCPCtrFlag = 0;
	
	/*If for some reason, data field is not freed, free it here
	  This may happen when a command response comes later than Timeout period*/
	if(ath_sock_context[index]->data)
	{
		A_FREE(ath_sock_context[index]->data,MALLOC_ID_CONTEXT);
		ath_sock_context[index]->data = NULL;	
	}
}


/*****************************************************************************/
/*  send_stack_init - Sends Stack initialization information.
 * RETURNS: A_OK on success A_ERROR otherwise
 *****************************************************************************/
A_STATUS send_stack_init(A_VOID* pCxt)
{
    A_DRIVER_CONTEXT *pDCxt;
    A_UINT8 buffer[sizeof(WMI_SOCKET_CMD)+4];
    WMI_SOCKET_CMD *sock_cmd = (WMI_SOCKET_CMD *)&buffer[0];
    pDCxt = GET_DRIVER_COMMON(pCxt);
    
    buffer[8] = 1;
    buffer[9] = 2;
    buffer[10] = MAX_SOCKETS_SUPPORTED;
    
    if(wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_STACK_INIT, (void *)(sock_cmd), sizeof(WMI_SOCKET_CMD)+4)!=A_OK){
        return A_ERROR;
    }  

#if ENABLE_SSL
    sslhandle = pCxt;
#endif

    return A_OK;       
}

enum {
    /* Least priority since we dont allow any operation during store-recall */
    POWER_STATE_MOVED_FOR_STRRCL = 1,
    /* IPDHCP will happen only after the network connection usually */
    POWER_STATE_MOVED_FOR_IPDHCP,
    POWER_STATE_MOVED_FOR_TCPRX
};

A_INT32 power_state_for_module = 0;
/*****************************************************************************/
/*  move_to_maxperf - Changes the power mode to max perf in firmware. There are
 *                    some commands which need to run in MAX_PERF mode like store_recall etc
 *                    This function changes the power modeto MAX_PERF and will not 
 *                    intimate the application. Later after the command succeeds,
 *                    use restore_power_state to change to original power state
 *                    Note: Application should not directly use this function
 *  A_INT32 - module which request the change
 *  Returns- A_OK on success, A_ERROR otherwise
 *****************************************************************************/
A_STATUS move_power_state_to_maxperf(void *pCxt, A_INT32 module)
{
    A_DRIVER_CONTEXT *pDCxt = (A_DRIVER_CONTEXT *)pCxt;
    
    /* Already some HIGH module changes state */
    if (power_state_for_module >= module)
    {
        return A_OK;
    }
    
    /* Change the power mode only when the current power mode is REC_POWER */
    if (pDCxt->userPwrMode==REC_POWER)
    {
        if(A_OK != wmi_cmd_start(pDCxt->pWmiCxt, &max_performance_power_param, WMI_SET_POWER_MODE_CMDID, sizeof(WMI_POWER_MODE_CMD)))
        {
            return A_ERROR;
        }
    }
    power_state_for_module = module;
    return A_OK;
}
/*****************************************************************************/
/*  restore_power_state - Restores the original power state, which is changed by
 *                        "move_power_state_to_maxperf"
 *  A_INT32 - module which request the change
 * RETURNS: A_OK on success A_ERROR otherwise
 *****************************************************************************/
A_STATUS restore_power_state(void *pCxt, A_INT32 module)
{
    A_DRIVER_CONTEXT *pDCxt = (A_DRIVER_CONTEXT *)pCxt;
    
    /* Already some HIGH priority module changes state,
     * so don't change the state now */
    if (!power_state_for_module || (power_state_for_module > module))
    {
        return A_OK;
    }
    if(pDCxt->userPwrMode!=max_performance_power_param)
    {
        if(A_OK!=wmi_cmd_start(pDCxt->pWmiCxt, &pDCxt->userPwrMode, WMI_SET_POWER_MODE_CMDID, sizeof(WMI_POWER_MODE_CMD)))
        {
            return A_ERROR;
        }
    }
    
    power_state_for_module = 0;
    return A_OK;
}

#if ENABLE_SSL
/*****************************************************************************/
/* SSL_ctx_new - Create new SSL context. This function must be called before
 *               using any other SSL functions. It needs to be called as either
 *               server or client
 * Sslrole role - 1 = server, 2 = client
 * A_INT32 inbufSize - initial inBuf size: Can grow
 * A_INT32 outbufSize - outBuf size: Fixed
 * A_INT32 reserved - currently not used (must be zero)
 * Returns - SSL context handle on success or NULL on error (out of memory)
 *****************************************************************************/
SSL_CTX* Api_SSL_ctx_new(A_VOID *pCxt,SSL_ROLE_T role, A_INT32 inbufSize, A_INT32 outbufSize, A_INT32 reserved)
{
    SOCK_SSL_CTX_NEW_T cmd;
    SSL_CTX *result = NULL;
    A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(pCxt);
    A_INT32 index = GLOBAL_SOCK_INDEX;

    do
    {

        /*Check if socket is blocked for a previous command*/
        if (IS_SOCKET_BLOCKED(ath_sock_context[index]))
        {
            break;
        }

        /* Send command and wait for response */
        cmd.role = role;
        cmd.inbufSize = inbufSize;
        cmd.outbufSize = outbufSize;
        cmd.reserved = reserved;
        SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_SSL_CTX_NEW);
        if (wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_SSL_CTX_NEW, (void *) (&cmd), sizeof(cmd)) != A_OK)
        {
            SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_SSL_CTX_NEW);
            break;
        }
        do
        {
            if (CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK)
            {
                A_ASSERT(0);
            }
        }while(SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_SSL_CTX_NEW));
        result = (SSL_CTX*) ath_sock_context[GLOBAL_SOCK_INDEX]->result;
    }
    while (0);

    return result;
}

/*****************************************************************************/
/* SSL_ctx_free - Free the SSL context
 * SSL_CTX *ctx - sslContext
 *****************************************************************************/
A_INT32 Api_SSL_ctx_free(A_VOID *pCxt,SSL_CTX *ctx)
{
    A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(pCxt);
    SOCK_SSL_CTX_FREE_T cmd;
    A_INT32 result = A_OK;
    A_INT32 index = GLOBAL_SOCK_INDEX;

    do
    {
        /*Check if socket is blocked for a previous command*/
        if (IS_SOCKET_BLOCKED(ath_sock_context[index]))
        {
            result = A_ERROR;
            break;
        }

        /* Send command and wait for response */
        cmd.ctx = (A_UINT32) ctx;
        SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_SSL_CTX_FREE);
        if (wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_SSL_CTX_FREE, (void *) (&cmd), sizeof(cmd)) != A_OK)
        {
            SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_SSL_CTX_FREE);
            result = A_ERROR;
            break;
        }
        do
        {
            if (CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK)
            {
                A_ASSERT(0);
            }
        }while(SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_SSL_CTX_FREE));
        result = ath_sock_context[GLOBAL_SOCK_INDEX]->result;
    }
    while (0);

    return result;
}

/*****************************************************************************/
/* SSL_new - Create SSL connection object. When SSL transaction is done, close
 *           it with ssl_shutdown().
 *           Note that the socket should subsequently also be closed.
 * SSL_CTX *ctx - sslContext
 * Return - SSL object handle on success or NULL on error (out of memory)
 *****************************************************************************/
SSL* Api_SSL_new(A_VOID *pCxt,SSL_CTX *ctx)
{
    A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(pCxt);
    SOCK_SSL_NEW_T cmd;
    SSL *result = NULL;
    A_INT32 index = GLOBAL_SOCK_INDEX;

    do
    {
        /*Check if socket is blocked for a previous command*/
        if (IS_SOCKET_BLOCKED(ath_sock_context[index]))
        {
            break;
        }

        /* Send command and wait for response */
        cmd.ctx = (A_UINT32)ctx;
        SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_SSL_NEW);
        if (wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_SSL_NEW, (void *)(&cmd), sizeof(cmd)) != A_OK)
        {
            SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_SSL_NEW);
            break;
        }
        do
        {
            if (CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK)
            {
                A_ASSERT(0);
            }
        }while(SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_SSL_NEW));
        result = (SSL*) ath_sock_context[index]->result;
    }
    while (0);

    return result;
}

/*****************************************************************************/
/* SSL_set_fd - Attach given socket descriptor to the SSL connection
 * SSL *ssl - SSL connection
 * A_UINT32 fd - Socket descriptor
 * Return - 1 on success or negative error code on error (see SslErrors)
 *****************************************************************************/
A_INT32 Api_SSL_set_fd(A_VOID *pCxt,SSL *ssl, A_UINT32 fd)
{
    A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(pCxt);
    SOCK_SSL_SET_FD_T cmd;
    A_INT32 result = A_OK;
    A_INT32 index;

    do
    {
        /* Find the socket */
        index = find_socket_context(fd, TRUE);
        if (index == SOCKET_NOT_FOUND)
        {
            return A_ERROR;
        }

        /* Check if socket is blocked for a previous command */
        if (IS_SOCKET_BLOCKED(ath_sock_context[index]))
        {
            return A_ERROR;
        }

        /* Store the ssl object pointer in the socket struct */
        ath_sock_context[index]->ssl = ssl;

        /* Send command and wait for response */
        cmd.ssl = (A_UINT32) ssl;
        cmd.fd = fd;
        SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_SSL_SET_FD);
        if (wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_SSL_SET_FD, (void *) (&cmd), sizeof(cmd)) != A_OK)
        {
            SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_SSL_SET_FD);
            result = A_ERROR;
            break;
        }
        do
        {
            if (CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK)
            {
                A_ASSERT(0);
            }
        }while(SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_SSL_SET_FD));
        result = ath_sock_context[index]->result;
    }
    while (0);

    if (result < 0)
    {
        /* remove the ssl pointer again if the cmd failed */
        ath_sock_context[index]->ssl = NULL;
    }

    return result;
}

/*****************************************************************************/
/* SSL_accept - Initiate SSL handshake.
 * SSL *ssl - SSL connection
 * Returns - 1 on success, ESSL_HSDONE if handshake has already been performed.
 *           Negative error code otherwise.
 *****************************************************************************/
A_INT32 Api_SSL_accept(A_VOID *pCxt,SSL *ssl)
{
    A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(pCxt);
    SOCK_SSL_ACCEPT_T cmd;
    A_INT32 result = A_OK;
    A_INT32 index;

    do
    {
        /* Find the socket used with this SSL connection */
        index = find_socket_context_from_ssl(ssl);
        if (index == SOCKET_NOT_FOUND)
        {
            result = A_ERROR;
            break;
        }

        /* Check if socket is blocked for a previous command */
        if (IS_SOCKET_BLOCKED(ath_sock_context[index]))
        {
            result = A_ERROR;
            break;
        }

        /* Send command and wait for response */
        cmd.ssl = (A_UINT32) ssl;
        SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_SSL_ACCEPT);
        if (wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_SSL_ACCEPT, (void *) (&cmd), sizeof(cmd)) != A_OK)
        {
            SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_SSL_ACCEPT);
            result = A_ERROR;
            break;
        }
        do
        {
            if (CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK)
            {
                A_ASSERT(0);
            }
        } while(SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_SSL_ACCEPT));
        result = ath_sock_context[index]->result;
    }
    while (0);

    return result;
}

/*****************************************************************************/
/* SSL_connect - Initiate SSL handshake.
 * SSL *ssl - SSL connection
 * Returns - 1 on success, ESSL_HSDONE if handshake has already been performed.
 *           Negative error code otherwise.
 *****************************************************************************/
A_INT32 Api_SSL_connect(A_VOID *pCxt,SSL *ssl)
{
    A_DRIVER_CONTEXT *pDCxt;
    SOCK_SSL_ACCEPT_T cmd;
    A_INT32 result = A_OK;
    A_INT32 index;

    pDCxt = GET_DRIVER_COMMON(pCxt);
    do
    {
        index = find_socket_context_from_ssl(ssl);
        if (index == SOCKET_NOT_FOUND)
        {
            result = A_ERROR;
            break;
        }

        /* Check if socket is blocked for a previous command */
        if (IS_SOCKET_BLOCKED(ath_sock_context[index]))
        {
            result = A_ERROR;
            break;
        }

        /* Send command and wait for response */
        cmd.ssl = (A_UINT32) ssl;
        SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_SSL_CONNECT);
        if (wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_SSL_CONNECT, (void *) (&cmd), sizeof(cmd)) != A_OK)
        {
            SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_SSL_CONNECT);
            result = A_ERROR;
            break;
        }
        do
        {
            if (CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK)
            {
                A_ASSERT(0);
            }
        }while(SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_SSL_CONNECT));
        result = ath_sock_context[index]->result;
    }
    while (0);

    return result;
}

/*****************************************************************************/
/* SSL_shutdown - Close SSL connection.
 *                The socket must be closed by other means.
 * SSL *ssl - SSL connection
 * Returns - 1 on success or negative error code on error (see SslErrors)
 *****************************************************************************/
A_INT32 Api_SSL_shutdown(A_VOID *pCxt,SSL *ssl)
{
    A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(pCxt);;
    SOCK_SSL_SHUTDOWN_T cmd;
    A_INT32 result = A_OK;
    A_INT32 index = find_socket_context_from_ssl(ssl);

    if (index == SOCKET_NOT_FOUND)
    {
        /* We send the shutdown command on the "global socket" if the socket used for
           the SSL is disconnected/closed before the SSL connection is shutdown. */
        index = GLOBAL_SOCK_INDEX;
    }
    do
    {

        /* Check if socket is blocked for a previous command */
        if (IS_SOCKET_BLOCKED(ath_sock_context[index]))
        {
            result = A_ERROR;
            break;
        }

        /* Send command and wait for response */
        cmd.ssl = (A_UINT32) ssl;
        SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_SSL_SHUTDOWN);
        if (wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_SSL_SHUTDOWN, (void *) (&cmd), sizeof(cmd)) != A_OK)
        {
            SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_SSL_SHUTDOWN);
            result = A_ERROR;
            break;
        }
        do
        {
            if (CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK)
            {
                A_ASSERT(0);
            }
        } while(SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_SSL_SHUTDOWN));
        result = ath_sock_context[index]->result;

        /* clear the SSL object ptr in the socket struct */
        ath_sock_context[index]->ssl = NULL;
    } while (0);

    return result;
}

A_INT32 Api_SSL_configure(A_VOID *pCxt, SSL *ssl, SSL_CONFIG *cfg)
{
    A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(pCxt);;
    SOCK_SSL_CONFIGURE_T cmd;
    A_INT32 result = A_OK;
    A_INT32 index = find_socket_context_from_ssl(ssl);

    if (index == SOCKET_NOT_FOUND)
    {
        /* We send the configure command on the "global socket" if the configure is
         * done before the socket is created */
        index = GLOBAL_SOCK_INDEX;
    }
    do
    {
        /* Check if socket is blocked for a previous command */
        if (IS_SOCKET_BLOCKED(ath_sock_context[index]))
        {
            result = A_ERROR;
            break;
        }

        /* Send command and wait for response */
        cmd.ssl = (A_UINT32) ssl;
        cmd.size = sizeof(SSL_CONFIG);
        memcpy(cmd.data, cfg, sizeof(SSL_CONFIG));
        SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_SSL_CONFIGURE);
        if (wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_SSL_CONFIGURE, (void *) (&cmd), sizeof(cmd)) != A_OK)
        {
            SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_SSL_CONFIGURE);
            result = A_ERROR;
            break;
        }
        do
        {
            if (CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK)
            {
                A_ASSERT(0);
            }
        } while(SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_SSL_CONFIGURE));
        result = ath_sock_context[index]->result;
    } while (0);

    return result;

}

/*****************************************************************************/
/* SSL_addCert - Add a certificate or Certificate Authority (CA) list to the
 *               SharkSsl object.
 *               A SharkSsl object in server mode is required to have at least
 *               one certificate.
 *               You can only set one CA list, thus the CA list must include
 *               all root certificates required for the session.
 * SSL *ssl - SSL connection
 * SslCert cert -address of array of binary data
 * A_UINT32 size - size of array
 * SSL_CERT_TYPE_T type - certificate or CA list
 * Returns - 1 on success or negative error code on error (see SslErrors)
 *****************************************************************************/
A_INT32 Api_SSL_addCert(A_VOID *pCxt, SSL_CTX *ctx, SSL_CERT_TYPE_T type, A_UINT8 *cert, A_UINT32 size)
{
    A_DRIVER_CONTEXT *pDCxt;
    SOCK_SSL_ADD_CERT_T *pCmd;
    A_INT32 result = A_ERROR;
    A_UINT32 n, buffSize, offset = 0;
    A_VOID *pBuff;
    int index = GLOBAL_SOCK_INDEX;
    A_UINT8 *data = (A_UINT8*)cert;

    pDCxt = GET_DRIVER_COMMON(pCxt);
    do
    {
        /* Check if socket is blocked for a previous command */
        if (IS_SOCKET_BLOCKED(ath_sock_context[index]))
        {
            break;
        }

        do
        {
            n = size - offset;
            if (n > MAX_CERT_DATA_LENGTH)
            {
                n = MAX_CERT_DATA_LENGTH;
            }
            buffSize = sizeof(SOCK_SSL_ADD_CERT_T) + n - 1;
            pBuff = A_NETBUF_ALLOC(buffSize);
            if (pBuff)
            {
                /* reserve the necessary space in the buffer */
                A_NETBUF_PUT(pBuff, buffSize);

                pCmd = A_NETBUF_DATA(pBuff);
                pCmd->wmi_cmd.cmd_type = SOCK_SSL_ADD_CERT;
                pCmd->wmi_cmd.length = buffSize - sizeof(WMI_SOCKET_CMD);
                pCmd->ctx = (A_UINT32) ctx;
                pCmd->type = type;
                pCmd->total = size;
                pCmd->offset = offset;
                pCmd->size = n;
                A_MEMCPY(pCmd->data, &data[offset], n);
                offset += n;

                /* send the buffer to the wifi device */
                SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_SSL_ADD_CERT);
                result = wmi_cmd_send(pDCxt->pWmiCxt, pBuff, WMI_SOCKET_CMDID, NO_SYNC_WMIFLAG);
                if (result != A_OK)
                {
                    SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_SSL_ADD_CERT);
                    return A_ERROR;
                }
                do
                {
                    if (CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK)
                    {
                        A_ASSERT(0);
                    }
                } while (SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_SSL_ADD_CERT));
                result = ath_sock_context[index]->result;
            }
        }
        while (offset < size && result >= A_OK);
        if (offset != size)
        {
            result = A_ERROR;
        }
    }
    while (0);

    return result;
}

A_INT32 Api_SSL_storeCert(A_VOID *pCxt, A_CHAR *name, A_UINT8 *cert, A_UINT32 size)
{
    A_DRIVER_CONTEXT *pDCxt;
    SOCK_SSL_STORE_CERT_T *pCmd;
    A_INT32 result = A_ERROR;
    A_UINT32 n, buffSize, offset = 0;
    A_VOID *pBuff;
    int index = GLOBAL_SOCK_INDEX;
    A_UINT8 *data = (A_UINT8*)cert;

    pDCxt = GET_DRIVER_COMMON(pCxt);
    do
    {
        /* Check if socket is blocked for a previous command */
        if (IS_SOCKET_BLOCKED(ath_sock_context[index]))
        {
            break;
        }

        do
        {
            n = size - offset;
            if (n > MAX_CERT_DATA_LENGTH)
            {
                n = MAX_CERT_DATA_LENGTH;
            }
            buffSize = sizeof(SOCK_SSL_STORE_CERT_T) + n - 1;
            pBuff = A_NETBUF_ALLOC(buffSize);
            if (pBuff)
            {
                /* reserve the necessary space in the buffer */
                A_NETBUF_PUT(pBuff, buffSize);

                pCmd = A_NETBUF_DATA(pBuff);
                pCmd->wmi_cmd.cmd_type = SOCK_SSL_STORE_CERT;
                pCmd->wmi_cmd.length = buffSize - sizeof(WMI_SOCKET_CMD);
                strncpy((char*)pCmd->name, name, SSL_MAX_CERT_NAME_LEN);
                pCmd->name[SSL_MAX_CERT_NAME_LEN] = '\0';
                pCmd->total = size;
                pCmd->offset = offset;
                pCmd->size = n;
                A_MEMCPY(pCmd->data, &data[offset], n);
                offset += n;

                /* send the buffer to the wifi device */
                SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_SSL_STORE_CERT);
                result = wmi_cmd_send(pDCxt->pWmiCxt, pBuff, WMI_SOCKET_CMDID, NO_SYNC_WMIFLAG);
                if (result != A_OK)
                {
                    SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_SSL_STORE_CERT);
                    return A_ERROR;
                }
                do
                {
                    if (CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK)
                    {
                        A_ASSERT(0);
                    }
                } while (SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_SSL_STORE_CERT));
                result = ath_sock_context[index]->result;
            }
        } while (offset < size && result >= A_OK);

        if (offset != size)
        {
            result = A_ERROR;
        }
    }
    while (0);

    return result;
}

A_INT32 Api_SSL_loadCert(A_VOID *pCxt, SSL_CTX *ctx, SSL_CERT_TYPE_T type, A_CHAR *name)
{
    A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(pCxt);
    SOCK_SSL_LOAD_CERT_T cmd;
    A_INT32 result = A_ERROR;
    A_INT32 index = GLOBAL_SOCK_INDEX;

    do
    {
        /*Check if socket is blocked for a previous command*/
        if (IS_SOCKET_BLOCKED(ath_sock_context[index]))
        {
            break;
        }

        /* Send command and wait for response */
        cmd.ctx = (A_UINT32)ctx;
        cmd.type = type;
        strncpy((char*)cmd.name, name, SSL_MAX_CERT_NAME_LEN);
        cmd.name[SSL_MAX_CERT_NAME_LEN] = '\0';
        SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_SSL_LOAD_CERT);
        if (wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_SSL_LOAD_CERT, (void *)(&cmd), sizeof(cmd)) != A_OK)
        {
            SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_SSL_LOAD_CERT);
            break;
        }
        do
        {
            if (CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK)
            {
                A_ASSERT(0);
            }
        }while(SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_SSL_LOAD_CERT));
        result = ath_sock_context[index]->result;
    }
    while (0);

    return result;
}

A_INT32 Api_SSL_listCert(A_VOID *pCxt, SSL_FILE_NAME_LIST *fileNames)
{
    A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(pCxt);
    SOCK_SSL_LIST_CERT_T cmd;
    A_INT32 result = A_ERROR;
    A_INT32 index = GLOBAL_SOCK_INDEX;

    do
    {
        /*Check if socket is blocked for a previous command*/
        if (IS_SOCKET_BLOCKED(ath_sock_context[index]))
        {
            break;
        }

        /* Send command and wait for response */
        SOCK_EV_MASK2_SET(ath_sock_context[index], SOCK_SSL_LIST_CERT);
        if (wmi_socket_cmd(pDCxt->pWmiCxt, SOCK_SSL_LIST_CERT, (void *)(&cmd), sizeof(cmd)) != A_OK)
        {
            SOCK_EV_MASK2_CLEAR(ath_sock_context[index], SOCK_SSL_LIST_CERT);
            break;
        }
        do
        {
            if (CUSTOM_BLOCK(pCxt, ath_sock_context[index], COMMAND_BLOCK_TIMEOUT, RX_DIRECTION) != A_OK)
            {
                A_ASSERT(0);
            }
        }while(SOCK_EV_MASK2_TEST(ath_sock_context[index], SOCK_SSL_LIST_CERT));
        result = ath_sock_context[index]->result;
        memcpy(fileNames, ath_sock_context[index]->data, sizeof(SSL_FILE_NAME_LIST));
    }
    while (0);

    return result;
}


#endif // ENABLE_SSL

#endif //ENABLE_STACK_OFFLOAD
