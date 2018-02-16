#include <a_config.h>
#include <a_types.h>
#include <a_osapi.h>
#include <driver_cxt.h>
#include <common_api.h>
#include <custom_api.h>
#include <wmi_api.h>
#include <htc.h>
#include "mqx.h"
#include "bsp.h"
#include "enet.h"
#include "enetprv.h"
#include "atheros_wifi.h"
#include "enet_wifi.h"
#include "atheros_wifi_api.h"
#include "atheros_wifi_internal.h"
#include "cust_netbuf.h"

#if ENABLE_STACK_OFFLOAD
#include "atheros_stack_offload.h"
#include "common_stack_offload.h"
#include "custom_stack_offload.h"




#if ZERO_COPY
A_NETBUF_QUEUE_T zero_copy_free_queue;
#endif

#if ENABLE_SSL
extern A_VOID *sslhandle;
A_INT32 find_socket_context_from_ssl(SSL *ssl);
#endif

A_UINT32 custom_queue_empty(A_UINT32 index);


/*****************************************************************************/
/*  custom_socket_context_init - Initializes the custom section of socket context.
 * RETURNS: A_OK on success or error otherwise. 
 *****************************************************************************/
A_STATUS custom_socket_context_init()
{
    SOCKET_CUSTOM_CONTEXT_PTR pcustctxt = NULL;
    A_UINT32 index;
    
    for (index = 0; index < MAX_SOCKETS_SUPPORTED + 1; index++)	
    {	
      if((pcustctxt = A_MALLOC(sizeof(SOCKET_CUSTOM_CONTEXT),MALLOC_ID_CONTEXT)) == NULL){
                return A_NO_MEMORY;
      }
        _lwevent_create(&pcustctxt->sockWakeEvent, 0/* no auto clear */); 		
        A_NETBUF_QUEUE_INIT(&(pcustctxt->rxqueue));
        pcustctxt->blockFlag = 0;
        pcustctxt->respAvailable = A_FALSE;
        ath_sock_context[index]->sock_custom_context = pcustctxt;
        ath_sock_context[index]->remaining_bytes = 0;
        ath_sock_context[index]->old_netbuf = NULL;
        
 #if NON_BLOCKING_TX
         A_NETBUF_QUEUE_INIT(&(pcustctxt->non_block_queue));
         A_MUTEX_INIT(&(pcustctxt->nb_tx_mutex));
#endif	
    }
                
    #if ZERO_COPY	
    /*Initilize common queue used to store Rx packets for zero copy option*/
    A_NETBUF_QUEUE_INIT(&zero_copy_free_queue);
    #endif
    
    
    return A_OK;
}

/*****************************************************************************/
/*  custom_socket_context_deinit - De-initializes the custom section of socket context.
 * RETURNS: A_OK on success or error otherwise. 
 *****************************************************************************/
A_STATUS custom_socket_context_deinit()
{
	SOCKET_CUSTOM_CONTEXT_PTR pcustctxt = NULL;
	A_UINT32 index;
	
	for (index = 0; index < MAX_SOCKETS_SUPPORTED + 1; index++)	
	{	
           pcustctxt = GET_CUSTOM_SOCKET_CONTEXT(ath_sock_context[index]); 
#if NON_BLOCKING_TX            
            A_MUTEX_DELETE(&(pcustctxt->nb_tx_mutex));
#endif	          		
	    _lwevent_destroy(&pcustctxt->sockWakeEvent); 		
             A_FREE(pcustctxt,MALLOC_ID_CONTEXT);
	}			

	return A_OK;
}



A_STATUS blockForDataRx(A_VOID* pCxt, A_VOID* ctxt, A_UINT32 msec, A_UINT8 direction)
{
    SOCKET_CUSTOM_CONTEXT_PTR pcustctxt = GET_CUSTOM_SOCKET_CONTEXT(ctxt);
    pcustctxt->respAvailable = A_FALSE;
    return (blockForResponse(pCxt, ctxt, msec, direction));
}
/*****************************************************************************/
/*  blockForResponse - blocks a thread, either indefinitely or for a specified
 *                     time period in milliseconds
 * RETURNS: A_OK on success or error if timeout occurs. 
 *****************************************************************************/
A_STATUS blockForResponse(A_VOID* pCxt, A_VOID* ctxt, A_UINT32 msec, A_UINT8 direction)
{
    A_UINT32 ret = (msec)*BSP_ALARM_FREQUENCY/1024;  //1024 is an approximation to convert division to right shift 
    A_STATUS result = A_OK;
    SOCKET_CUSTOM_CONTEXT_PTR pcustctxt = GET_CUSTOM_SOCKET_CONTEXT(ctxt);
    A_UINT32 priority, old_priority;
    
    if (msec && !ret) {
        ++ret;
    } 
    
    pcustctxt->blockFlag = 1;	
    
    //_task_get_priority(_task_get_id(), &old_priority);
    
    /* Priority  '1' higher than the driver task to prevent race condition */
    priority = 5;
    _task_set_priority(_task_get_id(), priority, &old_priority);
    
    if(direction == RX_DIRECTION)
    {
        if(pcustctxt->respAvailable == A_FALSE)
        {
            pcustctxt->rxBlock = A_TRUE;    
            if(MQX_OK != _lwevent_wait_ticks(&pcustctxt->sockWakeEvent, 0x01, FALSE, ret)){  
                result = A_ERROR;	                                               
            }else{                                                      
                _lwevent_clear(&pcustctxt->sockWakeEvent, 0x01);                            
            }     
            pcustctxt->rxBlock = A_FALSE;       
        }
        
        if(pcustctxt->respAvailable == A_TRUE)
        {
            /*Response is available, reset the flag for future use*/
            result = A_OK;
            pcustctxt->respAvailable = A_FALSE;
        }
    }
    else if(direction == TX_DIRECTION)
    {
        if(pcustctxt->txUnblocked == A_FALSE)
        {
            pcustctxt->txBlock = A_TRUE;    
            if(MQX_OK != _lwevent_wait_ticks(&pcustctxt->sockWakeEvent, 0x01, FALSE, ret)){  
                result = A_ERROR;	                                               
            }else{                                                      
                _lwevent_clear(&pcustctxt->sockWakeEvent, 0x01);                            
            }     
            pcustctxt->txBlock = A_FALSE;       
        }
        
        if(pcustctxt->txUnblocked == A_TRUE)
        {
            /*Response is available, reset the flag for future use*/
            result = A_OK;
            pcustctxt->txUnblocked = A_FALSE;
        }
    }
    else
    {
        last_driver_error = A_EINVAL;
    }
    _task_set_priority(_task_get_id(), old_priority, &priority);
    
    pcustctxt->blockFlag = 0;
    return result;
}


#if T_SELECT_VER1

A_STATUS blockSelect(A_VOID* pCxt, A_UINT32 msec)
{
    A_UINT32 ret = (msec)*BSP_ALARM_FREQUENCY/1024;  //1024 is an approximation to convert division to right shift 
    A_STATUS result = A_OK;
    
    if (msec && !ret) {
        ++ret;
    } 
    
    if(MQX_OK != _lwevent_wait_ticks(&((GET_DRIVER_CXT(pCxt))->sockSelectWakeEvent), 0x01, TRUE, ret)){  
      result = A_ERROR;	                                               
    }else{                                                      
      _lwevent_clear(&((GET_DRIVER_CXT(pCxt))->sockSelectWakeEvent), 0x01);                            
    }     
    return result;
}
#endif //T_SELECT_VER1


/*****************************************************************************/
/*  isSocketBlocked - Checks if a thread is blocked on a given socket                
 * RETURNS: value of block flag 
 *****************************************************************************/
A_UINT32 isSocketBlocked(A_VOID* ctxt)
{
        SOCKET_CUSTOM_CONTEXT_PTR pcustctxt = GET_CUSTOM_SOCKET_CONTEXT(ctxt);
        return pcustctxt->blockFlag;
}

/*****************************************************************************/
/*  unblock - Unblocks a thread if it is blocked.                
 * RETURNS: A_OK if unblock was successful, A_ERROR if thread was not blocked 
 *****************************************************************************/
A_STATUS unblock(A_VOID* ctxt, A_UINT8 direction)
{
	A_STATUS result = A_OK;
	SOCKET_CUSTOM_CONTEXT_PTR pcustctxt = GET_CUSTOM_SOCKET_CONTEXT(ctxt);

	/*Unblock task if it is blocked*/
	 if(direction == RX_DIRECTION)
        {
            pcustctxt->respAvailable = A_TRUE;
            /*Unblock task if it is blocked*/
            if(pcustctxt->rxBlock == A_TRUE) 
            {                    
               _lwevent_set(&pcustctxt->sockWakeEvent, 0x01);
            }
        }
        else if(direction == TX_DIRECTION)
        {
            pcustctxt->txUnblocked = A_TRUE;
            if(pcustctxt->txBlock == A_TRUE) 
            {                    
               _lwevent_set(&pcustctxt->sockWakeEvent, 0x01);
            }
        }
	return result;	
}


#if T_SELECT_VER1
A_STATUS unblockSelect(A_VOID* pCxt)
{
	A_STATUS result = A_OK;
        _lwevent_set(&((GET_DRIVER_CXT(pCxt))->sockSelectWakeEvent), 0x01);
	return result;	
}
#endif


/*****************************************************************************/
/*  t_socket - Custom version of socket API- please see Api_socket for details                
 * RETURNS: socket handle or A_ERROR 
 *****************************************************************************/
A_INT32 t_socket(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 domain, A_UINT32 type, A_UINT32 protocol)
{
	A_VOID *pCxt;
	
	if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
	{
	   	return A_ERROR;
	}  
	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
	 
	return (Api_socket(pCxt, domain, type, protocol));
}


/*****************************************************************************/
/*  t_shutdown - Custom version of socket shutdown API- please see Api_shutdown for details                
 * RETURNS: A_OK or A_ERROR 
 *****************************************************************************/
A_INT32 t_shutdown(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle)
{
	A_VOID *pCxt;
	
	if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
	{
	   	return A_ERROR;
	}
	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
	   
	return (Api_shutdown(pCxt, handle));
}


/*****************************************************************************/
/*  t_connect - Custom version of socket connect API- please see Api_connect for details                
 * RETURNS: A_OK or A_ERROR 
 *****************************************************************************/
A_INT32 t_connect(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_VOID* name, A_UINT16 length)
{
	A_VOID *pCxt;
	
	if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
	{
	   	return A_ERROR;
	} 
	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
	  
	return (Api_connect(pCxt, handle, name, length));
}


/*****************************************************************************/
/*  t_bind - Custom version of socket bind API- please see Api_bind for details                
 * RETURNS: A_OK or A_ERROR 
 *****************************************************************************/
A_INT32 t_bind(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_VOID* name, A_UINT16 length)
{
	A_VOID *pCxt;
	
	if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
	{
	   	return A_ERROR;
	}  
	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
	 
	return (Api_bind(pCxt, handle, name, length));
}

/*****************************************************************************/
/*  t_listen - Custom version of socket listen API- please see Api_listen for details                
 * RETURNS: A_OK or A_ERROR 
 *****************************************************************************/
A_INT32 t_listen(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_UINT32 backlog)
{
	A_VOID *pCxt;
	
	if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
	{
	   	return A_ERROR;
	}  
	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
	 
	return (Api_listen(pCxt, handle, backlog));
}


/*****************************************************************************/
/*  t_accept - Custom version of socket accept API- please see Api_accept for details                
 * RETURNS: A_OK or A_ERROR 
 *****************************************************************************/
A_INT32 t_accept(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_VOID* name, A_UINT16 length)
{
	A_VOID *pCxt;
	
	if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
	{
	   	return A_ERROR;
	}   
	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
	
    return (Api_accept(pCxt, handle, name, length));
}

#if T_SELECT_VER1
A_INT32 t_accept_nb(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_VOID* name, A_UINT16 length)
{
	A_VOID *pCxt;
	
	if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
	{
	   	return A_ERROR;
	}   
	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
	
	return (Api_accept_ver1(pCxt, handle, name, length));
}
#endif


/*****************************************************************************/
/*  t_select - Custom version of socket select API- please see Api_select for details                
 * RETURNS: A_OK or A_ERROR 
 *****************************************************************************/
A_INT32 t_select(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_UINT32 tv)
{
    A_VOID *pCxt;

    if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
    {
   	    return A_ERROR;
    }   
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_RX)){
        return A_ERROR;
    }

    return (Api_select(pCxt, handle, tv));
}




#if T_SELECT_VER1

#define __FDMASK(d)             (((A_UINT32) 1) << (d))
#define __FD_SET(d, set)        ((*set) |= __FDMASK(d))
#define __FD_CLR(d, set)        ((*set) &= ~__FDMASK(d))
#define __FD_ISSET(d, set)      (((set) & __FDMASK(d)) != 0)
#define __FD_ZERO(set)          ((*set) = 0)


A_INT32 FD_IsSet(A_UINT32 handle, A_UINT32 mask)
{
  A_INT32 index = 0;
  if((index = find_socket_context(handle, TRUE)) == SOCKET_NOT_FOUND){
    return 1;
  }          
  return(__FD_ISSET(index, mask));
}

A_INT32 FD_Set(A_UINT32 handle, A_UINT32* mask)
{
  A_INT32 index = 0;
  if((index = find_socket_context(handle, TRUE)) == SOCKET_NOT_FOUND){
    return 0;
  }          
  __FD_SET(index, (mask));
  return(A_OK);
}


A_INT32 FD_Clr(A_UINT32 handle, A_UINT32* mask)
{
  A_INT32 index = 0;
  if((index = find_socket_context(handle, TRUE)) == SOCKET_NOT_FOUND){
  	//special case, we are trying to clear the mask for a non-existent socket. 
  	//This is a workaround now. Need to come up with an elegant solution. 
  	//Go through the all the existing sockets and remove the mask elements for which no valid sockets exists. 

	*mask &= ((1<<(MAX_SOCKETS_SUPPORTED+1)) -1);
  	for(index=0; index<MAX_SOCKETS_SUPPORTED ; index++)
	{
	    if( (*mask & (1<<index) ) && (ath_sock_context[index]->handle == 0))
	    {
	         __FD_CLR(index, (mask));
	    }
  	}
	return A_OK;
  } 
  __FD_CLR(index, (mask));
  return(A_OK);
}

A_INT32 FD_Zero(A_UINT32* mask)
{
  *mask =0;
  return(A_OK);
}




A_INT32 t_select_ver1(void* handle, A_INT32 num, A_UINT32 *r_fd, A_UINT32 *w_fd, A_UINT32 *e_fd, A_UINT32 tv)
{
    A_VOID *pCxt ;
    if((pCxt = ((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR) == NULL)
	return A_ERROR;

    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_RX)){
        return A_ERROR;
    }

    return (Api_select_ver1(pCxt, num, r_fd, w_fd, e_fd, tv));
}

#endif //T_SELECT_VER1



/*****************************************************************************/
/*  t_errno - Custom version of socket errno API- please see Api_errno for details                
 * RETURNS: A_OK or A_ERROR 
 *****************************************************************************/
A_INT32 t_errno(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle)
{
    A_VOID *pCxt;

    if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
    {
   	    return A_ERROR;
    }   
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }

    return (Api_errno(pCxt, handle));
}


/*****************************************************************************/
/*  t_setsockopt - Custom version of socket setsockopt API- please see Api_setsockopt for details                
 * RETURNS: A_OK or A_ERROR 
 *****************************************************************************/
A_INT32 t_setsockopt(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_UINT32 level, A_UINT32 optname, A_UINT8* optval, A_UINT32 optlen)
{
    A_VOID *pCxt;

    if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
    {
   	    return A_ERROR;
    }  
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
     
    return (Api_setsockopt(pCxt, handle, level, optname, optval, optlen));
}

/*****************************************************************************/
/*  t_getsockopt - Custom version of socket getsockopt API- please see                 
 *                 Api_getsockopt for details. 
 * RETURNS: A_OK or A_ERROR 
 *****************************************************************************/
A_INT32 t_getsockopt(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_UINT32 level, A_UINT32 optname, A_UINT8* optval, A_UINT32 optlen)
{
    A_VOID *pCxt;

    if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
    {
   	    return A_ERROR;
    } 
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
      
    return (Api_getsockopt(pCxt, handle, level, optname, optval, optlen));
}


/*****************************************************************************/
/*  t_ipconfig - Custom version of ipconfig API- please see                 
 *                 Api_ipconfig for details. 
 * RETURNS: A_OK or A_ERROR 
 *****************************************************************************/
//A_INT32 t_ipconfig(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 mode,A_UINT32* ipv4_addr, A_UINT32* subnetMask, A_UINT32* gateway4)
A_INT32 t_ipconfig(void* handle, A_UINT32 mode, A_UINT32* ipv4_addr, A_UINT32* subnetMask, A_UINT32* gateway4,IP46ADDR *dnsaddr,A_CHAR *hostname)
{
    A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;
	
	if(pCxt == NULL)
	{
	   	return A_ERROR;
	} 

    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
      
    return (Api_ipconfig(pCxt, mode, ipv4_addr, subnetMask, gateway4,dnsaddr,hostname));
}

/*****************************************************************************/
/*  t_ip6config - Custom version of ipconfig API- please see                 
 *                 Api_ip6config for details. 
 * RETURNS: A_OK or A_ERROR 
 *****************************************************************************/
A_INT32 t_ip6config(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 mode,IP6_ADDR_T *v6Global,IP6_ADDR_T *v6Local,IP6_ADDR_T *v6DefGw,IP6_ADDR_T *v6GlobalExtd,A_INT32 *LinkPrefix,
		    A_INT32 *GlbPrefix, A_INT32 *DefGwPrefix, A_INT32 *GlbPrefixExtd)
{
    A_VOID *pCxt;

    if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
    {
   	    return A_ERROR;
    }
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
       
    return (Api_ip6config(pCxt, mode,v6Global,v6Local,v6DefGw,v6GlobalExtd,LinkPrefix,GlbPrefix,DefGwPrefix,GlbPrefixExtd));
}

/*****************************************************************************/
/*  t_ipconfig_dhcp_pool -  Function to create DHCP Pool                
 * RETURNS: A_OK or A_ERROR 
 *****************************************************************************/
A_INT32 t_ipconfig_dhcp_pool(ENET_CONTEXT_STRUCT_PTR enet_ptr,A_UINT32* start_ipv4_addr, A_UINT32* end_ipv4_addr,A_INT32 leasetime)
{
    A_VOID *pCxt;

    if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
    {
   	    return A_ERROR;
    } 
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
      
    return (Api_ipconfig_dhcp_pool(pCxt,start_ipv4_addr, end_ipv4_addr,leasetime));
}


A_INT32 t_ip6config_router_prefix(ENET_CONTEXT_STRUCT_PTR enet_ptr,A_UINT8 *v6addr,A_INT32 prefixlen, A_INT32 prefix_lifetime,
		                     A_INT32 valid_lifetime)
{

    A_VOID *pCxt;

    if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
    {
   	    return A_ERROR;
    } 
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ip6config_router_prefix(pCxt,v6addr,prefixlen,prefix_lifetime,valid_lifetime));
}	


A_INT32 custom_ipconfig_set_tcp_exponential_backoff_retry(ENET_CONTEXT_STRUCT_PTR enet_ptr,A_INT32 retry)
{
    A_VOID *pCxt;

    if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
    {
   	    return A_ERROR;
    } 
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ipconfig_set_tcp_exponential_backoff_retry(pCxt,retry));
}

A_INT32 custom_ipconfig_set_ip6_status(ENET_CONTEXT_STRUCT_PTR enet_ptr,A_UINT16 status)
{
    A_VOID *pCxt;

    if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
    {
   	    return A_ERROR;
    } 
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ipconfig_set_ip6_status(pCxt,status));
}	

A_INT32 custom_ipconfig_dhcp_release(ENET_CONTEXT_STRUCT_PTR enet_ptr)
{

    A_VOID *pCxt;

    if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
    {
   	    return A_ERROR;
    } 
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ipconfig_dhcp_release(pCxt));

}

A_INT32 custom_ipconfig_set_tcp_rx_buffer(ENET_CONTEXT_STRUCT_PTR enet_ptr,A_INT32 rxbuf)
{
    A_VOID *pCxt;
	
    if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
    {
   	    return A_ERROR;
    } 
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ipconfig_set_tcp_rx_buffer(pCxt,rxbuf));
}

#if ENABLE_HTTP_SERVER
A_INT32 custom_ip_http_server(void *handle, A_INT32 command)
{
	#if 0 /* keviny */
    A_VOID *pCxt = handle;
	#else
    A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;
	#endif
	
	if(pCxt == NULL)
	{
	   	return A_ERROR;
	} 
	
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ip_http_server(pCxt, command));
}

A_INT32 custom_ip_http_server_method(void *handle, A_INT32 command, A_UINT8 *pagename, A_UINT8 *objname, A_INT32 objtype, A_INT32 objlen, A_UINT8 * value)
{
#if 0 /* keviny */
		A_VOID *pCxt = handle;
#else
		A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;
#endif

	if(pCxt == NULL)
	{
	   	return A_ERROR;
	} 
	
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ip_http_server_method(pCxt, command, pagename, objname, objtype, objlen, value));
}

#endif

#if ENABLE_HTTP_CLIENT

/*****************************************************************************/
/*  cust_httpc_method - Custom version of httpc API- please see Api_httpc_connect for details                
* RETURNS: socket handle or A_ERROR 
*****************************************************************************/
A_INT32 custom_httpc_method(void* handle, A_UINT32 command, A_UINT8* url, A_UINT8 *data, A_UINT8 **output) // mari addr should be v6 compatable
{
#if 0 /* keviny */
		A_VOID *pCxt = handle;
#else
		A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;
#endif
	
	if(pCxt== NULL)
	{
	   	return A_ERROR;
	}  
	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    
	return (Api_httpc_method(pCxt, command, url, data, output));
}

/*****************************************************************************/
/*  zero_copy_http_free - ZERO_COPY free API.It will find and remove rx buffer 
 *                  for HTTP client.   
 * RETURNS: Nothing
 *****************************************************************************/
A_VOID zero_copy_http_free(A_VOID* buffer)
{
	A_NETBUF *a_netbuf_ptr = NULL;
    A_UINT32 *length;
    
    length = (A_UINT32 *) buffer;
    buffer = (A_VOID *) (length - 3);
    //printf("Deleting from Q %p\n", buffer);
    zero_copy_free(buffer);
}
#endif /* ENABLE_HTTP_CLIENT */

A_INT32 custom_ip_hostname(void *handle,char *domain_name)
{
		A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;

   if(pCxt == NULL)
   {
  	return A_ERROR;
   } 
   /*Wait for chip to be up*/
   if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ip_hostname(pCxt, domain_name));
}

#if ENABLE_DNS_SERVER
A_INT32 custom_ip_dns_local_domain(void *handle,char *domain_name)
{

    A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;
   if(pCxt == NULL)
   {
  	return A_ERROR;
   } 
   /*Wait for chip to be up*/
   if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ip_dns_local_domain(pCxt, domain_name));
}

A_INT32 custom_ipdns(void *handle,A_INT32 command, char *domain_name, IP46ADDR *dnsaddr)
{

   A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;

   if(pCxt == NULL)
   {
  	return A_ERROR;
   } 
   /*Wait for chip to be up*/
   if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ipdns(pCxt,command,domain_name,dnsaddr));
}
#endif /* ENABLE_DNS_SERVER */

#if ENABLE_DNS_CLIENT
A_INT32 custom_ip_resolve_hostname(void *handle, DNC_CFG_CMD *DncCfg,DNC_RESP_INFO *DncRespInfo)
{

		A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;

	if(pCxt == NULL)
	{
	   	return A_ERROR;
	} 
	
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ip_resolve_host_name(pCxt,DncCfg,DncRespInfo));
}	

A_INT32 custom_ip_dns_server_addr(void *handle, IP46ADDR *addr)
{

    A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;
   if(pCxt == NULL)
   {
  	return A_ERROR;
   } 
   /*Wait for chip to be up*/
   if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ip_dns_server_addr(pCxt, addr));
}

A_INT32 custom_ip_dns_client(void *handle, A_INT32 command)
{
    A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;

	if(pCxt == NULL)
	{
	   	return A_ERROR;
	} 
	
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ip_dns_client(pCxt, command));
}
#endif /* ENABLE_DNS_CLIENT */

#if ENABLE_SNTP_CLIENT
A_INT32 custom_ip_sntp_srvr_addr(void *handle,A_INT32 command,char * sntp_srvr_addr)
{
   A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;

   if(pCxt == NULL)
   {
  	return A_ERROR;
   } 
   /*Wait for chip to be up*/
   if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ip_sntp_srvr_addr(pCxt,command,sntp_srvr_addr));
}

A_INT32 custom_ip_sntp_get_time(void *handle,tSntpTime *SntpTime)
{
   A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;

   if(pCxt == NULL)
   {
  	return A_ERROR;
   } 
   /*Wait for chip to be up*/
   if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
   return (Api_ip_sntp_get_time(pCxt,SntpTime));
}
A_INT32 custom_ip_sntp_get_time_of_day(void *handle,tSntpTM *SntpTm)
{
   A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;

   if(pCxt == NULL)
   {
  	return A_ERROR;
   } 
   /*Wait for chip to be up*/
   if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
   return (Api_ip_sntp_get_time_of_day(pCxt,SntpTm));
}	

A_INT32 custom_ip_sntp_modify_zone_dse(void *handle, A_UINT8 hour,A_UINT8 min,A_UINT8 add_sub,A_UINT8 dse)
{
    A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;

   if(pCxt == NULL)
   {
  	return A_ERROR;
   } 
   /*Wait for chip to be up*/
   if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ip_sntp_modify_zone_dse(pCxt,hour,min,add_sub,dse));
}

A_INT32 custom_ip_sntp_query_srvr_address(void *handle,tSntpDnsAddr SntpDnsAddr[MAX_SNTP_SERVERS])
{
   A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;

   if(pCxt == NULL)
   {
  	return A_ERROR;
   } 
   /*Wait for chip to be up*/
   if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ip_sntp_query_srvr_address(pCxt,SntpDnsAddr));

}

A_INT32 custom_ip_sntp_client(void *handle, A_INT32 command)
{
    A_VOID *pCxt=((ENET_CONTEXT_STRUCT_PTR)handle)->MAC_CONTEXT_PTR;

	if(pCxt == NULL)
	{
	   	return A_ERROR;
	} 
	
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
    return (Api_ip_sntp_client(pCxt, command));
}
#endif /* ENABLE_SNTP_CLIENT */

/*****************************************************************************/
/*  t_ping - Custom version of ping API- please see                 
 *                 Api_ping for details. 
 * RETURNS: A_OK or A_ERROR 
 *****************************************************************************/
A_INT32 t_ping(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 ipv4_addr, A_UINT32 size)
{
    A_VOID *pCxt;

    if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
    {
   	    return A_ERROR;
    } 
    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
      
    return (Api_ping(pCxt, ipv4_addr, size));
}

/*****************************************************************************/
/*  t_ping6 - Custom version of ping API- please see                 
 *                 Api_ping6 for details. 
 * RETURNS: A_OK or A_ERROR 
 *****************************************************************************/
A_INT32 t_ping6(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT8 *ip6addr, A_UINT32 size)
{
	A_VOID *pCxt;
	
	if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
	{
	   	return A_ERROR;
	} 
	
	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)){
        return A_ERROR;
    }
	
	return (Api_ping6(pCxt, ip6addr, size));
}

/*****************************************************************************/
/*  t_send - Custom version of socket send API. Used for stream based sockets.                 
 * 			 Sends provided buffer to target.
 * RETURNS: Number of bytes sent to target 
 *****************************************************************************/
A_INT32 t_send(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_UINT8* buffer, A_UINT32 length, A_UINT32 flags)
{
	return (t_sendto(enet_ptr, handle, buffer, length, flags, NULL, 0));
}



/*****************************************************************************/
/*  t_sendto - Custom version of socket send API. Used for datagram sockets.                 
 * 		Sends provided buffer to target. Creates space for headroom at
 *              the begining of the packet. The headroom is used by the stack to 
 *              fill in the TCP-IP header.   
 * RETURNS: Number of bytes sent to target 
 *****************************************************************************/
A_INT32 t_sendto(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_UINT8* buffer, A_UINT32 length, A_UINT32 flags, A_VOID* name, A_UINT32 socklength)
{
	A_UINT8 custom_hdr[TCP6_HEADROOM];
	A_INT32 index = 0;
	A_UINT16 custom_hdr_length = 0;
	DRV_BUFFER_PTR             db_ptr;
	A_UINT16 hdr_length = sizeof(ATH_MAC_HDR);
	A_UINT8* hdr = NULL, *bufPtr=buffer;
	A_VOID *pCxt;
#if NON_BLOCKING_TX	
	SOCKET_CUSTOM_CONTEXT_PTR pcustctxt;
#endif	
	A_INT32 result=0;
	A_STATUS status = A_OK;
        A_UINT16  rem_len = length; 
        A_UINT16  offset = 0;
	
	if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
	{
	   	return A_ERROR;
	}   
	
	/*Find socket context*/
	if((index = find_socket_context(handle,TRUE)) == SOCKET_NOT_FOUND)
	{
		last_driver_error = A_SOCKCXT_NOT_FOUND;
		return A_SOCK_INVALID;
	}

     while (rem_len != 0)
     {    
        length = rem_len; 
	A_MEMZERO(custom_hdr, TCP6_HEADROOM);

	if(ath_sock_context[index]->domain == ATH_AF_INET)
	{
		if(ath_sock_context[index]->type == SOCK_STREAM_TYPE)
			hdr_length = TCP_HEADROOM;
		else
			hdr_length = UDP_HEADROOM;
	}
	else
	{

		if(ath_sock_context[index]->type == SOCK_STREAM_TYPE)
			hdr_length = TCP6_HEADROOM_WITH_NO_OPTION;
		else
			hdr_length = UDP6_HEADROOM;
	}

	
	/*Calculate fragmentation threshold. We cannot send packets greater that HTC credit size
	  over HTC, Bigger packets need to be fragmented. Thresholds are different for IP v4 vs v6*/
	if(ath_sock_context[index]->domain == ATH_AF_INET6)
	{

		custom_hdr_length = sizeof(SOCK_SEND6_T);
		if(name != NULL) {
			((SOCK_SEND6_T*)custom_hdr)->name6.sin6_port = A_CPU2LE16(((SOCKADDR_6_T*)name)->sin6_port);
			((SOCK_SEND6_T*)custom_hdr)->name6.sin6_family = A_CPU2LE16(((SOCKADDR_6_T*)name)->sin6_family);
			((SOCK_SEND6_T*)custom_hdr)->name6.sin6_flowinfo = A_CPU2LE32(((SOCKADDR_6_T*)name)->sin6_flowinfo);	
			A_MEMCPY((A_UINT8*)&(((SOCK_SEND6_T*)custom_hdr)->name6.sin6_addr),(A_UINT8*)&((SOCKADDR_6_T*)name)->sin6_addr,sizeof(IP6_ADDR_T));	
			((SOCK_SEND6_T*)custom_hdr)->socklength = A_CPU2LE32(socklength);
		}
		else
		{
			memset((A_UINT8*)(&((SOCK_SEND6_T*)custom_hdr)->name6),0,sizeof(name));
		}
	}
	else
	{
		custom_hdr_length = sizeof(SOCK_SEND_T);
		if(name != NULL) {
			((SOCK_SEND_T*)custom_hdr)->name.sin_port = A_CPU2LE16(((SOCKADDR_T*)name)->sin_port);
			((SOCK_SEND_T*)custom_hdr)->name.sin_family = A_CPU2LE16(((SOCKADDR_T*)name)->sin_family);
			((SOCK_SEND_T*)custom_hdr)->name.sin_addr = A_CPU2LE32(((SOCKADDR_T*)name)->sin_addr);	
			((SOCK_SEND_T*)custom_hdr)->socklength = A_CPU2LE32(socklength);
		}
		else
		{			
			memset((A_UINT8*)(&((SOCK_SEND_T*)custom_hdr)->name),0,sizeof(name));
		}		
	}
	
	/*Populate common fields of custom header, these are the same for IP v4/v6*/
	((SOCK_SEND_T*)custom_hdr)->handle = A_CPU2LE32(handle);
	((SOCK_SEND_T*)custom_hdr)->flags = A_CPU2LE32(flags);
	
         if(offset != 0) 
	 {
	    memmove(bufPtr,(bufPtr + offset),rem_len);
	 } 	 


        db_ptr = &((TX_PACKET_PTR)(bufPtr-TX_PKT_OVERHEAD))->db;
        hdr = ((TX_PACKET_PTR)(bufPtr - TX_PKT_OVERHEAD))->hdr;
  
        A_MEMZERO(hdr, hdr_length);		

#if NON_BLOCKING_TX
        pcustctxt = GET_CUSTOM_SOCKET_CONTEXT(ath_sock_context[index]); 
	/*In non blocking TX case, Enqueue the packet so that it can be freed later when 
          TX over SPI is successful. Application should not free this buffer*/

        A_MUTEX_LOCK(&(pcustctxt->nb_tx_mutex));
	A_NETBUF_ENQUEUE(&(pcustctxt->non_block_queue), bufPtr);   
        A_MUTEX_UNLOCK(&(pcustctxt->nb_tx_mutex));        
#endif	

	
		
	if((length == 0) || (length > IPV4_FRAGMENTATION_THRESHOLD)){
          /*Host fragmentation is not allowed, application must send payload
           below IPV4_FRAGMENTATION_THRESHOLD size*/
          A_ASSERT(0); 
        }

        ((SOCK_SEND_T*)custom_hdr)->length = A_CPU2LE32(length);
        
        A_MEMCPY(hdr,custom_hdr, custom_hdr_length);	

        db_ptr->context = (pointer)ath_sock_context[index];
        if(length > TCP_MSS)
        {
           offset = TCP_MSS;
           length = TCP_MSS;
        } 
        
	rem_len -= length; 
        
	db_ptr->bufFragment[0].payload = bufPtr;
        db_ptr->bufFragment[0].length = length;
        result += length;
        status = custom_send_tcpip(pCxt, db_ptr,length,1,hdr,hdr_length);
        if(status != A_OK)
        {
          goto END_TX;
        }

#if !NON_BLOCKING_TX	
        /*Wait till packet is sent to target*/		
        if(CUSTOM_BLOCK(pCxt,ath_sock_context[index], TRANSMIT_BLOCK_TIMEOUT, TX_DIRECTION) != A_OK)
        {
           result = A_ERROR;
           goto END_TX;	
        }                
#endif		
     }
END_TX: 

    if(status != A_OK)
    {   
        result = A_ERROR;
    }

    custom_init_netbuf_ptrs(db_ptr);
	
	return result;
}





#if ZERO_COPY
A_INT32 t_recvfrom(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, void** buffer, A_UINT32 length, A_UINT32 flags, A_VOID* name, A_UINT32* socklength)
#else
A_INT32 t_recvfrom(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, void* buffer, A_UINT32 length, A_UINT32 flags, A_VOID* name, A_UINT32* socklength)
#endif
{
	A_VOID *pCxt;
	
	if((pCxt = enet_ptr->MAC_CONTEXT_PTR) == NULL)
	{
	   	return A_ERROR;
	}   
	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_RX)){
        return A_ERROR;
    }

	return (Api_recvfrom(pCxt, handle, buffer, length, flags, name, socklength));
}



#if ZERO_COPY
A_INT32 t_recv(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, void** buffer, A_UINT32 length, A_UINT32 flags)
#else
A_INT32 t_recv(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, void* buffer, A_UINT32 length, A_UINT32 flags)
#endif
{
	return (t_recvfrom(enet_ptr, handle, buffer, length, flags, NULL, NULL));
}










#if ZERO_COPY

/*****************************************************************************/
/*  zero_copy_free - ZERO_COPY free API.It will find and remove rx buffer from
 *                  a common queue.   
 * RETURNS: Number of bytes received or A_ERROR in case of failure
 *****************************************************************************/
A_VOID zero_copy_free(A_VOID* buffer)
{
	A_NETBUF* a_netbuf_ptr = NULL;
	
	/*find buffer in zero copy queue*/
	a_netbuf_ptr = A_NETBUF_DEQUEUE_ADV(&zero_copy_free_queue, buffer);
	
	if(a_netbuf_ptr != NULL)
		A_NETBUF_FREE(a_netbuf_ptr);
	//else
		//printf("Error: buffer not found\n");
}


/*****************************************************************************/
/*  Api_recvfrom - ZERO_COPY version of receive API.It will check socket's
 *				receive quese for pending packets. If a packet is available,
 *				it will be passed to the application without a memcopy. The 
 *				application must call a zero_copy_free API to free this buffer.
 * RETURNS: Number of bytes received or A_ERROR in case of failure
 *****************************************************************************/
A_INT32 Api_recvfrom(A_VOID *pCxt, A_UINT32 handle, void** buffer, A_UINT32 length, A_UINT32 flags, A_VOID* name, A_UINT32* socklength)
{
	A_INT32 index;
	A_UINT32 len = 0, hdrlen = 0;
	A_NETBUF* a_netbuf_ptr = NULL;
	A_UINT8* data = NULL;
	SOCKET_CUSTOM_CONTEXT_PTR pcustctxt;
	
	/*Find context*/
	if((index = find_socket_context(handle,TRUE)) == SOCKET_NOT_FOUND)
	{
		return A_SOCK_INVALID;
	}
	pcustctxt = GET_CUSTOM_SOCKET_CONTEXT(ath_sock_context[index]);
	/*Check if a packet is available*/
	/*Check if a packet is available*/
    if((ath_sock_context[index]->type == SOCK_STREAM_TYPE) && 
       (ath_sock_context[index]->remaining_bytes != 0) &&
       (ath_sock_context[index]->old_netbuf != NULL))
    {
        a_netbuf_ptr = ath_sock_context[index]->old_netbuf;
        data = (A_UINT8*)A_NETBUF_DATA(a_netbuf_ptr);
    }
    else
    {
		while((a_netbuf_ptr = A_NETBUF_DEQUEUE(&(pcustctxt->rxqueue))) == NULL)
		{
#if NON_BLOCKING_RX
            /*No packet is available, return -1*/
            return A_ERROR;	
#else   
             if(ath_sock_context[index]->TCPCtrFlag == TCP_FIN)
	     {
                if(custom_queue_empty(index))
                {
                   printf("fin in recv \r\n");
                   clear_socket_context(index);
                   return A_SOCK_INVALID;
                }
	     } 	     
            /*No packet available, block*/
            if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], 0, RX_DIRECTION) != A_OK)
            {
                /*Check if Peer closed socket while we were waiting*/
		 		if(ath_sock_context[index]->handle == 0)
		    		return A_SOCK_INVALID;            
                if(ath_sock_context[index]->TCPCtrFlag == TCP_FIN)
                 {
                      printf("clr sock in recv \r\n");
                     clear_socket_context(index);
                     return A_SOCK_INVALID;
                 }   
               return A_ERROR;			
            }
#endif     
		}
        data = (A_UINT8*)A_NETBUF_DATA(a_netbuf_ptr);
	}
	
	if(ath_sock_context[index]->domain == ATH_AF_INET)
	{
		hdrlen = sizeof(SOCK_RECV_T); 
		/*If we are receiving data for a UDP socket, extract sockaddr info*/
		if(ath_sock_context[index]->type == SOCK_DGRAM_TYPE)
		{
			A_MEMCPY(name, &((SOCK_RECV_T*)data)->name, sizeof(SOCKADDR_T));
			((SOCKADDR_T*)name)->sin_port = A_CPU2LE16(((SOCKADDR_T*)name)->sin_port);
			((SOCKADDR_T*)name)->sin_family = A_CPU2LE16(((SOCKADDR_T*)name)->sin_family);
			((SOCKADDR_T*)name)->sin_addr = A_CPU2LE32(((SOCKADDR_T*)name)->sin_addr);
			*socklength = sizeof(SOCKADDR_T);
		}
	}
	else if(ath_sock_context[index]->domain == ATH_AF_INET6)
	{
		hdrlen = sizeof(SOCK_RECV6_T);
		/*If we are receiving data for a UDP socket, extract sockaddr info*/
		if(ath_sock_context[index]->type == SOCK_DGRAM_TYPE)
		{
			A_MEMCPY(name, &((SOCK_RECV6_T*)data)->name6, sizeof(SOCKADDR_6_T));
			*socklength = sizeof(SOCKADDR_6_T);
		}
	}
	else
	{
		A_ASSERT(0);
	}
	
	if(!((ath_sock_context[index]->type == SOCK_STREAM_TYPE) && 
         (ath_sock_context[index]->remaining_bytes != 0) &&
         (ath_sock_context[index]->old_netbuf != NULL)))
    {
        /*Remove custom header from payload*/
        A_NETBUF_PULL(a_netbuf_ptr, hdrlen);
    }
	
	len = A_NETBUF_LEN(a_netbuf_ptr);
    if(ath_sock_context[index]->type == SOCK_STREAM_TYPE)
    {
        if(len > length)
        {
            ath_sock_context[index]->remaining_bytes = (len - length);
            len = length;
        }
        else
        {
            ath_sock_context[index]->remaining_bytes = 0;
            ath_sock_context[index]->old_netbuf = NULL;
        }  
    }
    else
    {
        if(len > length)
        {    
            len = length;  //Discard excess bytes
        }
    }
	
	*buffer = (A_UINT8*)A_NETBUF_DATA(a_netbuf_ptr);
    if(ath_sock_context[index]->type == SOCK_STREAM_TYPE)
    {
        if(ath_sock_context[index]->remaining_bytes == 0)
        {    
            A_NETBUF_ENQUEUE(&zero_copy_free_queue, a_netbuf_ptr);
        }
        else
        {
            A_NETBUF_PULL(a_netbuf_ptr, len);
            ath_sock_context[index]->old_netbuf = a_netbuf_ptr;
        }    
    }
    else
    {
        A_NETBUF_ENQUEUE(&zero_copy_free_queue, a_netbuf_ptr);
    }
	return len;
}

#else



/*****************************************************************************/
/*  Api_recvfrom - Non ZERO_COPY version of receive API.It will check socket's
 *				receive quese for pending packets. If a packet is available,
 *				it will be copied into user provided buffer.
 * RETURNS: Number of bytes received or A_ERROR in case of failure
 *****************************************************************************/
A_INT32 Api_recvfrom(A_VOID *pCxt, A_UINT32 handle, void* buffer, A_UINT32 length, A_UINT32 flags, A_VOID* name, A_UINT32* socklength)
{
	A_INT32 index;
	A_UINT32 len = 0, hdrlen=0;
	A_NETBUF* a_netbuf_ptr = NULL;
	A_UINT8* data = NULL;
	SOCKET_CUSTOM_CONTEXT_PTR pcustctxt;
	
	/*Find context*/
	if((index = find_socket_context(handle,TRUE)) == SOCKET_NOT_FOUND)
	{
		return A_SOCK_INVALID;
	}
	pcustctxt = GET_CUSTOM_SOCKET_CONTEXT(ath_sock_context[index]);
	
	while((a_netbuf_ptr = A_NETBUF_DEQUEUE(&(pcustctxt->rxqueue))) == NULL)
	{
#if NON_BLOCKING_RX
		/*No packet is available, return -1*/
		return A_ERROR;	
#else          
		/*No packet available, block*/
		if(CUSTOM_BLOCK(pCxt, ath_sock_context[index], 0, RX_DIRECTION) != A_OK)
		{
			/*Check if Peer closed socket while we were waiting*/
		 	if(ath_sock_context[index]->handle == 0)
		    	return A_SOCK_INVALID;
		 	
		 return A_ERROR;			
		}
#endif     
	}
	
	/*Extract custom header*/
	data = (A_UINT8*)A_NETBUF_DATA(a_netbuf_ptr);
	if(ath_sock_context[index]->domain == ATH_AF_INET)
	{
		hdrlen = sizeof(SOCK_RECV_T); 
		/*If we are receiving data for a UDP socket, extract sockaddr info*/
		if(ath_sock_context[index]->type == SOCK_DGRAM_TYPE)
		{
			A_MEMCPY(name, &((SOCK_RECV_T*)data)->name, sizeof(SOCKADDR_T));
			((SOCKADDR_T*)name)->sin_port = A_CPU2LE16(((SOCKADDR_T*)name)->sin_port);
			((SOCKADDR_T*)name)->sin_family = A_CPU2LE16(((SOCKADDR_T*)name)->sin_family);
			((SOCKADDR_T*)name)->sin_addr = A_CPU2LE32(((SOCKADDR_T*)name)->sin_addr);
			*socklength = sizeof(SOCKADDR_T);
		}
	}
	else if(ath_sock_context[index]->domain == ATH_AF_INET6)
	{
		hdrlen = sizeof(SOCK_RECV6_T);
		/*If we are receiving data for a UDP socket, extract sockaddr info*/
		if(ath_sock_context[index]->type == SOCK_DGRAM_TYPE)
		{
			A_MEMCPY(name, &((SOCK_RECV6_T*)data)->name6, sizeof(SOCKADDR_6_T));
			*socklength = sizeof(SOCKADDR_6_T);
		}
	}
	else
	{
		A_ASSERT(0);
	}
	
	/*Remove custom header from payload*/
	A_NETBUF_PULL(a_netbuf_ptr, hdrlen);
	

	len = A_NETBUF_LEN(a_netbuf_ptr);
		
	if(len > length)
		len = length;  //Discard excess bytes

	A_MEMCPY((A_UINT8*)buffer,(A_UINT8*)A_NETBUF_DATA(a_netbuf_ptr), len);
	A_NETBUF_FREE(a_netbuf_ptr);
	
	return len;
}
#endif






/*****************************************************************************/
/*  txpkt_free - function to free driver buffers that are allocated during send 
 *          operations. The function is called from a_netbuf_free, after tx
 *          has successfully completed.
 *          If NON_BLOCKING_TX is defined, the user buffer is freed here
 *          as well. 
 *    Note 1- called from driver thread.
 *    Note 2- should not be called from application
 * RETURNS: none
 *****************************************************************************/
A_VOID txpkt_free(A_VOID* buffPtr)
{
    A_NETBUF_PTR a_netbuf_ptr = (A_NETBUF*)buffPtr;
    DRV_BUFFER_PTR db_ptr = a_netbuf_ptr->native_orig;
    SOCKET_CUSTOM_CONTEXT_PTR pcustctxt = GET_CUSTOM_SOCKET_CONTEXT((ATH_SOCKET_CONTEXT_PTR)db_ptr->context);		 

#if NON_BLOCKING_TX
    A_VOID* bufPtr = NULL;	
    /*In case of non blocking TX, driver should free the payload buffer*/	
    A_MUTEX_LOCK(&(pcustctxt->nb_tx_mutex));        
    bufPtr = A_NETBUF_DEQUEUE(&(pcustctxt->non_block_queue));
    A_MUTEX_UNLOCK(&(pcustctxt->nb_tx_mutex));                
    
    if(bufPtr != NULL) {    
      A_FREE(((A_UINT8*)bufPtr - TX_PKT_OVERHEAD),MALLOC_ID_CONTEXT);  
      bufPtr = NULL;
    }
    
    /*We are done with netbuf, free Netbuf structure here*/
    A_FREE(a_netbuf_ptr->head, MALLOC_ID_NETBUFF);
    A_FREE(a_netbuf_ptr, MALLOC_ID_NETBUFF_OBJ);    
#else
    /*Tail, data pointers within netbuf may have moved during previous TX, 
     reinit the netbuf so it can be used again*/
    a_netbuf_ptr = a_netbuf_reinit((A_VOID*)a_netbuf_ptr, TCP6_HEADROOM);
    
    /*Unblock the application thread*/
    CUSTOM_UNBLOCK(db_ptr->context, TX_DIRECTION);
#endif

}



/*****************************************************************************/
/*  custom_alloc - API for application to allocate a TX buffer. Here we ensure that 
              enough resources are available for a packet TX. All allocations 
             for a TX operation are made here. This includes Headroom, DB and netbuf.
             If any allocation fails, this API returns NULL, and the host must 
             wait for some time to allow memory to be freed. 
      Note 1-  Allocation may fail if too many packets are queued up for Transmission,
               e.g. in the the non-blocking TX case.
      Note 2- This API should ONLY be used for TX packets.

 * RETURNS: pointer to payload buffer for success and NULL for failure 
 *****************************************************************************/
A_VOID* custom_alloc(A_UINT32 size)
{
    A_UINT32 total_size = 0;
    A_UINT8* buffer, *payload;
    A_UINT32 hdr_length = 0;     
    
    /*Allocate TX buffer that will include- payload + headroom + Driver buffer + pointer to netbuf*/
    total_size = size + TX_PKT_OVERHEAD;
    
    /*Round off to 4 byte boundary*/
    if((total_size % 4) != 0){
        total_size += 4 - total_size%4;
    }
    if((buffer = A_MALLOC(total_size, MALLOC_ID_CONTEXT)) == NULL){
      return NULL;
    }
    
    /*Allocate netbuf with max possible headroom here so that there is no need to do it during TX*/
    if((((TX_PACKET_PTR)buffer)->a_netbuf_ptr = A_NETBUF_ALLOC(TCP6_HEADROOM)) == NULL){ 
      A_FREE(buffer, MALLOC_ID_CONTEXT);
      return NULL;
    }
    /*Obtain pointer to start of payload*/
    payload = buffer + TX_PKT_OVERHEAD;
    return payload;
}


/*****************************************************************************/
/*  custom_free - API for application to free TX buffer. It should ONLY be called
 *              by the app if Blocking TX mode is enabled. It will free all allocations
 *              made during the custom_alloc 
 * RETURNS: none 
 *****************************************************************************/
A_VOID custom_free(A_VOID* buf)
{
#if (!NON_BLOCKING_TX)	  
    A_UINT8* buffer;
    A_NETBUF_PTR a_netbuf_ptr;
    
    /*Move the the begining of TX buffer*/
    buffer = (A_UINT8*)buf - TX_PKT_OVERHEAD; 
    
    /*Get pointer to netbuf from TX buffer*/
    a_netbuf_ptr = ((TX_PACKET_PTR)buffer)->a_netbuf_ptr;
    
    if(a_netbuf_ptr)
    {
        /*We are done with netbuf, free Netbuf structure here*/
        A_FREE(a_netbuf_ptr->head, MALLOC_ID_NETBUFF);
	A_FREE(a_netbuf_ptr, MALLOC_ID_NETBUFF_OBJ);  
    }
    A_FREE(buffer, MALLOC_ID_CONTEXT);
#endif    
}



/*****************************************************************************/
/*  get_total_pkts_buffered - Returns number of packets buffered across all
 *          socket queues.           
 * RETURNS: number of packets 
 *****************************************************************************/
A_UINT32 get_total_pkts_buffered()
{
	A_UINT32 index;
	A_UINT32 totalPkts = 0;
	
	SOCKET_CUSTOM_CONTEXT_PTR pcustctxt;
	
	for(index = 0; index < MAX_SOCKETS_SUPPORTED; index++)
	{
		pcustctxt = GET_CUSTOM_SOCKET_CONTEXT(ath_sock_context[index]);
		totalPkts += A_NETBUF_QUEUE_SIZE(&(pcustctxt->rxqueue));
	}
	return totalPkts;
}



/*****************************************************************************/
/*  custom_queue_empty - Checkes whether a socket queue is empty
 * RETURNS: 1 - empty or 0 - not empty 
 *****************************************************************************/
A_UINT32 custom_queue_empty(A_UINT32 index)
{
	SOCKET_CUSTOM_CONTEXT_PTR pcustctxt;
	pcustctxt = GET_CUSTOM_SOCKET_CONTEXT(ath_sock_context[index]);
		
	return A_NETBUF_QUEUE_EMPTY(&(pcustctxt->rxqueue));
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
SSL_CTX* SSL_ctx_new(SSL_ROLE_T role, A_INT32 inbufSize, A_INT32 outbufSize, A_INT32 reserved)
{
    A_VOID *pCxt;

    if((pCxt = ((ENET_CONTEXT_STRUCT_PTR)sslhandle)->MAC_CONTEXT_PTR) == NULL){
        return NULL;
    }

    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)) {
        return NULL;
    }
	return (Api_SSL_ctx_new(pCxt,role,inbufSize,outbufSize,reserved));
}

/*****************************************************************************/
/* SSL_ctx_free - Free the SSL context
 * SSL_CTX *ctx - sslContext
 *****************************************************************************/
A_INT32 SSL_ctx_free(SSL_CTX *ctx)
{
    A_VOID *pCxt;

    if((pCxt = ((ENET_CONTEXT_STRUCT_PTR)sslhandle)->MAC_CONTEXT_PTR) == NULL){
        return A_ERROR;
    }

    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)) {
        return A_ERROR;
    }
	return (Api_SSL_ctx_free(pCxt,ctx));
}

/*****************************************************************************/
/* SSL_new - Create SSL connection object. When SSL transaction is done, close
 *           it with ssl_shutdown().
 *           Note that the socket should subsequently also be closed.
 * SSL_CTX *ctx - sslContext
 * Return - SSL object handle on success or NULL on error (out of memory)
 *****************************************************************************/
SSL* SSL_new(SSL_CTX *ctx)
{
    A_VOID *pCxt;

    if((pCxt = ((ENET_CONTEXT_STRUCT_PTR)sslhandle)->MAC_CONTEXT_PTR) == NULL){
        return NULL;
    }

	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)) {
        return NULL;
    }
	return (Api_SSL_new(pCxt,ctx));
}

/*****************************************************************************/
/* SSL_set_fd - Attach given socket descriptor to the SSL connection
 * SSL *ssl - SSL connection
 * A_UINT32 fd - Socket descriptor
 * Return - 1 on success or negative error code on error (see SslErrors)
 *****************************************************************************/
A_INT32 SSL_set_fd(SSL *ssl, A_UINT32 fd)
{
    A_VOID *pCxt;

    if((pCxt = ((ENET_CONTEXT_STRUCT_PTR)sslhandle)->MAC_CONTEXT_PTR) == NULL){
        return A_ERROR;
    }

	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)) {
        return A_ERROR;
    }
	return (Api_SSL_set_fd(pCxt,ssl,fd));
}

/*****************************************************************************/
/* SSL_accept - Initiate SSL handshake.
 * SSL *ssl - SSL connection
 * Returns - 1 on success, ESSL_HSDONE if handshake has already been performed.
 *           Negative error code otherwise.
 *****************************************************************************/
A_INT32 SSL_accept(SSL *ssl)
{
    A_VOID *pCxt;

    if((pCxt = ((ENET_CONTEXT_STRUCT_PTR)sslhandle)->MAC_CONTEXT_PTR) == NULL){
        return A_ERROR;
    }

	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)) {
        return A_ERROR;
    }
	return (Api_SSL_accept(pCxt,ssl));
}

/*****************************************************************************/
/* SSL_connect - Initiate SSL handshake.
 * SSL *ssl - SSL connection
 * Returns - 1 on success, ESSL_HSDONE if handshake has already been performed.
 *           Negative error code otherwise.
 *****************************************************************************/
A_INT32 SSL_connect(SSL *ssl)
{
    A_VOID *pCxt;

    if((pCxt = ((ENET_CONTEXT_STRUCT_PTR)sslhandle)->MAC_CONTEXT_PTR) == NULL){
        return A_ERROR;
    }

	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)) {
        return A_ERROR;
    }
	return (Api_SSL_connect(pCxt,ssl));
}

/*****************************************************************************/
/* SSL_shutdown - Close SSL connection.
 *                The socket must be closed by other means.
 * SSL *ssl - SSL connection
 * Returns - 1 on success or negative error code on error (see SslErrors)
 *****************************************************************************/
A_INT32 SSL_shutdown(SSL *ssl)
{
    A_VOID *pCxt;

    if((pCxt = ((ENET_CONTEXT_STRUCT_PTR)sslhandle)->MAC_CONTEXT_PTR) == NULL){
        return A_ERROR;
    }

	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)) {
        return A_ERROR;
    }
	return (Api_SSL_shutdown(pCxt,ssl));
}

/*****************************************************************************/
/* SSL_configure - Configure a SSL connection.
 * SSL *ssl - SSL connection
 * SSL_CONFIG *cfg - pointer to struct holding the SSL configuration.
 * Returns - 1 on success or negative error code on error (see SslErrors)
 *****************************************************************************/
A_INT32 SSL_configure(SSL *ssl, SSL_CONFIG *cfg)
{
    A_VOID *pCxt;

    if((pCxt = ((ENET_CONTEXT_STRUCT_PTR)sslhandle)->MAC_CONTEXT_PTR) == NULL){
        return A_ERROR;
    }

    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)) {
        return A_ERROR;
    }
    return (Api_SSL_configure(pCxt, ssl, cfg));
}

/*****************************************************************************/
/* SSL_setCaList - Set a Certificate Authority (CA) list so SSL can perform
 *                 certificate validation on the peer's certificate.
 *                 You can only set one CA list, thus the CA list must include
 *                 all root certificates required for the session.
 * SSL *ssl - SSL connection
 * SslCert cert -address of array of binary data
 * A_UINT32 size - size of array
 * Returns - 1 on success or negative error code on error (see SslErrors)
 *****************************************************************************/
A_INT32 SSL_setCaList(SSL_CTX *ctx, SslCAList caList, A_UINT32 size)
{
    A_VOID *pCxt;

    if((pCxt = ((ENET_CONTEXT_STRUCT_PTR)sslhandle)->MAC_CONTEXT_PTR) == NULL){
        return A_ERROR;
    }

    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)) {
        return A_ERROR;
    }
    return (Api_SSL_addCert(pCxt, ctx, SSL_CA_LIST, (A_UINT8*)caList, size));
}

/*****************************************************************************/
/* SSL_addCert - Add a certificate to the SharkSsl object. A SharkSsl object
 *               in server mode is required to have at least one certificate.
 * SSL *ssl - SSL connection
 * SslCert cert -address of array of binary data
 * A_UINT32 size - size of array
 * Returns - 1 on success or negative error code on error (see SslErrors)
 *****************************************************************************/
A_INT32 SSL_addCert(SSL_CTX *ctx, SslCert cert, A_UINT32 size)
{
    A_VOID *pCxt;

    if((pCxt = ((ENET_CONTEXT_STRUCT_PTR)sslhandle)->MAC_CONTEXT_PTR) == NULL){
        return A_ERROR;
    }

	/*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)) {
        return A_ERROR;
    }
	return (Api_SSL_addCert(pCxt, ctx, SSL_CERTIFICATE, (A_UINT8*)cert, size));
}

/*****************************************************************************/
/* SSL_storeCert - Store a certificate or CA list in FLASH.
 * A_CHAR *name - the name
 * SslCert cert -address of array of binary data
 * A_UINT32 size - size of array
 * Returns - 1 on success or negative error code on error (see SslErrors)
 *****************************************************************************/
A_INT32 SSL_storeCert(A_CHAR *name, SslCert cert, A_UINT32 size)
{
    A_VOID *pCxt;

    if((pCxt = ((ENET_CONTEXT_STRUCT_PTR)sslhandle)->MAC_CONTEXT_PTR) == NULL){
        return A_ERROR;
    }

    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)) {
        return A_ERROR;
    }
    return (Api_SSL_storeCert(pCxt, name, (A_UINT8*)cert, size));
}

/*****************************************************************************/
/* SSL_loadCert - Load a certificate or CA list from FLASH and store it in the
 *                sharkSs object.
 * SSL_CERT_TYPE_T type - certificate or CA list
 * A_CHAR *name - the name
 * Returns - 1 on success or negative error code on error (see SslErrors)
 *****************************************************************************/
A_INT32 SSL_loadCert(SSL_CTX *ctx, SSL_CERT_TYPE_T type, char *name)
{
    A_VOID *pCxt;

    if((pCxt = ((ENET_CONTEXT_STRUCT_PTR)sslhandle)->MAC_CONTEXT_PTR) == NULL){
        return A_ERROR;
    }

    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)) {
        return A_ERROR;
    }
    return (Api_SSL_loadCert(pCxt, ctx, type, name));
}

A_INT32 SSL_listCert(SSL_FILE_NAME_LIST *fileNames)
{
    A_VOID *pCxt;

    if((pCxt = ((ENET_CONTEXT_STRUCT_PTR)sslhandle)->MAC_CONTEXT_PTR) == NULL){
        return A_ERROR;
    }

    /*Wait for chip to be up*/
    if(A_OK != Api_DriverAccessCheck(pCxt, 1, ACCESS_REQUEST_IOCTL)) {
        return A_ERROR;
    }
    return (Api_SSL_listCert(pCxt, fileNames));
}

#if ZERO_COPY
/*****************************************************************************/
/*  SSL_read - ZERO_COPY version of SSL read function. It uses the ssl pointer
 *             to find the socket to read from.It will check socket's receive
 *             queues for pending packets.  If a packet is available, it will be
 *             passed to the application without a memcopy. The application must
 *             call a zero_copy_free API to free this buffer.
 * SSL *ssl - pointer to SSL connection object.
 * void **buf - pointer to pointer holding the address of the receive buffer.
 * A_INT32 num - The max number of bytes to read.
 * Returns - Number of bytes received or A_ERROR in case of failure
 *****************************************************************************/
A_INT32 SSL_read(SSL *ssl, void **buf, A_INT32 num)
#else
A_INT32 SSL_read(SSL *ssl, void *buf, A_INT32 num)
#endif
{
    /* Find the socket used with this SSL connection */
    A_INT32 index = find_socket_context_from_ssl(ssl);
    if (index == SOCKET_NOT_FOUND)
    {
        return A_ERROR;
    }

    /* Read data from the socket */
    return t_recv(sslhandle, ath_sock_context[index]->handle, buf, num, 0);
}

/*****************************************************************************/
/* SSL_write - Encrypt and send data in buf and send them on the associated
 *             socket. The ssl pointer is sued to find the socket to send on.
 * SSL *ssl - pointer to SSL connection object.
 * void *buf - pointer to buffer holding the data to send.
 * A_INT32 num  - The number of bytes to send.
 * Returns - Number of bytes sent on success or negative error code on error
 *****************************************************************************/
A_INT32 SSL_write(SSL *ssl, const void *buf, A_INT32 num)
{
    /* Find the socket used with this SSL connection */
    A_INT32 index = find_socket_context_from_ssl(ssl);
    if (index == SOCKET_NOT_FOUND)
    {
        return A_ERROR;
    }

    /* Send the data */
    return t_send(sslhandle, ath_sock_context[index]->handle, (A_UINT8 *)buf, num, 0);
}

#endif // ENABLE_SSL
#endif
