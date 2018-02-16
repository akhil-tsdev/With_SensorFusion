/*
 * Copyright (c) 2011 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
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
 *  
 */
#ifndef __COMMON_STACK_OFFLOAD_H__
#define __COMMON_STACK_OFFLOAD_H__


#if ENABLE_STACK_OFFLOAD

#include "atheros_stack_offload.h"

#if TURINGSENSE
#define COMMAND_BLOCK_TIMEOUT       (4000)	  //used during connect
#define TRANSMIT_BLOCK_TIMEOUT      (4000)   //cw tmp
#define TS_RECV_BLOCK_TIMEOUT		(4000)   //Receive blocking timeout
#else
//#define COMMAND_BLOCK_TIMEOUT       (960000)   //Socket will block for this period in msec, if no
                                              //response is received, socket will unblock
//#define TRANSMIT_BLOCK_TIMEOUT      (900000)  //Time in ms for which a send operation blocks,
											  //after this time, error is returned to application
#endif /* TURINGSENSE */
#define LAST_FRAGMENT               (0x01)    //Indicates this is the last fragment 
#define RX_DIRECTION                (0)
#define TX_DIRECTION                (1)
#define TCP_CONNECTION_AVAILABLE    (99)      //Sent by Target in Listen event that a new TCP 
                                              // incoming connection is available. Application
                                              // should call accept on receving this from target.
#define DHCP_WAIT_TIME    60000   // Wait for 1 Minute. On time out configure Auto Address

#if DEBUG
    #define DEBUG_PRINT(arg)        //User may define
#endif

#define TCP_MSS  1400

#define TCP_FIN  1
#define TCP_LISTEN 2

typedef struct ath_socket_context {
    A_INT32    handle;		    //Socket handle	
    A_UINT32   sock_st_mask;
    A_INT32    result; 		    //API return value	
    A_VOID*    sock_custom_context; //Pointer to custom socket context 
    A_VOID*    pReq;                //Used to hold wmi netbuf to be freed from the user thread  
    A_UINT8*   data;		    //Generic pointer to data recevied from target
    A_UINT8    domain;		    //IPv4/v6
    A_UINT8    type;		    //TCP vs UDP
    A_UINT16   remaining_bytes;
    A_VOID     *old_netbuf;
    A_UINT32   sock_st_mask2;      //  To handle commands greater than 31
#if ENABLE_SSL
    SSL *ssl;  // SSL connection object
#endif
    A_UINT8    TCPCtrFlag;
} ATH_SOCKET_CONTEXT, *ATH_SOCKET_CONTEXT_PTR;


typedef struct ath_sock_stack_init {
	A_UINT8	stack_enabled;            //Flag to indicate if stack should be enabled in the target
	A_UINT8 num_sockets;              //number of sockets supported by the host
	A_UINT8 num_buffers;              //Number of RX buffers supported by host
	A_UINT8 reserved;
} ATH_STACK_INIT;

enum SOCKET_CMDS
{
    SOCK_OPEN = 0,		/*Open a socket*/
    SOCK_CLOSE,		/*Close existing socket*/
    SOCK_CONNECT,		/*Connect to a peer*/
    SOCK_BIND,		/*Bind to interface*/
    SOCK_LISTEN,		/*Listen on socket*/
    SOCK_ACCEPT,		/*Accept incoming connection*/
    SOCK_SELECT,		/*Wait for specified file descriptors*/
    SOCK_SETSOCKOPT,	/*Set specified socket option*/
    SOCK_GETSOCKOPT,	/*Get socket option*/
    SOCK_ERRNO,		/*Get error number for last error*/
    SOCK_IPCONFIG,          /*Set static IP information, or get current IP config*/
    SOCK_PING,
    SOCK_STACK_INIT,        /*Command to initialize stack*/
    SOCK_STACK_MISC,		/*Used to exchanges miscellaneous info, e.g. reassembly etc*/
    SOCK_PING6,
    SOCK_IP6CONFIG,          /*Set static IP information, or get current IP config*/
    SOCK_IPCONFIG_DHCP_POOL,          /*Set DHCP Pool  */
    SOCK_IP6CONFIG_ROUTER_PREFIX,    /* Set ipv6 router prefix */
    SOCK_IP_SET_TCP_EXP_BACKOFF_RETRY,  /* set tcp exponential backoff retry */
    SOCK_IP_SET_IP6_STATUS,  /* set ip6 module status enable/disable */
    SOCK_IP_DHCP_RELEASE, /* Release the DHCP IP Addres */
    SOCK_IP_SET_TCP_RX_BUF,  /* set tcp rx buffer space */
    SOCK_HTTP_SERVER,             /* Http Server Command*/
    SOCK_HTTP_SERVER_CMD,         /* Commands to get and post data */
    SOCK_DNC_CMD,         /* Commands related to resolver */
    SOCK_DNC_ENABLE,       /* Command to enable/disable DNS Client */
    SOCK_DNS_SRVR_CFG_ADDR,  /* Command to configure DNS Server Address */
    SOCK_HTTPC,               /* HTTP Client commands */
    SOCK_DNS_LOCAL_DOMAIN,   /* Configures the local domain */
    SOCK_IP_HOST_NAME,	 /* Configures the local host name */
    SOCK_IP_DNS,              /* Configures DNS Database */
    SOCK_IP_SNTP_SRVR_ADDR,    /* Configures the sntp server addr */
    SOCK_IP_SNTP_GET_TIME,  /* GET UTC Time from SNTP Client */
    SOCK_IP_SNTP_GET_TIME_OF_DAY, /* Get time of day (secs)*/
    SOCK_IP_SNTP_CONFIG_TIMEZONE_DSE, /*Command to modify time zone and to enable/disable DSE */
    SOCK_IP_SNTP_QUERY_SNTP_ADDRESS, /* Command to query SNTP Server Address*/
    SOCK_IP_SNTP_CLIENT_ENABLE,          /* Command to enable/disable SNTP client */
    SOCK_SSL_CTX_NEW,       /* Create a new SSL context */
    SOCK_SSL_CTX_FREE,      /* Free/close SSL context */
    SOCK_SSL_NEW,           /* Create new SSL connection object/instance */
    SOCK_SSL_SET_FD,        /* Add socket handle to a SSL connection */
    SOCK_SSL_ACCEPT,        /* Accept SSL connection request from SSL client */
    SOCK_SSL_CONNECT,       /* Establish SSL connection from SSL client to SSL server */
    SOCK_SSL_SHUTDOWN,      /* Shutdown/close SSL connection */
    SOCK_SSL_ADD_CERT,      /* Add a certificate to SSL context */
    SOCK_SSL_STORE_CERT,    /* Store a certificate or CA list file in FLASH */
    SOCK_SSL_LOAD_CERT,     /* Reads a certificate or CA list from FLASH and adds it to SSL context */
    SOCK_SSL_CONFIGURE,     /* Configure a SSL connection */
    SOCK_SSL_LIST_CERT      /* Request the names of the cert's stored in FLASH */
};

typedef PREPACK struct sock_open {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
	A_UINT32 domain FIELD_PACKED;		//ATH_AF_INET, ATH_AF_INET6
	A_UINT32 type FIELD_PACKED;			//SOCK_STREAM_TYPE, SOCK_DGRAM_TYPE
	A_UINT32 protocol FIELD_PACKED;		// 0	
}POSTPACK SOCK_OPEN_T;


typedef PREPACK struct sock_close {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
	A_UINT32 handle FIELD_PACKED;		//socket handle
}POSTPACK SOCK_CLOSE_T;

typedef PREPACK struct sock_connect_cmd {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
	A_UINT32 handle FIELD_PACKED;		//socket handle
	union 
	{
		SOCKADDR_T name FIELD_PACKED;
		SOCKADDR_6_T name6 FIELD_PACKED;	
	}addr;
	A_UINT16 length FIELD_PACKED;		//socket address length
}POSTPACK SOCK_CONNECT_CMD_T, SOCK_BIND_CMD_T, SOCK_ACCEPT_CMD_T;

typedef PREPACK struct sock_connect_recv {
    A_UINT32 handle FIELD_PACKED;		//socket handle
	union 
	{
		SOCKADDR_T name FIELD_PACKED;
		SOCKADDR_6_T name6 FIELD_PACKED;	
	}addr;
	A_UINT16 length FIELD_PACKED;		//socket address length
}POSTPACK SOCK_CONNECT_RECV_T, SOCK_BIND_RECV_T, SOCK_ACCEPT_RECV_T;

typedef PREPACK struct sock_errno {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32 errno;
}POSTPACK SOCK_ERRNO_T;

typedef struct sock_listen {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
	A_UINT32 handle FIELD_PACKED;		//Socket handle
	A_UINT32 backlog FIELD_PACKED;		//Max length of queue of backlog connections
}POSTPACK SOCK_LISTEN_T;

typedef PREPACK struct sock_setopt 
{
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
	A_UINT32 handle FIELD_PACKED;		//socket handle
	A_UINT32 level FIELD_PACKED;		//Option level (ATH_IPPROTO_IP, ATH_IPPROTO_UDP, ATH_IPPROTO_TCP...)
	A_UINT32 optname FIELD_PACKED;		//Choose from list above
	A_UINT32 optlen FIELD_PACKED;		// option of length
	A_UINT8  optval[1] FIELD_PACKED;	// option value
}POSTPACK SOCK_OPT_T;



typedef PREPACK struct ipconfig_recv {
    A_UINT32        mode FIELD_PACKED;			//0-query, 1-static, 2-dhcp
    A_UINT32        ipv4 FIELD_PACKED;				//IPv4 address
    A_UINT32        subnetMask FIELD_PACKED;
    A_UINT32        gateway4 FIELD_PACKED;			
    IP6_ADDR_T      ipv6LinkAddr FIELD_PACKED;     /* IPv6 Link Local address */
    IP6_ADDR_T      ipv6GlobalAddr FIELD_PACKED;   /* IPv6 Global address */
    IP6_ADDR_T      ipv6DefGw FIELD_PACKED;     	  /* IPv6 Default Gateway */
    IP6_ADDR_T      ipv6LinkAddrExtd FIELD_PACKED;     /* IPv6 Link Local address for Logo*/
    A_INT32         LinkPrefix FIELD_PACKED;
    A_INT32         GlbPrefix FIELD_PACKED;
    A_INT32         DefGwPrefix FIELD_PACKED;
    A_INT32         GlbPrefixExtd FIELD_PACKED;
    IP46ADDR        dnsaddr[MAX_DNSADDRS] FIELD_PACKED;
    A_CHAR          hostname[33] FIELD_PACKED; 
}POSTPACK IPCONFIG_RECV_T;


typedef PREPACK struct ipconfig {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32        mode FIELD_PACKED;			//0-query, 1-static, 2-dhcp
    A_UINT32        ipv4 FIELD_PACKED;				//IPv4 address
    A_UINT32        subnetMask FIELD_PACKED;
    A_UINT32        gateway4 FIELD_PACKED;			
    IP6_ADDR_T      ipv6LinkAddr FIELD_PACKED;     /* IPv6 Link Local address */
    IP6_ADDR_T      ipv6GlobalAddr FIELD_PACKED;   /* IPv6 Global address */
    IP6_ADDR_T      ipv6DefGw FIELD_PACKED;     	  /* IPv6 Default Gateway */
    IP6_ADDR_T      ipv6LinkAddrExtd FIELD_PACKED;     /* IPv6 Link Local address for Logo*/
    A_INT32         LinkPrefix;
    A_INT32         GlbPrefix;
    A_INT32         DefGwPrefix;
    A_INT32         GlbPrefixExtd;
}POSTPACK IPCONFIG_CMD_T;

typedef PREPACK struct ping {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32 ip_addr FIELD_PACKED;			//Destination IPv4 address
    A_UINT32 size FIELD_PACKED;	
}POSTPACK PING_T;

typedef PREPACK struct ping6 {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT8 ip6addr[16] FIELD_PACKED;		//Destination IPv6 address
    A_UINT32 size FIELD_PACKED;
}POSTPACK PING_6_T;


typedef PREPACK struct ipconfigdhcppool {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
	A_UINT32   startaddr FIELD_PACKED;
	A_UINT32   endaddr FIELD_PACKED;
	A_INT32   leasetime FIELD_PACKED;
}POSTPACK IPCONFIG_DHCP_POOL_T;


typedef PREPACK struct  ip6config_router_prefix  {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT8  v6addr[16]  FIELD_PACKED;
    A_INT32  prefixlen  FIELD_PACKED; 
    A_INT32  prefix_lifetime FIELD_PACKED;
    A_INT32  valid_lifetime FIELD_PACKED;
}POSTPACK IP6CONFIG_ROUTER_PREFIX_T;	


typedef PREPACK struct  sock_ip_backoff  {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;    
    A_INT32  max_retry FIELD_PACKED;
}POSTPACK SOCK_IP_BACKOFF_T;

typedef PREPACK struct sock_ipv6_status {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED; 
    A_UINT16 ipv6_status FIELD_PACKED;
}POSTPACK SOCK_IPv6_STATUS_T;

typedef PREPACK struct sock_ip_dhcp_release {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;    
    A_UINT16  ifIndex FIELD_PACKED;
}POSTPACK SOCK_IP_DHCP_RELEASE_T;	


typedef PREPACK struct  sock_ip_tcp_rx_buf  {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;    
    A_INT32  rxbuf FIELD_PACKED;
}POSTPACK SOCK_IP_TCP_RX_BUF_T;

typedef PREPACK struct  sock_ip_http_server  {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;    
    A_INT32        enable FIELD_PACKED;
}POSTPACK SOCK_IP_HTTP_SERVER_T;

typedef PREPACK struct  sock_ip_http_server_cmd  {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;    
    A_INT32   command FIELD_PACKED;
    A_UINT8    pagename[32] FIELD_PACKED;
    A_UINT8    objname[32] FIELD_PACKED;
    A_UINT32   objlen FIELD_PACKED;
    A_UINT8    value[32] FIELD_PACKED;
}POSTPACK SOCK_IP_HTTP_SERVER_CMD_T;

typedef PREPACK struct  sock_ip_dns_client  {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;    
    A_INT32  command FIELD_PACKED;
}POSTPACK SOCK_IP_DNS_CLIENT_T;

typedef PREPACK struct sock_ip_dns_config_server_addr {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    IP46ADDR addr FIELD_PACKED;			//Server address
}POSTPACK SOCK_IP_CFG_DNS_SRVR_ADDR;

typedef PREPACK struct sock_ip_dns_local_domain {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_CHAR domain_name[33] FIELD_PACKED;			//Server address
}POSTPACK SOCK_IP_CFG_DNS_LOCAL_DOMAIN;

typedef PREPACK struct sock_ip_dns_hostname {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_CHAR domain_name[33] FIELD_PACKED;			//Server address
}POSTPACK SOCK_IP_CFG_HOST_NAME;

typedef PREPACK struct sock_ip_dns {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_INT32 command FIELD_PACKED;
    A_CHAR domain_name[36] FIELD_PACKED;			//Server address
    IP46ADDR addr FIELD_PACKED;
}POSTPACK SOCK_IP_DNS_T;

#if ENABLE_HTTP_CLIENT
typedef PREPACK struct httpc_command {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32  command FIELD_PACKED;
    A_UINT8   url[32] FIELD_PACKED;
    A_UINT8   data[64] FIELD_PACKED;
}POSTPACK SOCK_HTTPC_T;
#endif /* ENABLE_HTTP_CLIENT */

typedef PREPACK struct sock_ip_sntp_local_domain {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_INT32 command FIELD_PACKED;
    A_CHAR addr[68] FIELD_PACKED;			//Server address
}POSTPACK SOCK_IP_CFG_SNTP_SRVR_ADDR;

typedef PREPACK struct sock_ip_sntp_zone
{
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT8 hour FIELD_PACKED;
    A_UINT8 min FIELD_PACKED;
    A_UINT8 add_sub FIELD_PACKED; // add=1,sub=0
    A_UINT8 dse FIELD_PACKED;//enable/disable day light saving
}POSTPACK SOCK_SNTP_MODIFY_TIMEZONE;

typedef PREPACK struct sock_ip_sntp_config
{
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    tSntpDnsAddr SntpDnsAddr[MAX_SNTP_SERVERS] FIELD_PACKED;
}POSTPACK SOCK_IP_QUERY_SNTP_CONFIG;

typedef PREPACK struct  sock_ip_sntp_client  {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;    
    A_INT32  command FIELD_PACKED;
}POSTPACK SOCK_IP_SNTP_CLIENT_T;
/*This structure is sent to the target in a data packet.
  It allows the target to route the data to correct socket with 
  all the necessary parameters*/
typedef PREPACK struct sock_send 
{    
	A_UINT32 handle FIELD_PACKED;			//Socket handle
	A_UINT16 length FIELD_PACKED;			//Payload length
	A_UINT16 reserved FIELD_PACKED;			//Reserved	
	A_UINT32 flags FIELD_PACKED;			//Send flags
	SOCKADDR_T name FIELD_PACKED; 			//IPv4 destination socket information
	A_UINT16 socklength FIELD_PACKED;		
}POSTPACK SOCK_SEND_T;

typedef PREPACK struct sock_send6 
{    
	A_UINT32 handle FIELD_PACKED;			//Socket handle
	A_UINT16 length FIELD_PACKED;			//Payload length
	A_UINT16 reserved FIELD_PACKED;			//Reserved		
	A_UINT32 flags FIELD_PACKED;			//Send flags
	SOCKADDR_6_T name6 FIELD_PACKED;		//IPv6 destination socket information
	A_UINT16 socklength FIELD_PACKED;		
}POSTPACK SOCK_SEND6_T;

typedef PREPACK struct sock_recv 
{    
	A_UINT32 handle FIELD_PACKED;			//Socket handle
	SOCKADDR_T name FIELD_PACKED; 		    //IPv4 destination socket information
	A_UINT16 socklength FIELD_PACKED;		// Length of sockaddr structure
	A_UINT16 reserved FIELD_PACKED;		    // Length of sockaddr structure	
	A_UINT32 reassembly_info FIELD_PACKED;   //Placeholder for reassembly info
}POSTPACK SOCK_RECV_T;

typedef PREPACK struct sock_recv6 
{    
	A_UINT32 handle FIELD_PACKED;			//Socket handle
	SOCKADDR_6_T name6 FIELD_PACKED;  		//IPv6 destination socket information
	A_UINT16 socklength FIELD_PACKED;
	A_UINT16 reserved FIELD_PACKED;			//Reserved	
	A_UINT32 reassembly_info FIELD_PACKED; 	//Placeholder for reassembly info
}POSTPACK SOCK_RECV6_T;

#if ENABLE_SSL
typedef PREPACK struct sock_ssl_new {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32 ctx FIELD_PACKED;  // SSL context
}POSTPACK SOCK_SSL_NEW_T;

typedef PREPACK struct sock_ssl_set_fd {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32 ssl FIELD_PACKED;  // SSL connection object
    A_UINT32 fd FIELD_PACKED;   // Socket handle
}POSTPACK SOCK_SSL_SET_FD_T;

typedef PREPACK struct sock_ssl_accept {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32 ssl FIELD_PACKED;  // SSL connection object
}POSTPACK SOCK_SSL_ACCEPT_T;

typedef PREPACK struct sock_ssl_connect {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32 ssl FIELD_PACKED;  // SSL connection object
}POSTPACK SOCK_SSL_CONNECT_T;

typedef PREPACK struct sock_ssl_shutdown {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32 ssl FIELD_PACKED;  // SSL connection object
}POSTPACK SOCK_SSL_SHUTDOWN_T;

typedef PREPACK struct sock_ssl_configure {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32 ssl FIELD_PACKED;     // SSL connection object
    A_UINT32 size FIELD_PACKED;    // the size of the configuration data
    A_UINT8  data[sizeof(SSL_CONFIG)] FIELD_PACKED; // The configuration data
}POSTPACK SOCK_SSL_CONFIGURE_T;

typedef PREPACK struct sock_ssl_ctx_new {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32 role FIELD_PACKED;
    A_INT32 inbufSize FIELD_PACKED;
    A_INT32 outbufSize FIELD_PACKED;
    A_INT32 reserved FIELD_PACKED;
}POSTPACK SOCK_SSL_CTX_NEW_T;

typedef PREPACK struct sock_ssl_ctx_free {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32 ctx FIELD_PACKED;  // SSL context
}POSTPACK SOCK_SSL_CTX_FREE_T;

typedef PREPACK struct sock_ssl_add_cert {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32 ctx FIELD_PACKED;    // SSL context
    A_UINT32 type FIELD_PACKED;   // Type: 1 for device certificate, 2: CA list
    A_UINT32 total FIELD_PACKED;  // The size of ca_list
    A_UINT32 offset FIELD_PACKED; // offset of this fragment
    A_UINT32 size FIELD_PACKED;   // fragment size
    A_UINT8 data[1] FIELD_PACKED; // CA list or certificate data
}POSTPACK SOCK_SSL_ADD_CERT_T;

typedef PREPACK struct sock_ssl_store_cert {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT8  name[32] FIELD_PACKED; // The name of the certificate or CA list
    A_UINT32 total FIELD_PACKED;    // The size of the certificate or CA list
    A_UINT32 offset FIELD_PACKED;   // offset of this fragment
    A_UINT32 size FIELD_PACKED;     // fragment size
    A_UINT8 data[1] FIELD_PACKED;   // certificate or CA list data
}POSTPACK SOCK_SSL_STORE_CERT_T;

typedef PREPACK struct sock_ssl_load_cert {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32 ctx FIELD_PACKED;      // SSL context
    A_UINT32 type FIELD_PACKED;     // Type: 1 for device certificate, 2: CA list
    A_UINT8  name[32] FIELD_PACKED; // The name of the certificate or CA list
}POSTPACK SOCK_SSL_LOAD_CERT_T;

typedef PREPACK struct sock_ssl_list_cert {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32 reserved FIELD_PACKED;
}POSTPACK SOCK_SSL_LIST_CERT_T;

#endif


#define SOCK_EV_MASK_SET(_pcxt, _cmd)    (_pcxt)->sock_st_mask |= 1<<(_cmd);
#define SOCK_EV_MASK2_SET(_pcxt, _cmd)   (_pcxt)->sock_st_mask2 |= 1<<((_cmd) - 32);

#define SOCK_EV_MASK_CLEAR(_pcxt, _cmd)  (_pcxt)->sock_st_mask &= ~(1<<(_cmd));
#define SOCK_EV_MASK2_CLEAR(_pcxt, _cmd) (_pcxt)->sock_st_mask2 &= ~(1<<((_cmd) - 32));

#define SOCK_EV_MASK_TEST(_pcxt, _cmd)   (((_pcxt)->sock_st_mask  & (1<<(_cmd))) ? 1 : 0)
#define SOCK_EV_MASK2_TEST(_pcxt, _cmd)  (((_pcxt)->sock_st_mask2 & (1<<((_cmd) - 32))) ? 1 : 0)

extern ATH_SOCKET_CONTEXT* ath_sock_context[];



/************************** Internal Function declarations*****************************/
A_STATUS socket_context_init();
A_INT32 find_socket_context(A_UINT32 handle, A_UINT8 retrieve);
A_UINT32 getTransportLength(A_UINT8 proto);
A_UINT32 getIPLength(A_UINT8 version);
A_STATUS move_power_state_to_maxperf(void *pDCxt, A_INT32 module);
A_STATUS restore_power_state(void *pDCxt, A_INT32 module);

/************************* Socket APIs *************************************************/
A_INT32 Api_socket(A_VOID *pCxt, A_UINT32 domain, A_UINT32 type, A_UINT32 protocol);
A_INT32 Api_shutdown(A_VOID *pCxt, A_UINT32 handle);
A_INT32 Api_connect(A_VOID *pCxt, A_UINT32 handle, A_VOID* name, A_UINT16 length);
A_INT32 Api_bind(A_VOID *pCxt, A_UINT32 handle, A_VOID* name,  A_UINT16 length);
A_INT32 Api_listen(A_VOID *pCxt, A_UINT32 handle, A_UINT32 backlog);
A_INT32 Api_accept(A_VOID *pCxt, A_UINT32 handle, A_VOID* name,  A_UINT16 length);
A_INT32 Api_sendto(A_VOID *pCxt, A_UINT32 handle, A_UINT8* buffer, A_UINT32 length, A_UINT32 flags, SOCKADDR_T* name, A_UINT32 socklength);
A_INT32 Api_select(A_VOID *pCxt, A_UINT32 handle, A_UINT32 tv);

#if T_SELECT_VER1
A_INT32 Api_accept_ver1(A_VOID *pCxt, A_UINT32 handle, A_VOID* name,  A_UINT16 length);
A_INT32 Api_select_ver1(A_VOID* pCxt, A_INT32 num, A_UINT32 *r_fd, A_UINT32 *w_fd, A_UINT32 *e_fd, A_UINT32 tv);
#endif

A_INT32 Api_errno(A_VOID *pCxt, A_UINT32 handle);
A_INT32 Api_getsockopt(A_VOID *pCxt, A_UINT32 handle, A_UINT32 level, A_UINT32 optname, A_UINT8* optval, A_UINT32 optlen);
A_INT32 Api_setsockopt(A_VOID *pCxt, A_UINT32 handle, A_UINT32 level, A_UINT32 optname, A_UINT8* optval, A_UINT32 optlen);
#if ZERO_COPY
A_INT32 Api_recvfrom(A_VOID *pCxt, A_UINT32 handle, void** buffer, A_UINT32 length, A_UINT32 flags, A_VOID* name, A_UINT32* socklength);
#else
A_INT32 Api_recvfrom(A_VOID *pCxt, A_UINT32 handle, void* buffer, A_UINT32 length, A_UINT32 flags, A_VOID* name, A_UINT32* socklength);
#endif
A_INT32 Api_ipconfig(A_VOID *pCxt, A_UINT32 mode,A_UINT32* ipv4_addr, A_UINT32* subnetMask, A_UINT32* gateway4,IP46ADDR *dnsaddr,A_CHAR *hostname);
A_VOID clear_socket_context(A_INT32 index);
A_INT32 Api_ping(A_VOID *pCxt, A_UINT32 ipv4_addr, A_UINT32 size);
A_INT32 Api_ping6(A_VOID *pCxt, A_UINT8 *ip6addr, A_UINT32 size);
A_INT32 Api_ip6config(A_VOID *pCxt, A_UINT32 mode,IP6_ADDR_T *v6Global, IP6_ADDR_T *v6Link, IP6_ADDR_T *v6DefGw,IP6_ADDR_T *v6GlobalExtd, 
		      A_INT32 *LinkPrefix, A_INT32 *GlbPrefix, A_INT32 *DefgwPrefix, A_INT32 *GlbPrefixExtd);
A_STATUS send_stack_init(A_VOID* pCxt);		      
A_VOID socket_context_deinit();
A_INT32 Api_ipconfig_dhcp_pool(A_VOID *pCxt,A_UINT32* start_ipv4_addr, A_UINT32* end_ipv4_addr, A_INT32 leasetime);
A_INT32 Api_ip6config_router_prefix(A_VOID *pCxt,A_UINT8 *v6addr,A_INT32 prefixlen,A_INT32 prefix_lifetime,A_INT32 valid_lifetime);
A_INT32 Api_ipconfig_set_tcp_exponential_backoff_retry(A_VOID *pCxt,A_INT32 retry);
A_INT32 Api_ipconfig_set_ip6_status(A_VOID *pCxt,A_UINT16 status);
A_INT32 Api_ipconfig_dhcp_release(A_VOID *pCxt);
A_INT32 Api_ipconfig_set_tcp_rx_buffer(A_VOID *pCxt,A_INT32 rxbuf);
A_INT32 Api_ip_http_server(A_VOID *pCxt,A_INT32 command);
A_INT32 Api_ip_http_server_method(A_VOID *pCxt, A_INT32 command, A_UINT8 *pagename, A_UINT8 *objname, A_INT32 objtype, A_INT32 objlen, A_UINT8 * value);
A_INT32 Api_httpc_method(A_VOID* pCxt, A_UINT32 command, A_UINT8 *url, A_UINT8 *data, A_UINT8 **output);
A_INT32 Api_ip_resolve_host_name(A_VOID *pCxt,DNC_CFG_CMD *DncCfg,DNC_RESP_INFO *DncRespInfo);
A_INT32 Api_ip_dns_client(A_VOID *pCxt,A_INT32 command);
A_INT32 Api_ip_dns_server_addr(A_VOID *pCxt,IP46ADDR *addr);
A_INT32 Api_ip_dns_local_domain(A_VOID *pCxt, A_CHAR *domain_name);
A_INT32 Api_ip_hostname(A_VOID *pCxt, A_CHAR *domain_name);
A_INT32 Api_ipdns(A_VOID *pCxt,A_INT32 command,A_CHAR *domain_name, IP46ADDR *dnsaddr);
A_INT32 Api_ip_sntp_srvr_addr(A_VOID *pCxt,A_INT32 command,A_CHAR *sntp_srvr_addr);
A_INT32 Api_ip_sntp_get_time(A_VOID *pCxt,tSntpTime *SntpTime);
A_INT32 Api_ip_sntp_get_time_of_day(A_VOID *pCxt,tSntpTM *SntpTm);
A_INT32 Api_ip_sntp_modify_zone_dse(A_VOID *pCxt,A_UINT8 hr,A_UINT8 mn,A_UINT8 zone_cal,A_UINT8 dse_en_dis);
A_INT32 Api_ip_sntp_query_srvr_address(A_VOID *pcxt,tSntpDnsAddr SntpDnsAddr[MAX_SNTP_SERVERS]);
A_INT32 Api_ip_sntp_client(A_VOID *pCxt,A_INT32 command);

#if ENABLE_SSL
SSL_CTX* Api_SSL_ctx_new(A_VOID *pCxt, SSL_ROLE_T role, A_INT32 inbufSize, A_INT32 outbufSize, A_INT32 reserved);
A_INT32 Api_SSL_ctx_free(A_VOID *pCxt, SSL_CTX *ctx);
SSL* Api_SSL_new(A_VOID *pCxt, SSL_CTX *ctx);
A_INT32 Api_SSL_set_fd(A_VOID *pCxt, SSL *ssl, A_UINT32 fd);
A_INT32 Api_SSL_accept(A_VOID *pCxt, SSL *ssl);
A_INT32 Api_SSL_connect(A_VOID *pCxt, SSL *ssl);
A_INT32 Api_SSL_shutdown(A_VOID *pCxt, SSL *ssl);
A_INT32 Api_SSL_configure(A_VOID *pCxt, SSL *ssl, SSL_CONFIG *cfg);
A_INT32 Api_SSL_addCert(A_VOID *pCxt, SSL_CTX *ctx, SSL_CERT_TYPE_T type, A_UINT8 *cert, A_UINT32 size);
A_INT32 Api_SSL_storeCert(A_VOID *pCxt, A_CHAR *name, A_UINT8 *cert, A_UINT32 size);
A_INT32 Api_SSL_loadCert(A_VOID *pCxt, SSL_CTX *ctx, SSL_CERT_TYPE_T type, A_CHAR *name);
A_INT32 Api_SSL_listCert(A_VOID *pCxt, SSL_FILE_NAME_LIST *fileNames);
#endif
#endif //ENABLE_STACK_OFFLOAD
#endif 
