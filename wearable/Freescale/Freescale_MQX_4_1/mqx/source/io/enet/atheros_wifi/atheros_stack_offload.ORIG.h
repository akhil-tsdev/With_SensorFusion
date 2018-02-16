//------------------------------------------------------------------------------
// <copyright file="atheros_stack_offload.h" company="Atheros">
//    Copyright (c) 2004-2007 Atheros Corporation.  All rights reserved.
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
#ifndef __ATHEROS_STACK_OFFLOAD_H__
#define __ATHEROS_STACK_OFFLOAD_H__
#include "a_config.h"
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

#if ENABLE_STACK_OFFLOAD

/**********************Macros and config parameters**********/
#if ENABLE_P2P_MODE
#define MAX_SOCKETS_SUPPORTED      (2)
#else
#define MAX_SOCKETS_SUPPORTED      (8)
#endif
#define SOCKET_NOT_FOUND           (-1)
#define EMPTY_SOCKET_CONTEXT       (0)
#define SOCKET_HANDLE_PLACEHOLDER  (0xFFFFFFFF)
#define ATH_AF_INET                (2)
#define ATH_PF_INET                (ATH_AF_INET)
#define ATH_AF_INET6               (3)
#define ATH_PF_INET6               (ATH_AF_INET6)
#define IPV4_HEADER_LENGTH		   (20)
#define IPV6_HEADER_LENGTH		   (40)
#define TCP_HEADER_LENGTH          (20)
#define UDP_HEADER_LENGTH          (8)
#define IPV6_FRAGMENTATION_THRESHOLD   (1280)
#define IPV4_FRAGMENTATION_THRESHOLD   (AR4100_BUFFER_SIZE -TCP6_HEADROOM)    
#define MAX_ETHERNET_FRAME_SIZE    (1500)

#define GLOBAL_SOCK_INDEX          (MAX_SOCKETS_SUPPORTED)        //Index used for global commands e.g. ipconfig, ping


/*The following macro enables the zero copy feature. In memory-constrained systems,
  the application will provide a pointer instead of an allocated buffer. The driver 
  will return the pointer to received packet instead of copying the packet over.
  The application must call zero_copy_free() API after it is done with the buffer
  and pass the pointer to buffer.*/
#define ZERO_COPY                     1

/* NON_BLOCKING_TX- Macro used to control SEND behavior. 
   SECTION 1- Macro is disabled.
    If this macro is disabled, application thread will block until a packet is 
    sent over SPI. This is not desirable in a single buffer scenario as it may 
    lead to deadlocks. 
   SECTION 2- Macro is enabled.
   If this macro is enabled, the application thread will return after submitting 
   packet to the driver thread. Under this setting, the application MUST NOT 
   TRY TO REUSE OR FREE THIS BUFFER. This buffer is now owned by the driver. 
   The application should call custom_alloc again to get a new buffer. */
#define NON_BLOCKING_TX            0 


/* There may be scenarios where application does not wish to block on a receive operation.
   This macro will enable non blocking receive behavior. Note that this change is only 
   limited to the host and does not affect target behavior.*/ 
#define NON_BLOCKING_RX            0


/* If the host has only 1 RX buffer, receive operation can never be blocking, as
   this may lead to deadlocks.*/
#if WLAN_CONFIG_NUM_PRE_ALLOC_RX_BUFFERS < 2

#if NON_BLOCKING_RX == 0
#error "Blocking receive not permitted with single RX buffer, please enable NON_BLOCKING_RX"
#endif

#if NON_BLOCKING_TX == 0
#error "Blocking tranmit not permitted with single RX buffer, please enable NON_BLOCKING_TX"
#endif

#endif

/* BSD Sockets errors */
#define ENOBUFS         1
#define ETIMEDOUT       2
#define EISCONN         3
#define EOPNOTSUPP      4
#define ECONNABORTED    5
#define EWOULDBLOCK     6
#define ECONNREFUSED    7
#define ECONNRESET      8
#define ENOTCONN        9
#define EALREADY        10
#define EINVAL          11
#define EMSGSIZE        12
#define EPIPE           13
#define EDESTADDRREQ    14
#define ESHUTDOWN       15
#define ENOPROTOOPT     16
#define EHAVEOOB        17
#define ENOMEM          18
#define EADDRNOTAVAIL   19
#define EADDRINUSE      20
#define EAFNOSUPPORT    21


/* Generic/TCP socket options          */

#define SO_KEEPALIVE       0x0008    /* keep connections alive    */
#define SO_MAXMSG          0x1010    /* get TCP_MSS (max segment size) */
#define SO_LINGER          0x0080    /* linger on close if data present    */

/* TCP User-settable options (used with setsockopt).   */
#define TCP_MAXSEG         0x2003    /* set maximum segment size    */

/*Multicast options*/
#define  IP_MULTICAST_IF   9  /* u_char; set/get IP multicast i/f  */
#define  IP_MULTICAST_TTL  10 /* u_char; set/get IP multicast ttl */
#define  IP_MULTICAST_LOOP 11 /* u_char; set/get IP multicast loopback */
#define  IP_ADD_MEMBERSHIP 12 /* ip_mreq; add an IP group membership */
#define  IP_DROP_MEMBERSHIP 13 /* ip_mreq; drop an IP group membership */
#define  IP_HDRINCL         2 /* int ; header is included with data */ 

/*IPv6*/
#define IPV6_MULTICAST_IF   80 /* unisgned int; set IF for outgoing MC pkts */
#define IPV6_MULTICAST_HOPS 81 /* int; set MC hopcount */
#define IPV6_MULTICAST_LOOP 82 /* unisgned int; set to 1 to loop back */
#define IPV6_JOIN_GROUP     83 /* ipv6_mreq; join MC group */
#define IPV6_LEAVE_GROUP    84 /* ipv6_mreq; leave MC group */
#define IPV6_V6ONLY         85 /* int; IPv6 only on this socket */

/*Driver level socket errors*/
#define A_SOCK_INVALID           -2  /*Socket handle is invalid*/ 

#define CUSTOM_ALLOC(size)  \
	custom_alloc(size)
	
#define CUSTOM_FREE(buf)  \
	custom_free(buf)

#define     ATH_IPPROTO_IP     0  
#define     ATH_IPPROTO_ICMP   1
#define     ATH_IPPROTO_IGMP   2  /* added for IP multicasting changes */
#define     ATH_IPPROTO_TCP    6
#define     ATH_IPPROTO_UDP    17
#define     ATH_IPPROTO_IPV6      41 /* IPv6 header */
#define     ATH_IPPROTO_ROUTING   43 /* IPv6 Routing header */
#define     ATH_IPPROTO_FRAGMENT  44 /* IPv6 fragmentation header */

/* SSL cipher suites. To be used with SSL_configure(). */
#define TLS_NULL_WITH_NULL_NULL                    0x0000
#define TLS_RSA_WITH_NULL_MD5                      0x0001
#define TLS_RSA_WITH_NULL_SHA                      0x0002
#define TLS_RSA_WITH_RC4_128_MD5                   0x0004
#define TLS_RSA_WITH_RC4_128_SHA                   0x0005
#define TLS_RSA_WITH_DES_CBC_SHA                   0x0009
#define TLS_RSA_WITH_3DES_EDE_CBC_SHA              0x000A
#define TLS_DHE_RSA_WITH_DES_CBC_SHA               0x0015
#define TLS_DHE_RSA_WITH_3DES_EDE_CBC_SHA          0x0016
#define TLS_RSA_WITH_AES_128_CBC_SHA               0x002F
#define TLS_DHE_RSA_WITH_AES_128_CBC_SHA           0x0033
#define TLS_RSA_WITH_AES_256_CBC_SHA               0x0035
#define TLS_DHE_RSA_WITH_AES_256_CBC_SHA           0x0039
#define TLS_RSA_WITH_NULL_SHA256                   0x003B
#define TLS_RSA_WITH_AES_128_CBC_SHA256            0x003C
#define TLS_RSA_WITH_AES_256_CBC_SHA256            0x003D
#define TLS_DHE_RSA_WITH_AES_128_CBC_SHA256        0x0067
#define TLS_DHE_RSA_WITH_AES_256_CBC_SHA256        0x006B
#define TLS_PSK_WITH_RC4_128_SHA                   0x008A
#define TLS_PSK_WITH_3DES_EDE_CBC_SHA              0x008B
#define TLS_PSK_WITH_AES_128_CBC_SHA               0x008C
#define TLS_PSK_WITH_AES_256_CBC_SHA               0x008D
#define TLS_RSA_WITH_AES_128_GCM_SHA256            0x009C
#define TLS_RSA_WITH_AES_256_GCM_SHA384            0x009D
#define TLS_DHE_RSA_WITH_AES_128_GCM_SHA256        0x009E
#define TLS_DHE_RSA_WITH_AES_256_GCM_SHA384        0x009F
#define TLS_ECDH_ECDSA_WITH_NULL_SHA               0xC001
#define TLS_ECDH_ECDSA_WITH_RC4_128_SHA            0xC002
#define TLS_ECDH_ECDSA_WITH_3DES_EDE_CBC_SHA       0xC003
#define TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA        0xC004
#define TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA        0xC005
#define TLS_ECDHE_ECDSA_WITH_NULL_SHA              0xC006
#define TLS_ECDHE_ECDSA_WITH_RC4_128_SHA           0xC007
#define TLS_ECDHE_ECDSA_WITH_3DES_EDE_CBC_SHA      0xC008
#define TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA       0xC009
#define TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA       0xC00A
#define TLS_ECDH_RSA_WITH_NULL_SHA                 0xC00B
#define TLS_ECDH_RSA_WITH_RC4_128_SHA              0xC00C
#define TLS_ECDH_RSA_WITH_3DES_EDE_CBC_SHA         0xC00D
#define TLS_ECDH_RSA_WITH_AES_128_CBC_SHA          0xC00E
#define TLS_ECDH_RSA_WITH_AES_256_CBC_SHA          0xC00F
#define TLS_ECDHE_RSA_WITH_NULL_SHA                0xC010
#define TLS_ECDHE_RSA_WITH_RC4_128_SHA             0xC011
#define TLS_ECDHE_RSA_WITH_3DES_EDE_CBC_SHA        0xC012
#define TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA         0xC013
#define TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA         0xC014
#define TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256    0xC023
#define TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA384    0xC024
#define TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA256     0xC025
#define TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA384     0xC026
#define TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256      0xC027
#define TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA384      0xC028
#define TLS_ECDH_RSA_WITH_AES_128_CBC_SHA256       0xC029
#define TLS_ECDH_RSA_WITH_AES_256_CBC_SHA384       0xC02A
#define TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256    0xC02B
#define TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384    0xC02C
#define TLS_ECDH_ECDSA_WITH_AES_128_GCM_SHA256     0xC02D
#define TLS_ECDH_ECDSA_WITH_AES_256_GCM_SHA384     0xC02E
#define TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256      0xC02F
#define TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384      0xC030
#define TLS_ECDH_RSA_WITH_AES_128_GCM_SHA256       0xC031
#define TLS_ECDH_RSA_WITH_AES_256_GCM_SHA384       0xC032
#define TLS_RSA_WITH_AES_128_CCM                   0xC09C
#define TLS_RSA_WITH_AES_256_CCM                   0xC09D
#define TLS_DHE_RSA_WITH_AES_128_CCM               0xC09E
#define TLS_DHE_RSA_WITH_AES_256_CCM               0xC09F
#define TLS_RSA_WITH_AES_128_CCM_8                 0xC0A0
#define TLS_RSA_WITH_AES_256_CCM_8                 0xC0A1
#define TLS_DHE_RSA_WITH_AES_128_CCM_8             0xC0A2
#define TLS_DHE_RSA_WITH_AES_256_CCM_8             0xC0A3

#define SSL_PROTOCOL_UNKNOWN                  0x00
#define SSL_PROTOCOL_SSL_3_0                  0x30
#define SSL_PROTOCOL_TLS_1_0                  0x31
#define SSL_PROTOCOL_TLS_1_1                  0x32
#define SSL_PROTOCOL_TLS_1_2                  0x33

/******************************************************************************
 * *****************************     CAUTION     ******************************
 * THESE DATA STRUCTURES ARE USED BY FIRMWARE ALSO. MAKE SURE THAT BOTH ARE IN
 * SYNCH WHEN YOU MODIFY THESE.
 ******************************************************************************/

typedef PREPACK struct ip6_addr {
   A_UINT8   addr[16];   /* 128 bit IPv6 address */
}POSTPACK IP6_ADDR_T;

typedef PREPACK struct sockaddr_4 { ///E.Y. add _4
	A_UINT16 sin_port FIELD_PACKED;		//Port number
	A_UINT16 sin_family FIELD_PACKED;	//ATH_AF_INET
	A_UINT32 sin_addr FIELD_PACKED;		//IPv4 Address
} POSTPACK SOCKADDR_T;


typedef PREPACK struct sockaddr_6
{
   A_UINT16        sin6_family FIELD_PACKED;   // ATH_AF_INET6 
   A_UINT16        sin6_port FIELD_PACKED;     // transport layer port #
   A_UINT32        sin6_flowinfo FIELD_PACKED; // IPv6 flow information 
   IP6_ADDR_T     sin6_addr FIELD_PACKED;      // IPv6 address 
   A_UINT32        sin6_scope_id FIELD_PACKED; // set of interfaces for a scope 
}POSTPACK SOCKADDR_6_T;

typedef PREPACK struct _ip_mreq
{
   A_UINT32 imr_multiaddr FIELD_PACKED;     //Multicast group address
   A_UINT32 imr_interface FIELD_PACKED;     //Interface address
}POSTPACK IP_MREQ_T;

#if 0
typedef PREPACK struct ipv6_mreq {
   IP6_ADDR_T ipv6mr_multiaddr FIELD_PACKED; /* IPv6 multicast addr */
   IP6_ADDR_T ipv6mr_interface FIELD_PACKED; /* IPv6 interface address */
}POSTPACK IPV6_MREQ_T;
#endif
typedef ipv6_mreq IPV6_MREQ_T;

enum SOCKET_TYPE 
{
	SOCK_STREAM_TYPE=1,    /* TCP */
	SOCK_DGRAM_TYPE,       /* UDP */
	SOCK_RAW_TYPE          /* IP */
};

enum IPCONFIG_MODE
{
    IPCFG_QUERY=0,          /*Retrieve IP parameters*/
    IPCFG_STATIC,           /*Set static IP parameters*/
    IPCFG_DHCP              /*Run DHCP client*/
};

enum DNSC_CMDS
{

GETHOSTBYNAME=0,
GETHOSTBYNAME2,
RESOLVEHOSTNAME
};

#define  MAX_DNSADDRS 3
#define  MAX_SNTP_SERVERS 2
#define DNS_NAME_NOT_SPECIFIED 0

typedef PREPACK struct dnccfgcmd {
    WMI_SOCKET_CMD wmi_cmd FIELD_PACKED;
    A_UINT32        mode FIELD_PACKED;			//0-gethostbyname, 1-gethostbyname2, 2-resolvehostname 
    A_UINT32        domain FIELD_PACKED;     // AF_INET ,AF_INET6
    A_UINT8  ahostname[36];  FIELD_PACKED
}POSTPACK DNC_CFG_CMD;



typedef PREPACK struct dncrespinfo {
 
    char           dns_names[64] FIELD_PACKED; /* Buffer of names (usually w/domain) */	
    A_INT32        h_length; FIELD_PACKED /* Length of the address */
    A_INT32        h_addrtype FIELD_PACKED;	//host address type
    A_INT32        ipaddrs FIELD_PACKED; // count of entries in ipaddr_list
    A_UINT32       ipaddrs_list[MAX_DNSADDRS]  FIELD_PACKED;  // list of ipv4 address
    A_INT32        ip6addrs FIELD_PACKED;  //count of ip6 entries in ip6addr_list
    IP6_ADDR_T     ip6addrs_list[MAX_DNSADDRS] FIELD_PACKED; //list of ip6 address

}POSTPACK DNC_RESP_INFO;     

typedef PREPACK struct ip46addr {

    A_UINT8 type  FIELD_PACKED;
    A_UINT8 au1Rsvd[3] FIELD_PACKED; 
    A_UINT32 addr4 FIELD_PACKED;
    IP6_ADDR_T addr6 FIELD_PACKED;
}POSTPACK IP46ADDR;    


typedef  struct sntp_time 
{
   A_INT32  Sec FIELD_PACKED;
   A_INT32  min FIELD_PACKED;
   A_INT32  hour FIELD_PACKED;
   A_INT32  mon FIELD_PACKED;
   A_INT32  year FIELD_PACKED;
   A_INT32  wday FIELD_PACKED;
   A_INT32  yday FIELD_PACKED;
}POSTPACK tSntpTime;

typedef PREPACK struct sntp_tm
{
   A_UINT32  tv_sec FIELD_PACKED;     /* seconds */
   A_UINT32  tv_usec FIELD_PACKED;    /* and microseconds */
}POSTPACK tSntpTM;

typedef struct sntp_dns_addr
{
  A_UINT8 addr[68] FIELD_PACKED;
  A_UINT8 resolve FIELD_PACKED;
}POSTPACK tSntpDnsAddr;

enum SslErrors 
{
    ESSL_OK =              0, // success
    ESSL_INVAL =          -1, // Invalid argument
    ESSL_NOSOCKET =       -2, // No more SSL socket descriptors available
    ESSL_HSNOTDONE =      -3, // Handshake not done
    ESSL_HSDONE =         -4, // Handshake already done
    ESSL_NOMEM =          -5, // Out of memory
    ESSL_CONN =           -6, // SharkSslCon_Error
    ESSL_CERT =           -7, // SharkSslCon_CertificateError
    ESSL_ALERTRECV =      -8, // SharkSslCon_AlertRecv
    ESSL_ALERTFATAL =     -9, // SharkSslCon_AlertSend FATAL received. Connection must be closed.
    ESSL_TIMEOUT =       -10, // Timeout during handshake
    ESSL_OOPS =          -29, // Oops (something is terribly wrong)
    ESSL_OK_HANDSHAKE =  -32, // handshake complete (internal reason code, not an error)

    // Following TRUST reason codes are returned by sslValidate()

    /** The peer's SSL certificate is trusted, CN matches the host name, time is valid */
    ESSL_TRUST_CertCnTime = -32, // Same as ESSL_OK_HANDSHAKE
    /** The peer's SSL certificate is trusted, CN matches the host name, time is expired */
    ESSL_TRUST_CertCn =     -33, // name    OK, time NOTOK
    /** The peer's SSL certificate is trusted, CN does NOT match the host name, time is valid */
    ESSL_TRUST_CertTime =   -34, // name NOTOK, time    OK
    /** The peer's SSL certificate is trusted, CN does NOT match host name, time is expired */
    ESSL_TRUST_Cert =       -35, // name NOTOK, time NOTOK
    /** The peer's SSL certificate is NOT trusted */
    ESSL_TRUST_None =       -36,
};

typedef void SSL_CTX;
typedef void SSL;
typedef const A_UINT8 *SslCert;
typedef const A_UINT8 *SslCAList;

typedef enum 
{
  SSL_SERVER = 1,
  SSL_CLIENT = 2
} SSL_ROLE_T;

typedef enum
{
  SSL_CERTIFICATE = 1,
  SSL_CA_LIST = 2
} SSL_CERT_TYPE_T;

typedef PREPACK struct sslVerifyPolicy {
    /** True to verify certificate commonName against peer's domain name */
    A_UINT8 domain FIELD_PACKED;
    /** True to verify certificate time validity */
    A_UINT8 timeValidity FIELD_PACKED;
    /** True to immediately send a fatal alert on detection of untrusted certificate */
    A_UINT8 sendAlert FIELD_PACKED;
    /** Reserved */
    A_UINT8 reserved FIELD_PACKED;
} POSTPACK SSL_VERIFY_POLICY;

#define SSL_CIPHERSUITE_LIST_DEPTH 8
typedef PREPACK struct SSL_config {
    A_UINT16 protocol FIELD_PACKED;
    A_UINT16 cipher[SSL_CIPHERSUITE_LIST_DEPTH] FIELD_PACKED;
    SSL_VERIFY_POLICY verify FIELD_PACKED;
    A_CHAR matchName[40] FIELD_PACKED;
} POSTPACK SSL_CONFIG;

typedef PREPACK struct ssl_file_name_list {
    A_UINT8  name[10][20] FIELD_PACKED; // The file names of the certificates or CA lists
}POSTPACK SSL_FILE_NAME_LIST;



#if ZERO_COPY
A_VOID zero_copy_free(A_VOID* buffer);
#endif

A_INT32 t_socket(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 domain, A_UINT32 type, A_UINT32 protocol);
A_INT32 t_shutdown(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle);
A_INT32 t_connect(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_VOID* name, A_UINT16 length);
A_INT32 t_bind(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_VOID* name, A_UINT16 length);
A_INT32 t_listen(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_UINT32 backlog);
A_INT32 t_accept(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_VOID* name, A_UINT16 length);

#if T_SELECT_VER1
A_INT32 t_accept_nb(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_VOID* name, A_UINT16 length);
#endif
A_INT32 t_sendto(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, uint_8* buffer, A_UINT32 length, A_UINT32 flags, A_VOID* name, A_UINT32 socklength);
A_INT32 t_send(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, uint_8* buffer, A_UINT32 length, A_UINT32 flags);

A_INT32 t_setsockopt(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_UINT32 level, A_UINT32 optname, uint_8* optval, A_UINT32 optlen);
A_INT32 t_getsockopt(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_UINT32 level, A_UINT32 optname, uint_8* optval, A_UINT32 optlen);

#if ZERO_COPY
A_INT32 t_recvfrom(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, void** buffer, A_UINT32 length, A_UINT32 flags, A_VOID* name, A_UINT32* socklength);
A_INT32 t_recv(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, void** buffer, A_UINT32 length, A_UINT32 flags);
#else
A_INT32 t_recvfrom(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, void* buffer, A_UINT32 length, A_UINT32 flags, A_VOID* name, A_UINT32* socklength);
A_INT32 t_recv(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, void* buffer, A_UINT32 length, A_UINT32 flags);
#endif
A_INT32 t_select(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle, A_UINT32 tv);

#if T_SELECT_VER1
A_INT32 FD_IsSet(A_UINT32 handle, A_UINT32 mask);
A_INT32 FD_Set(A_UINT32 handle, A_UINT32* mask);
A_INT32 FD_Clr(A_UINT32 handle, A_UINT32* mask);
A_INT32 FD_Zero(A_UINT32* mask);

A_INT32 t_select_ver1(void *a_handle, A_INT32 num, A_UINT32 *r_fd, A_UINT32 *w_fd, A_UINT32 *e_fd, A_UINT32 tv);
#endif //T_SELECT_VER1


A_INT32 t_errno(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 handle);
A_INT32 t_ipconfig(void *handle, A_UINT32 mode,A_UINT32* ipv4_addr, A_UINT32* subnetMask, A_UINT32* gateway4,IP46ADDR *Dnsaddr,A_CHAR *hostname);
A_INT32 t_ping(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 ipv4_addr, A_UINT32 size);
A_INT32 t_ping6(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT8 *ip6addr, A_UINT32 size);
A_INT32 t_ip6config(ENET_CONTEXT_STRUCT_PTR enet_ptr, A_UINT32 mode,IP6_ADDR_T *v6Global,IP6_ADDR_T *v6Local,
		    IP6_ADDR_T *v6DefGw,IP6_ADDR_T *v6GlobalExtd,A_INT32 *LinkPrefix, A_INT32 *GlbPrefix, A_INT32 *DefGwPrefix,A_INT32 *GlbPrefixExtd);
A_INT32  t_ipconfig_dhcp_pool(ENET_CONTEXT_STRUCT_PTR enet_ptr,A_UINT32* start_ipv4_addr, A_UINT32* end_ipv4_addr,A_INT32 leasetime);
A_INT32  t_ip6config_router_prefix(ENET_CONTEXT_STRUCT_PTR enet_ptr,A_UINT8 *v6addr,A_INT32 prefixlen, A_INT32 prefix_lifetime,
		                     A_INT32 valid_lifetime);

A_INT32 custom_ipconfig_set_tcp_exponential_backoff_retry(ENET_CONTEXT_STRUCT_PTR enet_ptr,A_INT32 retry);
A_INT32 custom_ipconfig_set_ip6_status(ENET_CONTEXT_STRUCT_PTR enet_ptr,A_UINT16 status);	    
A_INT32 custom_ipconfig_dhcp_release(ENET_CONTEXT_STRUCT_PTR enet_ptr);
A_INT32 custom_ipconfig_set_tcp_rx_buffer(ENET_CONTEXT_STRUCT_PTR enet_ptr,A_INT32 rxbuf);
A_INT32 custom_ip_http_server(void *handle, A_INT32 command);
A_INT32 custom_ip_http_server_method(void *handle, A_INT32 command, A_UINT8 *pagename, A_UINT8 *objname, A_INT32 objtype, A_INT32 objlen, A_UINT8 * value);

#if ENABLE_DNS_CLIENT
A_INT32 custom_ip_resolve_hostname(void *handle, DNC_CFG_CMD *DncCfg,DNC_RESP_INFO *DncRespInfo);
A_INT32 custom_ip_dns_client(void *handle, A_INT32 command);
A_INT32 custom_ip_dns_server_addr(void *handle, IP46ADDR *addr);
#endif
A_INT32 custom_ip_hostname(void *handle,char *domain_name);
#if ENABLE_DNS_SERVER
A_INT32 custom_ip_dns_local_domain(void *handle,char *domain_name);
A_INT32 custom_ipsetdns(void *handle,A_INT32 command,char *domain_name, IP46ADDR *dnsaddr);
#endif
#if ENABLE_SNTP_CLIENT
A_INT32 custom_ip_sntp_srvr_addr(void *handle,A_INT32 command,char * sntp_srvr_addr);
A_INT32 custom_ip_sntp_get_time(void *handle,tSntpTime *SntpTime);
A_INT32 custom_ip_sntp_get_time_of_day(void *handle,tSntpTM *SntpTm);
A_INT32 custom_ip_sntp_modify_zone_dse(void *handle, A_UINT8 hour,A_UINT8 min,A_UINT8 add_sub,A_UINT8 dse);
A_INT32 custom_ip_sntp_query_srvr_address(void *handle,tSntpDnsAddr SntpDnsAddr[MAX_SNTP_SERVERS]);
A_INT32 custom_ip_sntp_client(void *handle, A_INT32 command);
#endif
#if ENABLE_HTTP_CLIENT
A_INT32 custom_httpc_method(void* a_handle, A_UINT32 command, A_UINT8 *url, A_UINT8 *data, A_UINT8 **output);
#endif /* ENABLE_HTTP_CLIENT */
A_VOID zero_copy_http_free(A_VOID* buffer);


/***************************************************************************************************/

A_VOID* custom_alloc(A_UINT32 size);
A_VOID custom_free(A_VOID* buf);
#if ENABLE_SSL
SSL_CTX* SSL_ctx_new(SSL_ROLE_T role, A_INT32 inbufSize, A_INT32 outbufSize, A_INT32 reserved);
A_INT32 SSL_ctx_free(SSL_CTX *ctx);
SSL* SSL_new(SSL_CTX *ctx);
A_INT32 SSL_setCaList(SSL_CTX *ctx, SslCAList caList, A_UINT32 size);
A_INT32 SSL_addCert(SSL_CTX *ctx, SslCert cert, A_UINT32 size);
A_INT32 SSL_storeCert(A_CHAR *name, SslCert cert, A_UINT32 size);
A_INT32 SSL_loadCert(SSL_CTX *ctx, SSL_CERT_TYPE_T type, char *name);
A_INT32 SSL_listCert(SSL_FILE_NAME_LIST *fileNames);

A_INT32 SSL_set_fd(SSL *ssl, A_UINT32 fd);
A_INT32 SSL_accept(SSL *ssl);
A_INT32 SSL_connect(SSL *ssl);
A_INT32 SSL_shutdown(SSL *ssl);
A_INT32 SSL_configure(SSL *ssl, SSL_CONFIG *cfg);

#if ZERO_COPY
A_INT32 SSL_read(SSL *ssl, void **buf, A_INT32 num);
#else
A_INT32 SSL_read(SSL *ssl, void *buf, A_INT32 num);
#endif
A_INT32 SSL_write(SSL *ssl, const void *buf, A_INT32 num);
#endif
#endif //ENABLE_STACK_OFFLOAD
#endif 
