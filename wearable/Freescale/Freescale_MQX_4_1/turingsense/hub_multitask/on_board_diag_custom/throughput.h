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

#ifndef _THROUGHPUT_H_
#define _THROUGHPUT_H_

/* Turingsense */
#if TURINGSENSE
#include "atheros_main_loop_dtm.h"
#endif

#define USE_ATH_CHANGES                 1	 // Macro to control Atheros specific changes to FNET tools	  
#define ATH_RECV_TIMEOUT  		(60000)
#define TX_DELAY_INTERVAL 		500
#define TCP_TXTEST_TX_BUF_SIZE          4096	 // TCP Transmit Test TX Buffer
#define TCP_TXTEST_HT_TX_BUF_SIZE       7360	 // High Throughput buffer, only used in case of single stream	
#define TCP_TXTEST_RX_BUF_SIZE          1460	 // TCP Transmit Test RX Buffer
#define TCP_RXTEST_TX_BUF_SIZE          1460	 // TCP Receive Test TX Buffer 
#define TCP_RXTEST_RX_BUF_SIZE            2920	 // TCP Receive Test RX Buffer 	
#define TCP_RXTEST_HT_RX_BUF_SIZE         2920*3 // High Throughput buffer, only used in case of single stream
#define MAX_END_MARK_RETRY                50   // Max number of End Mark retries in UDP Tx test
#define MSEC_HEARTBEAT 			  10000
#define MAX_TASKS_ALLOWED 		  2	 // For now, we allow ony 2 tasks
#define MAX_STREAMS_ALLOWED 		  2	 // Max number of allowed TCP/UDP streams- limited by memory
#define TCP_CONNECTION_WAIT_TIME          50 
#define UDP_CONNECTION_WAIT_TIME          500
#define UDP_CONNECTION_WAIT_TIME_MULTI_SOCK 50
#define MAX_ARGC 		          (10)
#define MAX_STRLEN 			  (64+1)
#define CFG_PORT                     	 (7007)
#define CFG_PACKET_SIZE_MAX_TX           (1576) /* Defines size of Application and Socket TX&RX buffers.*/
#define CFG_PACKET_SIZE_MAX_RX           (1556) /* Defines size of Application and Socket TX&RX buffers.*/
#define CFG_TX_PACKET_SIZE_DEFAULT       (1400)
#define END_OF_TEST_CODE                 (0xAABBCCDD) 
#define MAX_END_MARK_RETRY                50   // Max number of End Mark retries in UDP Tx test
#define TX_DELAY_INTERVAL 		 500
#define CFG_MAX_IPV6_PACKET_SIZE         1220
/* Socket Tx&Rx buffer sizes. */
#define CFG_SOCKET_BUF_SIZE      		(CFG_PACKET_SIZE_MAX_RX)
#define CFG_BUFFER_SIZE         		(CFG_PACKET_SIZE_MAX_RX)
#define ATH_RECV_TIMEOUT  		(60000)
#define TX_DELAY			 5
#define SMALL_TX_DELAY                   1
#define _ip_address                                A_UINT32 
#define MAX_STREAMS_ALLOWED 		 2	 // Max number of allowed TCP/UDP streams- limited by memory
#define MAX_SSL_INST                 1
#define SSL_SERVER_INST              0
#define SSL_CLIENT_INST              0
#define SSL_INBUF_SIZE               4500
#define SSL_OUTBUF_SIZE              2000

/*structure to manage stats received from peer during UDP traffic test*/
typedef struct stat_packet
{
    A_UINT32 bytes;  
    A_UINT32 kbytes;
    A_UINT32 msec;
    A_UINT32 numPackets;
}stat_packet_t;


/**************************************************************************/ /*!
 * TX/RX Test parameters
 ******************************************************************************/
typedef struct transmit_params
{
    _ip_address ip_address;  
    unsigned char v6addr[16];
    unsigned short port;
    int packet_size;
    int tx_time;
    int packet_number;
    int interval;
    int test_mode;
}TX_PARAMS;

typedef struct receive_params
{     
    unsigned short port;
#if ENABLE_STACK_OFFLOAD
    IP_MREQ_T group;
    IPV6_MREQ_T group6;
    unsigned short v6mcastEnabled;    
    int ip_hdr_inc; 
#else
    struct ip_mreq group;
#endif  
}RX_PARAMS;


#if ENABLE_HTTP_CLIENT
typedef struct httpc_params
{
    A_UINT32          command;
    A_UINT8           url[64];
    A_UINT8           data[128];
}HTTPC_PARAMS;

typedef PREPACK struct {
    A_UINT32    length;
    A_UINT32    resp_code;
    A_UINT32    flags;
    A_UINT8     data[1];
} POSTPACK HTTPC_RESPONSE;

#endif /* ENABLE_HTTP_CLIENT */


/************************************************************************
*    Benchmark server control structure.
*************************************************************************/
#if TURINGSENSE
typedef struct ts_throughput_cxt
{
    int sock_local;                /* Listening socket.*/
    int sock_peer;                  /* Foreign socket.*/    
    char* txbuffer;   
    char* rxbuffer;    
    TIME_STRUCT block_time;         //Last time we blocked for other threads
    TIME_STRUCT cur_time;           //Current time
    unsigned long long bytes;		 //Number of bytes received in current test
    unsigned long kbytes;            //Number of kilo bytes received in current test
    unsigned long last_bytes;        //Number of bytes received in the previous test
    unsigned long last_kbytes;
    unsigned long sent_bytes;
    A_UINT32     pkts_recvd;
    A_UINT32     pkts_expctd;
    A_UINT32     last_interval;
    TX_PARAMS    tx_params;
    RX_PARAMS    rx_params;
    A_UINT8      test_type; 
}TS_THROUGHPUT_CXT;
#else
typedef struct throughput_cxt
{
    int sock_local;                /* Listening socket.*/
    int sock_peer;                  /* Foreign socket.*/    
    char* buffer;    
    TIME_STRUCT first_time;         //Test start time
    TIME_STRUCT last_time;          //Current time
    unsigned long long bytes;		 //Number of bytes received in current test
    unsigned long kbytes;            //Number of kilo bytes received in current test
    unsigned long last_bytes;        //Number of bytes received in the previous test
    unsigned long last_kbytes;
    unsigned long sent_bytes;
    A_UINT32     pkts_recvd;
    A_UINT32     pkts_expctd;
    A_UINT32     last_interval;
    TX_PARAMS    tx_params;
    RX_PARAMS    rx_params;
    A_UINT8      test_type; 
}THROUGHPUT_CXT;
#endif /* TURINGSENSE differentiate THROUGHPUT_CXT */

typedef struct end_of_test{
  int code;
  int packet_count;
} EOT_PACKET;



enum test_type
{
	UDP_TX,   //UDP Transmit (Uplink Test)
	UDP_RX,   //UDP Receive (Downlink Test)
	TCP_TX,   //TCP Transmit (Uplink Test)
	TCP_RX,   //TCP Receive (Downlink Test)
	SSL_TX,   //SSL Transmit (Uplink Test)
	SSL_RX    //SSL Receive (Downlink Test)
};


enum Test_Mode
{
	TIME_TEST,
	PACKET_TEST
};

A_INT32 rx_command_parser(A_INT32 argc, char_ptr argv[] );
A_INT32 tx_command_parser(A_INT32 argc, char_ptr argv[] );
#if ENABLE_STACK_OFFLOAD
A_INT32 rx_command_parser_multi_socket(A_INT32 argc, char_ptr argv[] );
A_INT32 tx_command_parser_multi_socket(A_INT32 argc, char_ptr argv[] );
#endif


#if TURINGSENSE
enum 
{
   
   SWITCH_TASK=1,
   SHELL_TASK,
   LOGGING_TASK,
   USB_TASK,
   ALIVE_TASK,
   HUB_MAIN_LOOP_TASK, /* Turingsense */
   ATHEROS_TASK,
   USB_CDC_TASK,
};
#endif

/* From task_manager.c */
#if !TURINGSENSE
void set_hvac_init(uint_8 val);
#endif

int_32 worker_cmd_handler(int_32 argc, char_ptr argv[]);
int_32 worker_cmd_quit(int_32 argc, char_ptr argv[]);
A_INT32 setBenchMode(A_INT32 argc, char_ptr argv[] );
int_32 test_for_quit(void);
#if TURINGSENSE
uint_32 check_test_time(TS_THROUGHPUT_CXT *p_tCxt);
#else
uint_32 check_test_time(THROUGHPUT_CXT *p_tCxt);
#endif /* TURINGSENSE */
int_32 test_for_delay(TIME_STRUCT *pCurr, TIME_STRUCT *pBase);
char *inet_ntoa( A_UINT32 /*struct in_addr*/ addr, char *res_str );
void wmiconfig_Task2(uint_32);
void wmiconfig_Task1(uint_32);
int ishexdigit(char digit);
unsigned int hexnibble(char digit);
unsigned int atoh(char * buf);
int_32 ath_inet_aton
   (
      const char*  name,
         /* [IN] dotted decimal IP address */
      A_UINT32*     ipaddr_ptr
         /* [OUT] binary IP address */
   );
int_32 parse_ipv4_ad(unsigned long * ip_addr,   /* pointer to IP address returned */
   unsigned *  sbits,      /* default subnet bit number */
   char *   stringin);


#if ENABLE_STACK_OFFLOAD

char * print_ip6(IP6_ADDR_T * addr, char * str);
char * inet6_ntoa(char* addr, char * str);
int Inet6Pton(char * src, void * dst);

A_INT32 httpc_command_parser(A_INT32 argc, char_ptr argv[]);


#if ENABLE_SSL
typedef struct ssl_inst
{
    SSL_CTX*     sslCtx;
    SSL*         ssl;
    SSL_CONFIG   config;
    A_UINT8      config_set;
    SSL_ROLE_T   role;
} SSL_INST;

extern SSL_INST ssl_inst[MAX_SSL_INST];
extern A_UINT8 *ssl_cert_data_buf;
extern A_UINT16 ssl_cert_data_buf_len;
extern A_UINT8 const *ssl_default_cert;
extern const A_UINT16 ssl_default_cert_len;
extern A_UINT8 const *ssl_default_calist;
extern const A_UINT16 ssl_default_calist_len;

A_INT32 ssl_get_cert_handler(A_INT32 argc, char* argv[]);
A_INT32 https_server_handler(A_INT32 argc, char* argv[]);
A_INT32 https_client_handler(A_INT32 argc, char* argv[]);

#endif


#endif

#endif
