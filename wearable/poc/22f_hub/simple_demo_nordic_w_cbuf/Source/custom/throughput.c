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

#include "main.h"
#include <string.h>
#include <stdlib.h>
#include "throughput.h"
#include <atheros_stack_offload.h>
#include "atheros_wifi.h"
#include "atheros_wifi_api.h"



#if ENABLE_STACK_OFFLOAD

#define CLIENT_WAIT_TIME     30000

#define OFFSETOF(type, field) ((size_t)(&((type*)0)->field))

#if ENABLE_SSL
/* Include certificate used by benchrx ssl test*/
#include "cert-kingfisher.inc"

/* Include CA list used by benchtx ssl test*/
#include "calist-kingfisher.inc"

#define CERT_HEADER_LEN sizeof(CERT_HEADER_T)

typedef struct
{
    A_CHAR id[4];
    A_UINT32 length;
    A_UINT8 data[0];
} CERT_HEADER_T;

SSL_INST ssl_inst[MAX_SSL_INST];
A_UINT8 *ssl_cert_data_buf;
A_UINT16 ssl_cert_data_buf_len;

A_UINT8 const *ssl_default_cert = (A_UINT8*)sharkSslRSACertKingfisher;
const A_UINT16 ssl_default_cert_len = sizeof(sharkSslRSACertKingfisher);
A_UINT8 const *ssl_default_calist = (A_UINT8*)sharkSslCAList;
const A_UINT16 ssl_default_calist_len = sizeof(sharkSslCAList);

SSL_ROLE_T ssl_role;
#endif // ENABLE_SSL


/**************************Globals *************************************/
static A_UINT32 num_traffic_streams = 0;        //Counter to monitor simultaneous traffic streams
static A_UINT32 port_in_use = 0;		//Used to prevent two streams from using same port

extern unsigned char v6EnableFlag;
A_UINT8 bench_quit= 0;


/*TCP receive timeout*/
const int rx_timeout = ATH_RECV_TIMEOUT;

unsigned char v6EnableFlag =  0;
/************************************************************************
*     Definitions.
*************************************************************************/


#define CFG_COMPLETED_STR    "IOT Throughput Test Completed.\n"
#define CFG_REPORT_STR "."


/*************************** Function Declarations **************************/
static void ath_tcp_tx (THROUGHPUT_CXT *p_tCxt);
static void ath_udp_tx (THROUGHPUT_CXT *p_tCxt);
uint_32 check_test_time(THROUGHPUT_CXT *p_tCxt);
void sendAck(THROUGHPUT_CXT *p_tCxt, A_VOID * address, uint_16 port);
static void ath_udp_rx (THROUGHPUT_CXT *p_tCxt);
static void ath_tcp_rx (THROUGHPUT_CXT *p_tCxt);
#if MULTI_SOCKET_SUPPORT
#define UDP_MULTI_SOCKET_MAX	(MAX_SOCKETS_SUPPORTED)

///Redcucing the number of sockets with multisockets as we are running out of stack space in the demo application
#define TCP_MULTI_SOCKET_MAX	4 //(MAX_SOCKETS_SUPPORTED - 1) 

static int ath_create_socket_and_bind(int *socket, A_INT16 port, A_INT32 type);
static int ath_create_socket_and_bind6(int *socket, A_INT16 port, A_INT32 type);
static void ath_udp_rx_multi_socket (A_INT16 *port, A_INT32 multi_socket_count);
static void ath_tcp_rx_multi_socket (A_INT16 *port, A_INT32 multi_socket_count);
#endif
int_32 test_for_delay(TIME_STRUCT *pCurr, TIME_STRUCT *pBase);
A_INT32 receive_incoming(THROUGHPUT_CXT *p_tCxt, A_UINT32* incoming_connections, A_UINT16* num_connections);
int_32 handle_mcast_param(THROUGHPUT_CXT *p_tCxt);
int_32 wait_for_response(THROUGHPUT_CXT *p_tCxt, SOCKADDR_T foreign_addr,SOCKADDR_6_T foreign_addr6);
extern int Inet6Pton(char * src, void * dst);

#if ENABLE_HTTP_CLIENT
static void httpc_connect(HTTPC_PARAMS *p_httpc);
static void httpc_method(HTTPC_PARAMS *p_httpc);
#endif

extern void app_time_delay(int msec);
extern void app_time_get_elapsed(TIME_STRUCT* time);
extern void app_time_get(TIME_STRUCT* time);

static void ath_tcp_tx_multi_socket (THROUGHPUT_CXT *p_tCxt, A_INT32 multi_socket_count);




/************************************************************************
* NAME: test_for_quit
*
* DESCRIPTION:
* Parameters: none
************************************************************************/
int_32 test_for_quit(void)
{
    return bench_quit;
}


A_INT32 setBenchMode(A_INT32 argc, char_ptr argv[] )
{
	if(argc < 2)
	{
		printf("Missing Parameters, specify v4 or v6\n");
		return A_ERROR;
	}


	if(ATH_STRCMP((char const*)argv[1],"v4")==0)
	{
		v6EnableFlag = 0;
	}
	else if(ATH_STRCMP((char const*)argv[1],"v6")==0)
	{
		v6EnableFlag = 1;
	}
	else
	{
		printf("Invalid parameter, specify v4 or v6\n");
		return A_ERROR;
	}
	return A_OK;
}





/************************************************************************
* NAME: print_test_results
*
* DESCRIPTION: Print throughput results
************************************************************************/
static void print_test_results(THROUGHPUT_CXT *p_tCxt)
{
    /* Print throughput results.*/
    A_UINT32 throughput = 0;
    A_UINT32 total_bytes = 0;
    A_UINT32 total_kbytes = 0;
    A_UINT32 sec_interval = (p_tCxt->last_time.SECONDS - p_tCxt->first_time.SECONDS);
    A_UINT32 ms_interval = (p_tCxt->last_time.MILLISECONDS - p_tCxt->first_time.MILLISECONDS);
    A_UINT32 total_interval = sec_interval*1000 + ms_interval;

    p_tCxt->last_interval = total_interval;

    if(total_interval > 0)
    {
    	/*No test was run, or test is terminated, print results for previous test*/
    	if(p_tCxt->bytes == 0 && p_tCxt->kbytes == 0)
    	{
	    //We get a max throughput of 10MBPS and assuming that we run the
	    //test for 48Hr max and the throughput is 8MBPS, we will get the below calculation
	    //throughput = (X/Y)*8 =>   X = 1000*48*3600 = 172800000 = 0xA4CB800 can fit in a single word 
	    //                          Y = 48*3600 = 172800 = 0x2A300
	    //            =(0xA4CB800/0x2A300)*8 = 8000
    	    throughput = ((p_tCxt->last_kbytes/sec_interval)*8) + (p_tCxt->last_bytes*8)/(total_interval);
    	    total_bytes = p_tCxt->last_bytes;
            total_kbytes =  p_tCxt->last_kbytes;

            /*Reset previous test data*/
            p_tCxt->last_bytes = 0;
            p_tCxt->last_kbytes = 0;
    	}

    	else
    	{
    	    throughput = ((p_tCxt->kbytes/sec_interval)*8) + ((p_tCxt->bytes/total_interval)*8);
    	    total_bytes = p_tCxt->bytes;
            total_kbytes = p_tCxt->kbytes;
    	    p_tCxt->last_bytes = p_tCxt->bytes;
            p_tCxt->last_kbytes = p_tCxt->kbytes;
    	}
    }
    else{
 	throughput = 0;
    }

    switch(p_tCxt->test_type)
    {
    	case UDP_TX:
    	printf("\nResults for %s test:\n\n", "UDP Transmit");
    	break;

    	case UDP_RX:
    	printf("\nResults for %s test:\n\n", "UDP Receive");
    	break;

    	case TCP_TX:
    	printf("\nResults for %s test:\n\n", "TCP Transmit");
    	break;

    	case TCP_RX:
    	printf("\nResults for %s test:\n\n", "TCP Receive");
    	break;

        case SSL_TX:
        printf("\nResults for %s test:\n\n", "SSL Transmit");
        break;

        case SSL_RX:
        printf("\nResults for %s test:\n\n", "SSL Receive");
        break;
    }

    printf("\t%d KBytes %d bytes in %d seconds %d ms  \n\n", total_kbytes, total_bytes,total_interval/1000, total_interval%1000);
   	printf("\t throughput %d kb/sec\n", throughput);
}






/************************************************************************
* NAME: tx_command_parser
*
* DESCRIPTION:
************************************************************************/
A_INT32 tx_command_parser(A_INT32 argc, char_ptr argv[] )
{ /* Body */
    boolean           print_usage=0;
    A_INT32            return_code = A_OK;
    THROUGHPUT_CXT     tCxt;
    A_INT32            retval = -1;
    A_INT32 max_packet_size = 0;

    memset(&tCxt,0,sizeof(THROUGHPUT_CXT));
    if(argc != 8)
    {
        /*Incorrect number of params, exit*/
        return_code = A_ERROR;
        print_usage = TRUE;
    }
    else
    {
        if(v6EnableFlag)
        {
            /*Get IPv6 address of Peer*/
            retval =  Inet6Pton(argv[1], tCxt.tx_params.v6addr); /* server IP*/
            if(retval == 0)
              retval = -1;
        }
        else
        {
            /*Get IPv4 address of Peer*/
            retval = ath_inet_aton(argv[1], (A_UINT32*) &tCxt.tx_params.ip_address);
        }
        if(retval == -1)
        {

            /* Port.*/
            tCxt.tx_params.port = atoi(argv[2]);

            /*Check port validity*/
            if(port_in_use == tCxt.tx_params.port)
            {
                printf("This port is in use, use another Port\n");
                return_code = A_ERROR;
                goto ERROR;
            }

            port_in_use = tCxt.tx_params.port;

            /* Packet size.*/
            tCxt.tx_params.packet_size = atoi(argv[4]);
            if(!v6EnableFlag)
            max_packet_size = CFG_PACKET_SIZE_MAX_TX;
            else
            max_packet_size = 1230;

            if ((tCxt.tx_params.packet_size == 0) || (tCxt.tx_params.packet_size > max_packet_size))
            {
                if(!v6EnableFlag)
                printf("Invalid packet length for v4, allowed %d\n",max_packet_size); /* Print error mesage. */
                else
                printf("Invalid packet length for v6, allowed %d\n",max_packet_size); /* Print error mesage. */

                return_code = A_ERROR;
                print_usage = TRUE;
                goto ERROR;
            }

            /* Test mode.*/
            tCxt.tx_params.test_mode = atoi(argv[5]);

            if(tCxt.tx_params.test_mode != 0 && tCxt.tx_params.test_mode != 1)
            {
                printf("Invalid test mode, please enter 0-time or 1-number of packets\n"); /* Print error mesage. */
                return_code = A_ERROR;
                print_usage = TRUE;
                goto ERROR;
            }

            /* Number of packets OR time period.*/
            if(tCxt.tx_params.test_mode == 0)
            {
                tCxt.tx_params.tx_time = atoi(argv[6]);
            }
            else if (tCxt.tx_params.test_mode == 1)
            {
                tCxt.tx_params.packet_number = atoi(argv[6]);
            }

            /* Inter packet interval for Bandwidth control*/
            tCxt.tx_params.interval = atoi(argv[7]);

            /* TCP */
            if(strcasecmp("tcp", argv[3]) == 0)
            {
              tCxt.test_type = TCP_TX;
              ath_tcp_tx (&tCxt);
            }
            /* UDP */
            else if(strcasecmp("udp", argv[3]) == 0)
            {
               tCxt.test_type = UDP_TX;
               ath_udp_tx (&tCxt);
            }
#if ENABLE_SSL
            else if(strcasecmp("ssl", argv[3]) == 0)
            {
                if (ssl_role)
                {
                    printf("ERROR: busy.\n");
                    return_code = A_ERROR;
                    goto ERROR;
                }
                if (ssl_inst[SSL_CLIENT_INST].sslCtx == NULL || ssl_inst[SSL_CLIENT_INST].role != SSL_CLIENT)
                {
                    printf("ERROR: SSL client not stated (Use 'wmiconfig --ssl_start client' first).\n");
                    return_code = A_ERROR;
                    goto ERROR;
                }

                tCxt.test_type = SSL_TX;
                ssl_role = SSL_CLIENT;
                ath_tcp_tx (&tCxt);
                ssl_role = (SSL_ROLE_T)0;
            }
#endif
            else
            {
                printf("Invalid protocol %s\n", argv[3]);
            }
        }
        else
        {
            printf("Incorrect Server IP address %s\n", argv[1]);   /* Wrong throughput Server IP address. */
            return_code = A_ERROR;
            goto ERROR;
        }
    }
ERROR:
    if (print_usage)
    {
#if ENABLE_SSL
       printf("%s <Rx IP> <port> <tcp|udp|ssl> <msg size> <test mode> <number of packets | time (sec)> <delay in msec> [ssl options] \n", argv[0]);
#else
       printf("%s <Rx IP> <port> <tcp|udp> <msg size> <test mode> <number of packets | time (sec)> <delay in msec> \n", argv[0]);
#endif
       printf("               where-   test mode = 0  - enter time period in sec\n");
       printf("            test mode = 1  - enter number of packets\n");
    }
    port_in_use = 0;
    return return_code;
} /* Endbody */

#if MULTI_SOCKET_SUPPORT
/************************************************************************
* NAME: rx_command_parser_multi_socket
*
* DESCRIPTION: parses parameters of rx command for multi sockets (TCP & UDP)
************************************************************************/

A_INT32 rx_command_parser_multi_socket(A_INT32 argc, char_ptr argv[] )
{
    boolean           print_usage = 0;
    A_INT32            return_code = A_OK;
    //THROUGHPUT_CXT     tCxt1, tCxt2, tCxt3, tCxt4;
    A_INT16 port[UDP_MULTI_SOCKET_MAX];
	A_UINT32 loop = 0, multi_socket_count = 0;
    
    if (!print_usage)
    {
        if (argc < 5)
        {
            printf("Incomplete parameters\n");
            return_code = A_ERROR;
            print_usage = TRUE;
        }
        else
        {
        	if ((strcasecmp("udp", argv[1]) == 0))
        	{
        		multi_socket_count = (atoi(argv[4]) - atoi(argv[2])) + 1;
        		if (multi_socket_count > UDP_MULTI_SOCKET_MAX)
        		{
        			multi_socket_count = 0;
        			printf("Max UDP sockets: %d\n", UDP_MULTI_SOCKET_MAX);
        			return A_ERROR;
        		}
        		for (loop = 0; loop < multi_socket_count; loop++)
        		{
                    port[loop] = atoi(argv[2]) + loop;
        		}
        		ath_udp_rx_multi_socket(port, multi_socket_count);
        	}
        	else if((strcasecmp("tcp", argv[1]) == 0))
        	{
        		multi_socket_count = (atoi(argv[4]) - atoi(argv[2])) + 1;
        		if (multi_socket_count > TCP_MULTI_SOCKET_MAX)
        		{
        			multi_socket_count = 0;
        			printf("Max TCP sockets: %d\n", TCP_MULTI_SOCKET_MAX);
        			return A_ERROR;
        		}
        		for (loop = 0; loop < multi_socket_count; loop++)
        		{
                    port[loop] = atoi(argv[2]) + loop;
        		}
        		ath_tcp_rx_multi_socket(port, multi_socket_count);
        	}
            
        }
    }
    
    if (print_usage)
    {
        
        printf("Usage: %s [udp] <port1> - <port2> or \n", argv[0]);
        printf("Usage: %s [tcp] <port1> - <port2>\n", argv[0]);
        printf("   tcp  = TCP receiver(Max range : %d)\n", TCP_MULTI_SOCKET_MAX);
        printf("   udp  = UDP receiver(Max range : %d)\n", UDP_MULTI_SOCKET_MAX);
        printf("   port1, port2 = receiver port numbers\n");
        
    }
    
    return return_code;
}

#if T_SELECT_VER1
/************************************************************************
* NAME: tx_command_parser_multi_socket
*
* DESCRIPTION:
************************************************************************/
A_INT32 tx_command_parser_multi_socket(A_INT32 argc, char_ptr argv[] )
{ /* Body */
    boolean           print_usage=0;
    A_INT32            return_code = A_OK;
    THROUGHPUT_CXT     tCxt;
    A_INT32            retval = -1;
    A_INT32 max_packet_size = 0;
	A_INT32 range;

    memset(&tCxt,0,sizeof(THROUGHPUT_CXT));
    if(argc != 9)
    {
        /*Incorrect number of params, exit*/
        return_code = A_ERROR;
        print_usage = TRUE;
    }
    else
    {
        if(v6EnableFlag)
        {
            /*Get IPv6 address of Peer*/
            retval =  Inet6Pton(argv[1], tCxt.tx_params.v6addr); /* server IP*/
            if(retval == 0)
              retval = -1;
        }
        else
        {
            /*Get IPv4 address of Peer*/
            retval = ath_inet_aton(argv[1], (A_UINT32*) &tCxt.tx_params.ip_address);
        }
        if(retval == -1)
        {
            /* Port.*/
            tCxt.tx_params.port = atoi(argv[2]);
			
			range = atoi(argv[3]) - tCxt.tx_params.port + 1;
			if(range > TCP_MULTI_SOCKET_MAX)
			{
				printf("Max socket supported:%d\n", TCP_MULTI_SOCKET_MAX);
            	return_code = A_ERROR;
            	goto ERROR;
			}

            /*Check port validity*/
            if(port_in_use == tCxt.tx_params.port)
            {
            printf("This port is in use, use another Port\n");
            return_code = A_ERROR;
            goto ERROR;
            }

            port_in_use = tCxt.tx_params.port;

            /* Packet size.*/
            tCxt.tx_params.packet_size = atoi(argv[5]);
            if(!v6EnableFlag)
            max_packet_size = CFG_PACKET_SIZE_MAX_TX;
            else
            max_packet_size = 1230;

            if ((tCxt.tx_params.packet_size == 0) || (tCxt.tx_params.packet_size > max_packet_size))
            {
                if(!v6EnableFlag)
                printf("Invalid packet length for v4, allowed %d\n",max_packet_size); /* Print error mesage. */
                else
                printf("Invalid packet length for v6, allowed %d\n",max_packet_size); /* Print error mesage. */

                return_code = A_ERROR;
                print_usage = TRUE;
                goto ERROR;
            }

            /* Test mode.*/
            tCxt.tx_params.test_mode = atoi(argv[6]);

            if(tCxt.tx_params.test_mode != 0 && tCxt.tx_params.test_mode != 1)
            {
                printf("Invalid test mode, please enter 0-time or 1-number of packets\n"); /* Print error mesage. */
                return_code = A_ERROR;
                print_usage = TRUE;
                goto ERROR;
            }

            /* Number of packets OR time period.*/
            if(tCxt.tx_params.test_mode == 0)
            {
                tCxt.tx_params.tx_time = atoi(argv[7]);
            }
            else if (tCxt.tx_params.test_mode == 1)
            {
                tCxt.tx_params.packet_number = atoi(argv[7]);
            }

            /* Inter packet interval for Bandwidth control*/
            tCxt.tx_params.interval = atoi(argv[8]);

            /* TCP */
            if(strcasecmp("tcp", argv[4]) == 0)
            {
			  ath_tcp_tx_multi_socket(&tCxt, range);
            }
            else
            {
                printf("Invalid protocol %s\n", argv[4]);
            }
        }
        else
        {
            printf("Incorrect Server IP address %s\n", argv[1]);   /* Wrong throughput Server IP address. */
            return_code = A_ERROR;
            goto ERROR;
        }
    }
ERROR:
    if (print_usage)
    {
       printf("%s <Rx IP> <start port> <end port> <tcp> <msg size> <test mode> <number of packets | time (sec)> <delay in msec>\n", argv[0]);
    }
    port_in_use = 0;
    return return_code;
} /* Endbody */
#endif //T_SELECT_VER1

#endif // MULTI_SOCKET_SUPPORT

/************************************************************************
* NAME: rx_command_parser
*
* DESCRIPTION: parses parameters of receive command (both TCP & UDP)
************************************************************************/
A_INT32 rx_command_parser(A_INT32 argc, char_ptr argv[] )
{ /* Body */
    boolean           print_usage = 0, shorthelp = FALSE;
    A_INT32            return_code = A_OK;
    THROUGHPUT_CXT     tCxt;

    memset(&tCxt, 0, sizeof(THROUGHPUT_CXT));

    if (!print_usage)
    {
        if (argc < 3)
        {
            printf("Incomplete parameters\n");
            return_code = A_ERROR;
            print_usage = TRUE;
        }
        else
        {

            tCxt.rx_params.port = atoi(argv[2]);

            /*Check port validity*/
            if(port_in_use ==  tCxt.rx_params.port)
            {
                printf("This port is in use, use another Port\n");
                return A_ERROR;
            }
            port_in_use = tCxt.rx_params.port;


            if(argc > 3)
            {
                if(argc >= 5)
                {
                    //Multicast address is provided
                    if(!v6EnableFlag)
                    {
                        ath_inet_aton(argv[3], (A_UINT32 *) & tCxt.rx_params.group.imr_multiaddr);
                        ath_inet_aton(argv[4], (A_UINT32 *) & tCxt.rx_params.group.imr_interface);
                    }
                    else
                    {
                        Inet6Pton(argv[3], tCxt.rx_params.group6.ipv6imr_multiaddr.s6_addr16);
                        Inet6Pton(argv[4], (unsigned int *) & tCxt.rx_params.group6.ipv6imr_interface);
                        tCxt.rx_params.v6mcastEnabled = 1;
                    }
                    if(argc == 6)
                    {
                        if(strcasecmp("ip_hdr_inc",argv[5]) == 0)
                        {
                            tCxt.rx_params.ip_hdr_inc = 1;
                        }
                    }
                }
                else if (argc == 4)
                {
                    if(strcasecmp("ip_hdr_inc",argv[3]) == 0)
                    {
                        tCxt.rx_params.ip_hdr_inc = 1;
                    }
                }
                else
                {
                    printf("ERROR: Incomplete Multicast Parameters provided\n");
                    return_code = A_ERROR;
                    goto ERROR;
                }
            }

            if((argc == 1)||(strcasecmp("tcp", argv[1]) == 0))
            {
                tCxt.test_type = TCP_RX;
                ath_tcp_rx (&tCxt);
            }
            else if((argc == 1)||(strcasecmp("udp", argv[1]) == 0))
            {
                tCxt.test_type = UDP_RX;
                ath_udp_rx (&tCxt);
            }
#if ENABLE_SSL
            else if((argc == 1)||(strcasecmp("ssl", argv[1]) == 0))
            {
                if (ssl_role)
                {
                    printf("ERROR: busy.\n");
                    return_code = A_ERROR;
                    goto ERROR;
                }
                if (ssl_inst[SSL_SERVER_INST].sslCtx == NULL || ssl_inst[SSL_SERVER_INST].role != SSL_SERVER)
                {
                    printf("ERROR: SSL server not stated (Use 'wmiconfig --ssl_start server' first).\n");
                    return_code = A_ERROR;
                    goto ERROR;
                }
                tCxt.test_type = SSL_RX;
                ssl_role = SSL_SERVER;
                ath_tcp_rx (&tCxt);
                ssl_role = (SSL_ROLE_T)0;
            }
#endif
            else
            {
                printf("Invalid parameter\n");
            }
        }
    }
ERROR:
    if (print_usage)
    {
        if (shorthelp)
        {
#if ENABLE_SSL
            printf("%s [tcp|udp|ssl] <port>\n", argv[0]);
#else
            printf("%s [tcp|udp] <port>\n", argv[0]);
#endif
        }
        else
        {
#if ENABLE_SSL
            printf("Usage: %s [tcp|udp|ssl] <port> <multicast address*> <local address*>\n", argv[0]);
#else
            printf("Usage: %s [tcp|udp] <port> (multicast address*> <local address*>\n", argv[0]);
#endif
            printf("   tcp  = TCP receiver\n");
            printf("   udp  = UDP receiver\n");
#if ENABLE_SSL
            printf("   ssl  = SSL receiver (TCP + SSL security on top)\n");
#endif
            printf("   port = receiver port\n");
            printf("   multicast address  = IP address of multicast group to join (optional)\n");
            printf("   local address      = IP address of interface (optional)\n");
            printf("   ip_hdr_inc      =    To include IP Header in data (optional))\n");
        }
    }

    port_in_use = 0;

    return return_code;
} /* Endbody */



////////////////////////////////////////////// TX //////////////////////////////////////

/************************************************************************
* NAME: ath_tcp_tx
*
* DESCRIPTION: Start TCP Transmit test.
************************************************************************/
static void ath_tcp_tx (THROUGHPUT_CXT *p_tCxt)
{
    SOCKADDR_T local_addr;
    SOCKADDR_T foreign_addr;
    char ip_str[16];
    SOCKADDR_6_T foreign_addr6;
    char ip6_str[48];
    TIME_STRUCT start_time,current_time,block_time;
    A_UINT32 packet_size = p_tCxt->tx_params.packet_size;
    A_UINT32 cur_packet_number;
    A_UINT32 next_prog_report, prog_report_inc;
    A_UINT32 buffer_offset;
    A_INT32 send_result, result;
    _ip_address temp_addr;
#if ENABLE_SSL
    SSL_INST *ssl = &ssl_inst[SSL_CLIENT_INST];
#endif

    //init quit flag
    bench_quit = 0;
    num_traffic_streams++;

    if(packet_size > CFG_PACKET_SIZE_MAX_TX) /* Check max size.*/
         packet_size = CFG_PACKET_SIZE_MAX_TX;

    /* ------ Start test.----------- */
    printf("\n**********************************************************\n");
#if ENABLE_SSL
    printf("IOT %s TX Test\n", p_tCxt->test_type == SSL_TX ? "SSL" : "TCP");
#else
    printf("IOT TCP TX Test\n" );
#endif
    printf("**********************************************************\n");

    if(!v6EnableFlag)
    {
        temp_addr = LONG_BE_TO_HOST(p_tCxt->tx_params.ip_address);
        memset(ip_str, 0, sizeof(ip_str));
        printf("Remote IP addr. %s\n", inet_ntoa(*(A_UINT32 *)( &temp_addr), ip_str));
    }
    else
    {
        printf("Remote IP addr. %s\n", inet6_ntoa((char *)p_tCxt->tx_params.v6addr, (void *)ip6_str));
    }
    printf("Remote port %d\n", p_tCxt->tx_params.port);
    printf("Message size %d\n", p_tCxt->tx_params.packet_size);
    printf("Number of messages %d\n", p_tCxt->tx_params.packet_number);
    printf("Type benchquit to cancel\n");
    printf("**********************************************************\n");

    if(p_tCxt->tx_params.test_mode == TIME_TEST)
    {
        app_time_get(&start_time);
        prog_report_inc = p_tCxt->tx_params.tx_time/20;

        if(prog_report_inc==0)
            prog_report_inc = 1;

        next_prog_report = start_time.SECONDS + prog_report_inc;
    }else if(p_tCxt->tx_params.test_mode == PACKET_TEST){
        /* generate 20 progress characters across screen to provide progress feedback */
        prog_report_inc = p_tCxt->tx_params.packet_number/20;

        if(prog_report_inc==0)
                prog_report_inc = 1;

        next_prog_report = prog_report_inc;
    }

    /* Create socket */
    if(!v6EnableFlag)
    {
        if((p_tCxt->sock_peer = t_socket((void*)handle, ATH_AF_INET, SOCK_STREAM_TYPE, 0)) == A_ERROR)
        {
            printf("ERROR: Unable to create socket\n");
            goto ERROR_1;
        }
    }
    else
    {
        if((p_tCxt->sock_peer = t_socket((void*)handle, ATH_AF_INET6, SOCK_STREAM_TYPE, 0)) == A_ERROR)
        {
            printf("ERROR: Unable to create socket\n");
            goto ERROR_1;
        }
    }

    /*Allow small delay to allow other thread to run*/
    app_time_delay(TX_DELAY);

    printf("Connecting.\n");

    if(!v6EnableFlag)
    {
      memset(&foreign_addr, 0, sizeof(local_addr));
      foreign_addr.sin_addr = p_tCxt->tx_params.ip_address;
      foreign_addr.sin_port = p_tCxt->tx_params.port;
      foreign_addr.sin_family = ATH_AF_INET;

      /* Connect to the server.*/
      if(t_connect((void*)handle, p_tCxt->sock_peer, (&foreign_addr), sizeof(foreign_addr)) == A_ERROR)
      {
         printf("Connection failed.\n");
         goto ERROR_2;
      }
    }
    else
    {
      memset(&foreign_addr6, 0, sizeof(foreign_addr6));
      memcpy(&foreign_addr6.sin6_addr, p_tCxt->tx_params.v6addr,16);;
      foreign_addr6.sin6_port = p_tCxt->tx_params.port;
      foreign_addr6.sin6_family = ATH_AF_INET6;

      /* Connect to the server.*/
      if(t_connect((void*)handle, p_tCxt->sock_peer, (SOCKADDR_T *)(&foreign_addr6), sizeof(foreign_addr6)) == A_ERROR)
      {
         printf("Connection failed.\n");
         goto ERROR_2;
      }
    }

#if ENABLE_SSL
    if (p_tCxt->test_type == SSL_TX)
    {
        if (ssl->ssl == NULL)
        {
            // Create SSL connection object
            ssl->ssl = SSL_new(ssl->sslCtx);
            if (ssl->ssl == NULL)
            {
                printf("ERROR: Unable to create SSL context\n");
                goto ERROR_2;
            }

            // configure the SSL connection
            if (ssl->config_set)
            {
                result = SSL_configure(ssl->ssl, &ssl->config);
                if (result < A_OK)
                {
                    printf("ERROR: SSL configure failed (%d)\n", result);
                    goto ERROR_2;
                }
            }

        }

        // Add socket handle to SSL connection
        result = SSL_set_fd(ssl->ssl, p_tCxt->sock_peer);
        if (result < 0)
        {
            printf("ERROR: Unable to add socket handle to SSL (%d)\n", result);
            goto ERROR_2;
        }

        // SSL handshake with server
        result = SSL_connect(ssl->ssl);
        if (result < 0)
        {
            if (result == ESSL_TRUST_CertCnTime)
            {
                /** The peer's SSL certificate is trusted, CN matches the host name, time is valid */
                printf("The certificate is trusted\n");
            }
            else if (result == ESSL_TRUST_CertCn)
            {
                /** The peer's SSL certificate is trusted, CN matches the host name, time is expired */
                printf("ERROR: The certificate is expired\n");
                goto ERROR_2;
            }
            else if (result == ESSL_TRUST_CertTime)
            {
                /** The peer's SSL certificate is trusted, CN does NOT match the host name, time is valid */
                printf("ERROR: The certificate is trusted, but the host name is not valid\n");
                goto ERROR_2;
            }
            else if (result == ESSL_TRUST_Cert)
            {
                /** The peer's SSL certificate is trusted, CN does NOT match host name, time is expired */
                printf("ERROR: The certificate is expired and the host name is not valid\n");
                goto ERROR_2;
            }
            else if (result == ESSL_TRUST_None)
            {
                /** The peer's SSL certificate is NOT trusted */
                printf("ERROR: The certificate is NOT trusted\n");
                goto ERROR_2;
            }
            else
		    {
		        printf("ERROR: SSL connect failed (%d)\n", result);
		        goto ERROR_2;
		    }
        }

    }
#endif

        /* Sending.*/
        printf("Sending.\n");

        /*Reset all counters*/
        p_tCxt->bytes = 0;
        p_tCxt->kbytes = 0;
        p_tCxt->last_bytes = 0;
        p_tCxt->last_kbytes = 0;
        cur_packet_number = 0;
        buffer_offset = 0;

        app_time_get_elapsed(&p_tCxt->first_time);
        app_time_get_elapsed(&p_tCxt->last_time);
        block_time = p_tCxt->first_time;

        app_time_delay(20);

        while(1)
        {
            if(test_for_quit()){
                    break;
            }

            while((p_tCxt->buffer = CUSTOM_ALLOC(packet_size)) == NULL)
            {
                //Wait till we get a buffer
                if(test_for_quit()){
                    goto ERROR_2;
                }
                /*Allow small delay to allow other thread to run*/
                app_time_delay(SMALL_TX_DELAY);
            }
           {
              uint8_t data = 0;
              int i;

              p_tCxt->buffer[0] = packet_size & 0xFF;
              p_tCxt->buffer[1] = packet_size >> 8;
              for (i = 2; i < packet_size; ++i) {
                p_tCxt->buffer[i] = data++;
              }
            }

#if ENABLE_SSL
           if (p_tCxt->test_type == SSL_TX)
           {
               send_result = SSL_write(ssl->ssl, p_tCxt->buffer, packet_size);
           }
           else
#endif
            if(!v6EnableFlag)
            {
               send_result = t_send((void*)handle, p_tCxt->sock_peer, (unsigned char*)(p_tCxt->buffer), packet_size, 0);
            }
            else
            {
               send_result = t_send((void*)handle, p_tCxt->sock_peer, (unsigned char*)(p_tCxt->buffer), packet_size, 0);
            }
#if !NON_BLOCKING_TX
            /*Free the buffer only if NON_BLOCKING_TX is not enabled*/
            if(p_tCxt->buffer)
                CUSTOM_FREE(p_tCxt->buffer);
#endif
            app_time_get_elapsed(&p_tCxt->last_time);
#if 0
            if(test_for_delay(&p_tCxt->last_time, &block_time)){
            	/* block to give other tasks an opportunity to run */
            	app_time_delay(TX_DELAY);
            	app_time_get_elapsed(&block_time);
            }
#endif
            /****Bandwidth control- add delay if user has specified it************/
            if(p_tCxt->tx_params.interval)
            	app_time_delay(p_tCxt->tx_params.interval);

            if ( send_result == A_ERROR )
            {
                printf("send packet error = %d\n", send_result);
               // resetTarget();
                goto ERROR_2;
            }
            else if(send_result == A_SOCK_INVALID )
            {
               /*Socket has been closed by target due to some error, gracefully exit*/
               printf("Socket closed unexpectedly\n");
               break;
            }
            else if(send_result)
            {
                p_tCxt->bytes += send_result;
                p_tCxt->sent_bytes += send_result;
                buffer_offset += send_result;

                if(buffer_offset == packet_size)
                {
                    cur_packet_number ++;
                    buffer_offset = 0;
                }

                if(p_tCxt->tx_params.test_mode == PACKET_TEST)
                {
                	/*Test based on number of packets, check if we have reached specified limit*/
                	if(cur_packet_number >= next_prog_report){
                		printf(".");
                		next_prog_report += prog_report_inc;
                	}

	                if((cur_packet_number >= p_tCxt->tx_params.packet_number))
	                {
	                    /* Test completed, print throughput results.*/
	                    break;
	                }
                }
                else if(p_tCxt->tx_params.test_mode == TIME_TEST)
                {
                    /*Test based on time interval, check if we have reached specified limit*/
                    app_time_get(&current_time);

                    if(current_time.SECONDS >= next_prog_report){
                            printf(".");
                            next_prog_report += prog_report_inc;
                    }

                    if(check_test_time(p_tCxt))
                    {
                        /* Test completed, print test results.*/

                        break;
                    }
                }
            }
        }

    ERROR_2:
    	printf("\n"); // new line to separate from progress line
        p_tCxt->kbytes = p_tCxt->bytes/1024;
        p_tCxt->bytes = p_tCxt->bytes % 1024;
    	p_tCxt->test_type = TCP_TX;
    	print_test_results(p_tCxt);

        if(send_result != A_SOCK_INVALID )
          t_shutdown((void*)handle, p_tCxt->sock_peer);

#if ENABLE_SSL
        if (ssl_role == SSL_CLIENT && ssl->ssl != NULL)
        {
            SSL_shutdown(ssl->ssl);
            ssl->ssl = NULL;
        }
#endif

ERROR_1:
    printf(CFG_COMPLETED_STR);
    printf("\n");
    num_traffic_streams--;
}






/************************************************************************
* NAME: ath_udp_tx
*
* DESCRIPTION: Start TX UDP throughput test.
************************************************************************/
static void ath_udp_tx (THROUGHPUT_CXT *p_tCxt)
{
	char ip_str[16];
	TIME_STRUCT start_time,current_time,block_time;
	A_UINT32 next_prog_report, prog_report_inc;
	SOCKADDR_T foreign_addr;
	SOCKADDR_T local_addr;
        SOCKADDR_6_T foreign_addr6;
        SOCKADDR_6_T local_addr6;
        char ip6_str [48];
	A_INT32 send_result,result;
	A_UINT32 packet_size = p_tCxt->tx_params.packet_size;
	A_UINT32 cur_packet_number;
	_ip_address temp_addr;

	bench_quit = 0;

	if(packet_size > CFG_PACKET_SIZE_MAX_TX) /* Check max size.*/
	     packet_size = CFG_PACKET_SIZE_MAX_TX;

 	temp_addr = LONG_BE_TO_HOST(p_tCxt->tx_params.ip_address);


    /* ------ Start test.----------- */
    printf("****************************************************************\n");
    printf(" UDP TX Test\n" );
    printf("****************************************************************\n");
    if(!v6EnableFlag)
    {
      printf( "Remote IP addr. %s\n", inet_ntoa(*(A_UINT32 *)( &temp_addr), ip_str));
    }
    else
    {
      printf( "Remote IP addr. %s\n", inet6_ntoa((char *)p_tCxt->tx_params.v6addr, ip6_str));
    }
    printf("Remote port %d\n", p_tCxt->tx_params.port);
    printf("Message size %d\n", p_tCxt->tx_params.packet_size);
    printf("Number of messages %d\n", p_tCxt->tx_params.packet_number);

    printf("Type benchquit to terminate test\n");
    printf("****************************************************************\n");

      if(p_tCxt->tx_params.test_mode == TIME_TEST)
      {
          app_time_get(&start_time);

          prog_report_inc = p_tCxt->tx_params.tx_time/20;

          if(prog_report_inc==0)
                  prog_report_inc = 1;

          next_prog_report = start_time.SECONDS + prog_report_inc;
      }else if(p_tCxt->tx_params.test_mode == PACKET_TEST){
          /* generate 20 progress characters across screen to provide progress feedback */
          prog_report_inc = p_tCxt->tx_params.packet_number/20;

          if(prog_report_inc==0)
                  prog_report_inc = 1;

          next_prog_report = prog_report_inc;
      }

      /* Create UDP socket */
      if(!v6EnableFlag)
      {
        /* Create IPv4 socket */
        if((p_tCxt->sock_peer = t_socket((void*)handle, ATH_AF_INET, SOCK_DGRAM_TYPE, 0)) == A_ERROR) {
            goto ERROR_1;
        }
      }
      else
      {
         /* Create IPv6 socket */
        if((p_tCxt->sock_peer = t_socket((void*)handle, ATH_AF_INET6, SOCK_DGRAM_TYPE, 0)) == A_ERROR){
            goto ERROR_1;
        }
      }

        /* Bind to the server.*/
        printf("Connecting.\n");
        memset(&foreign_addr, 0, sizeof(local_addr));
        foreign_addr.sin_addr = p_tCxt->tx_params.ip_address;
        foreign_addr.sin_port = p_tCxt->tx_params.port;
        foreign_addr.sin_family = ATH_AF_INET;

        if(test_for_quit()){
        	goto ERROR_2;
        }

        /*Allow small delay to allow other thread to run*/
    	app_time_delay(TX_DELAY);

        if(!v6EnableFlag)
        {
            if(t_connect((void*)handle, p_tCxt->sock_peer, (&foreign_addr), sizeof(foreign_addr)) == A_ERROR)
            {
               printf("Conection failed.\n");
               goto ERROR_2;
            }
        }
        else
        {
             memset(&foreign_addr6, 0, sizeof(local_addr6));
             memcpy(&foreign_addr6.sin6_addr,p_tCxt->tx_params.v6addr,16);
             foreign_addr6.sin6_port = p_tCxt->tx_params.port;
             foreign_addr6.sin6_family = ATH_AF_INET6;
             if(t_connect((void*)handle, p_tCxt->sock_peer, (SOCKADDR_T *)(&foreign_addr6), sizeof(foreign_addr6)) == A_ERROR)
             {
                     printf("Conection failed.\n");
                     goto ERROR_2;
             }
        }

        /* Sending.*/
        printf("Sending.\n");

        /*Reset all counters*/
        p_tCxt->bytes = 0;
        p_tCxt->kbytes = 0;
        p_tCxt->last_bytes = 0;
        p_tCxt->last_kbytes = 0;
        p_tCxt->sent_bytes = 0;
        cur_packet_number = 0;

        app_time_get_elapsed(&p_tCxt->first_time);
        app_time_get_elapsed(&p_tCxt->last_time);
        block_time = p_tCxt->first_time;

        while(1)
        {
          if(test_for_quit()){
                  p_tCxt->test_type = UDP_TX;
                  print_test_results(p_tCxt);
                  break;
          }

          while((p_tCxt->buffer = CUSTOM_ALLOC(packet_size)) == NULL)
          {
              //Wait till we get a buffer
              if(test_for_quit()){
                  p_tCxt->test_type = UDP_TX;
                  print_test_results(p_tCxt);
                  goto ERROR_2;
              }
              /*Allow small delay to allow other thread to run*/
       	      app_time_delay(SMALL_TX_DELAY);
          }

          if(!v6EnableFlag)
          {
              send_result = t_sendto((void*)handle, p_tCxt->sock_peer, (unsigned char*)(&p_tCxt->buffer[0]), packet_size, 0,(&foreign_addr), sizeof(foreign_addr)) ;
          }
          else
          {
              send_result = t_sendto((void*)handle, p_tCxt->sock_peer, (unsigned char*)(&p_tCxt->buffer[0]), packet_size, 0,(&foreign_addr6),
                            sizeof(foreign_addr6)) ;
          }

#if !NON_BLOCKING_TX
          /*Free the buffer only if NON_BLOCKING_TX is not enabled*/
          if(p_tCxt->buffer){
              CUSTOM_FREE(p_tCxt->buffer);
          }
#endif
          app_time_get_elapsed(&p_tCxt->last_time);
#if 0
          if(test_for_delay(&p_tCxt->last_time, &block_time)){
              /* block to give other tasks an opportunity to run */
              app_time_delay(TX_DELAY);
              app_time_get_elapsed(&block_time);
          }
#endif
          /****Bandwidth control***********/
          if(p_tCxt->tx_params.interval)
              app_time_delay(p_tCxt->tx_params.interval);

          if(send_result == A_ERROR)
          {

              //printf("socket_error = %d\n", sock_err);

              p_tCxt->test_type = UDP_TX;

              /* Print throughput results.*/
              print_test_results(p_tCxt);
              break;
          }
          else
          {
              p_tCxt->bytes += send_result;
              p_tCxt->sent_bytes += send_result;
              cur_packet_number ++;

              /*Test mode can be "number of packets" or "fixed time duration"*/
              if(p_tCxt->tx_params.test_mode == PACKET_TEST)
              {
                  if(cur_packet_number >= next_prog_report){
                          printf(".");
                          next_prog_report += prog_report_inc;
                  }

                  if((cur_packet_number >= p_tCxt->tx_params.packet_number))
                  {
                      /*Test completed, print throughput results.*/
                      //print_test_results(p_tCxt);
                      break;
                  }
              }
              else if(p_tCxt->tx_params.test_mode == TIME_TEST)
              {
                  app_time_get(&current_time);
                  if(current_time.SECONDS >= next_prog_report){
                       printf(".");
                       next_prog_report += prog_report_inc;
                  }

                  if(check_test_time(p_tCxt))
                  {
                      /* Test completed, print throughput results.*/
                      //print_test_results(p_tCxt);
                      break;
                  }
              }
          }
        }

        result = wait_for_response(p_tCxt, foreign_addr, foreign_addr6);

        if(result != A_OK){
            printf("UDP Transmit test failed, did not receive Ack from Peer\n");
        }
        else
        {
            p_tCxt->test_type = UDP_TX;
            print_test_results(p_tCxt);
        }

ERROR_2:
        t_shutdown((void*)handle,p_tCxt->sock_peer);

ERROR_1:
    printf("*************IOT Throughput Test Completed **************\n");
}


#if MULTI_SOCKET_SUPPORT
/************************************************************************
* NAME: ath_create_socket_and_bind
*
* DESCRIPTION: Create a IPv4 socket and bind a address.
************************************************************************/
static int ath_create_socket_and_bind(int *socket, A_INT16 port, A_INT32 type)
{
    SOCKADDR_T        local_addr;
    
    if((*socket = t_socket((void *)handle, ATH_AF_INET, type, 0)) == A_ERROR)
    {
        printf("ERROR:: Socket creation error.\r\n");
        return A_ERROR;
    }
    
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_port = port;
    local_addr.sin_family = ATH_AF_INET;
    
    if(t_bind((void*)handle, *socket, (&local_addr), sizeof(local_addr)) != A_OK)
    {
        printf("ERROR:: Socket bind error.\r\n");
        return A_ERROR;
    }
    return A_OK;
}

/************************************************************************
* NAME: ath_create_socket_and_bind
*
* DESCRIPTION: Create a IPv4 socket and bind a address.
************************************************************************/
static int ath_create_socket_and_bind6(int *socket, A_INT16 port, A_INT32 type)
{
    SOCKADDR_6_T      local_addr6;
    
    if((*socket = t_socket((void*)handle, ATH_AF_INET6, type, 0)) == A_ERROR)
    {
        printf("ERROR:: Socket creation error.\r\n");
        return A_ERROR;
    }
    
    memset(&local_addr6, 0, sizeof(local_addr6));
    local_addr6.sin6_port = port;
    local_addr6.sin6_family = ATH_AF_INET6;
    
    if(t_bind((void*)handle, *socket, (&local_addr6), sizeof(local_addr6)) != A_OK)
    {
        printf("ERROR:: Socket bind error.\r\n");
        return A_ERROR;
    }
    
    return A_OK;
}

/************************************************************************
* NAME: ath_udp_rx_multi_socket
*
* DESCRIPTION: Start throughput UDP server.
************************************************************************/
static void ath_udp_rx_multi_socket (A_INT16 *port, A_INT32 multi_socket_count)
{
    A_UINT32          addr_len;
    A_INT32 	      received;
    A_INT32 	      conn_sock;
    A_INT32 	      is_first;
    SOCKADDR_T        addr;
    SOCKADDR_6_T      addr6;
    A_UINT8           *ip_str = NULL ;
    A_UINT32 	      temp_addr;
    TIME_STRUCT       block_time;
    A_UINT32          loop;
    A_UINT32          error;
    A_INT32           sock_local[UDP_MULTI_SOCKET_MAX];
    THROUGHPUT_CXT    tCxt;
    
    bench_quit = 0;
    //Parse remaining parameters
#if !ZERO_COPY
    if((tCxt.buffer = A_MALLOC(CFG_PACKET_SIZE_MAX_RX,MALLOC_ID_CONTEXT)) == NULL)
    {
        printf("Out of memory error\n");
        goto ERROR;
    }
#endif
    
    
    for (loop = 0; loop < multi_socket_count; loop++)
    {
        sock_local[loop] = 0;
        if(!v6EnableFlag)
        {
            error = ath_create_socket_and_bind(&(sock_local[loop]), port[loop], SOCK_DGRAM_TYPE);
        }
        else
        {
            error = ath_create_socket_and_bind6(&(sock_local[loop]), port[loop], SOCK_DGRAM_TYPE);
        }
        
        if (error == A_ERROR)
        {
            goto ERROR;
        }
    }
    
    /* ------ Start test.----------- */
    printf("****************************************************\n");
    printf(" UDP RX Test\n" );
    printf("****************************************************\n");
    
    printf("Local ports ");
    for (loop = 0; loop < multi_socket_count; loop++)
        printf("%d ",port[loop]);
    printf("\n");
    printf("Type benchquit to terminate test\n");
    printf("****************************************************\n");
    
    /*Initialize start, stop times*/
    app_time_get_elapsed(&(tCxt.last_time));
    app_time_get_elapsed(&(tCxt.first_time));
    tCxt.last_bytes = 0;
    tCxt.last_kbytes = 0;
    block_time = tCxt.first_time;
    
    while(test_for_quit()==0) /* Main loop */
    {
        //printf("Waiting.\n");
        for (loop = 0; loop < multi_socket_count; loop++)
        {
            if((conn_sock  =  t_select((void*)handle, sock_local[loop],UDP_CONNECTION_WAIT_TIME)) == A_OK)
            {
                app_time_get_elapsed(&(tCxt.last_time));
                app_time_get_elapsed(&(tCxt.first_time));
                tCxt.last_bytes = 0;
                tCxt.last_kbytes = 0;
                block_time = tCxt.first_time;
                tCxt.bytes = 0;
                tCxt.kbytes = 0;
                tCxt.sent_bytes = 0;
                tCxt.sock_local = sock_local[loop];
                is_first = 1;
                
     	        while(test_for_quit()==0)
     	        {
#if ZERO_COPY
                    if(!v6EnableFlag)
                    {
                        addr_len = sizeof(SOCKADDR_T);
                        received = t_recvfrom((void*)handle, sock_local[loop],
                                              (void**)(&tCxt.buffer),
                                              CFG_PACKET_SIZE_MAX_RX, 0,
                                              &addr, (A_UINT32*)&addr_len);
                    }
                    else
                    {
                        addr_len = sizeof(SOCKADDR_6_T);
                        received = t_recvfrom((void*)handle, sock_local[loop],
                                              (void**)(&tCxt.buffer),
                                              CFG_PACKET_SIZE_MAX_RX, 0,
                                              &addr6, (A_UINT32*)&addr_len);
                    }
                    if(received > 0)
                    {
              	        zero_copy_free(tCxt.buffer);
                    }
#else
                    if(!v6EnableFlag)
                    {
                        received = t_recvfrom((void*)handle, sock_local[loop],
                                              
                                              (char*)(&tCxt.buffer[0]),
                                              CFG_PACKET_SIZE_MAX_RX, 0,
                                              &addr, (A_UINT32*)&addr_len);
                    }
                    else
                    {
                        addr_len = sizeof(SOCKADDR_6_T);
                        received = t_recvfrom((void*)handle, sock_local[loop],
                                              (char*)(&tCxt.buffer[0]),
                                              CFG_PACKET_SIZE_MAX_RX, 0,
                                              &addr6, (A_UINT32*)&addr_len);
                    }
#endif
                    
                    if(received >= sizeof(EOT_PACKET))
                    {
                        /* Reset timeout. */
                        if( received > sizeof(EOT_PACKET) ){
                            _sched_yield();
                            app_time_get_elapsed(&tCxt.last_time);
                        }
                        if(is_first)
                        {
                            if( received > sizeof(EOT_PACKET) )
                            {
                            	temp_addr = LONG_BE_TO_HOST(addr.sin_addr);
                            	ip_str = A_MALLOC(sizeof(A_UINT8) * 48, MALLOC_ID_TEMPORARY);
                            	if (NULL == ip_str)
                            	{
                                    printf("Out of Memory error\n");
                            	    goto ERROR;
                            	}
                                
                                if(!v6EnableFlag)
                                {
                                    printf("Receiving from %s:%d\n", inet_ntoa(*(A_UINT32 *)(&temp_addr), (char *)ip_str), addr.sin_port);
                                }
                                else
                                {
                                    printf("Receiving from %s:%d\n", inet6_ntoa((char *)&addr6.sin6_addr, (char *)ip_str), addr6.sin6_port);
                                }
                                
                                if (NULL != ip_str)
                                    A_FREE(ip_str, MALLOC_ID_TEMPORARY);
                                
                                app_time_get_elapsed(&tCxt.first_time);
                                is_first = 0;
                            }
                        }
                        else /* if(is_first) */
                        {
                            tCxt.bytes += received;
                            tCxt.sent_bytes += received;
                            if(test_for_delay(&tCxt.last_time, &block_time)){
                                /* block to give other tasks an opportunity to run */
                                app_time_delay(SMALL_TX_DELAY);
                                app_time_get_elapsed(&block_time);
                            }
                            if(received == sizeof(EOT_PACKET) ) /* End of transfer. */
                            {
                                /* Send throughput results to Peer so that it can display correct results*/
                                if(!v6EnableFlag)
                                {
                                    sendAck( &tCxt, (A_VOID*)&addr, port[loop]);
                                }
                                else
                                {
                                    sendAck( &tCxt, (A_VOID*)&addr6, port[loop]);
                                }
                                
                                tCxt.kbytes = tCxt.bytes/1024;
                                tCxt.bytes = tCxt.bytes % 1024;
                                tCxt.test_type = UDP_RX;
                                print_test_results (&tCxt);
                                break;
                                
                            } /* End of transfer. */
                        } /* if(is_first) */
                    } /* if(received >= sizeof(EOT_PACKET)) */
                } /* while(test_for_quit()==0) */
            } /* if((conn_sock  =  t_select((void*)handle, sock_local[loop],UDP_CONNECTION_WAIT_TIME_MULTI_SOCK)) == A_OK) */
        } /* for (loop = 0; loop < multi_socket_count; loop++) */
    } /* Main loop */
    
ERROR:
	//t_shutdown((void*)handle,p_tCxt->sock_local);
    for(loop = 0; loop < multi_socket_count; loop++)
    {
        if((0 != sock_local[loop]) && (A_ERROR != sock_local[loop]))
        {
            t_shutdown((void*)handle, sock_local[loop]);
        }
    }
    
    printf(CFG_COMPLETED_STR);
    printf("shell> ");
    
#if !ZERO_COPY
    if(tCxt.buffer)
        A_FREE(tCxt.buffer, MALLOC_ID_CONTEXT);
#endif
    //num_traffic_streams = 0;
    return;
}

/************************************************************************
* NAME: ath_tcp_rx_multi_socket
*
* DESCRIPTION: Start throughput TCP server.
************************************************************************/

#if T_SELECT_VER1


static void ath_tcp_rx_multi_socket (A_INT16 *port, A_INT32 multi_socket_count)
{
    A_UINT8 *ip_str = NULL;
    A_INT32 result, mask;
    A_UINT16 addr_len=0;
    A_INT32 received =0, loop = 0, error = A_OK;
    A_INT32 conn_sock;
    A_UINT8 is_first[TCP_MULTI_SOCKET_MAX], count;
    A_UINT32 r_fd, r_fd_working_set ;
    _ip_address temp_addr;
    typedef union  {
	SOCKADDR_T foreign_addr;
	SOCKADDR_6_T foreign_addr6;
    } SockAddr_t;
    SockAddr_t u_addr;
    TIME_STRUCT      block_time;
    THROUGHPUT_CXT    tCxt[TCP_MULTI_SOCKET_MAX];
    
    
    bench_quit = 0;
    for(loop=0;loop<multi_socket_count;loop++)
    {
	tCxt[loop].sock_peer = 0;
    }
    
#if !ZERO_COPY
    if((tCxt.buffer = A_MALLOC(CFG_PACKET_SIZE_MAX_RX,MALLOC_ID_CONTEXT)) == NULL)
    {
        printf("Out of memory error\n");
        goto ERROR;
    }
#endif
    
    for (loop = 0; loop < multi_socket_count; loop++)
    {
        if(!v6EnableFlag)
        {
            error = ath_create_socket_and_bind(&(tCxt[loop].sock_local), port[loop], SOCK_STREAM_TYPE);
        }
        else
        {
            error = ath_create_socket_and_bind6(&(tCxt[loop].sock_local), port[loop], SOCK_STREAM_TYPE);
        }
        
        if (error == A_ERROR)
        {
            goto ERROR;
        }
    }
    
    /* ------ Start test.----------- */
    printf("****************************************************\n");
    printf(" TCP RX Test\n" );
    printf("****************************************************\n");
    
    printf( "Local ports ");
    for (loop = 0; loop < multi_socket_count; loop++)
    	printf("%d ",port[loop]);
    printf("\n");
    printf("Type benchquit to terminate test\n");
    printf("****************************************************\n");
    
    for (loop=0;loop<multi_socket_count;loop++)
    {
	app_time_get_elapsed(&tCxt[loop].first_time);
	app_time_get_elapsed(&tCxt[loop].last_time);
	tCxt[loop].last_bytes = 0;
	tCxt[loop].last_kbytes = 0;
	is_first[loop] = 1;
    }
    
    
    for (loop = 0; loop < multi_socket_count; loop++)
    {
        if(t_listen((void*)handle, tCxt[loop].sock_local, 1) == A_ERROR)
        {
            printf("ERROR: Socket listen error.\r\n");
            goto ERROR;
        }
    }

    //Form the r_fd mask
    r_fd = 0;
    for(loop=0; loop<multi_socket_count; loop++)
    {
	FD_Set(tCxt[loop].sock_local, &r_fd);
    }
    count = multi_socket_count;
    while(test_for_quit() == 0)
    {
	r_fd_working_set = r_fd;
	    if( (result = (t_select_ver1((void*)handle, count, &r_fd_working_set, NULL, NULL, UDP_CONNECTION_WAIT_TIME))) > 0 )
            {
		for(loop=0; loop<multi_socket_count; loop++)
		{
		    //Data reception here
		    if((tCxt[loop].sock_peer) && FD_IsSet(tCxt[loop].sock_peer, r_fd_working_set))
		    {
#if ZERO_COPY
			received = t_recv((void*)handle, tCxt[loop].sock_peer, (char**)(&(tCxt[loop].buffer)), CFG_PACKET_SIZE_MAX_RX, 0);
			if(received > 0)
			    zero_copy_free(tCxt[loop].buffer);
#else
			received = t_recv((void*)handle, tCxt[loop].sock_peer, (char*)(&tCxt[loop].buffer[0]), CFG_PACKET_SIZE_MAX_RX, 0);
#endif
			if(received >0)
			{
			    /*Valid packet received*/
			    if(is_first[loop])
			    {
				/*This is the first packet, set initial time used to calculate throughput*/
				app_time_get_elapsed(&(tCxt[loop].first_time));
				is_first[loop] = 0;
			    }
			    app_time_get_elapsed(&(tCxt[loop].last_time));
			    tCxt[loop].bytes += received;
			    tCxt[loop].sent_bytes += received;
			} 
			else //either the socket is closed or we got some error
			{
			    //printf("result== A_SOCK_INVALID\n");
			    FD_Clr(tCxt[loop].sock_peer, &r_fd);
			    //Close the socket. 
			    tCxt[loop].sock_peer = 0;
			    /*Test ended, peer closed connection*/
			    tCxt[loop].kbytes = tCxt[loop].bytes/1024;
			    tCxt[loop].bytes = tCxt[loop].bytes % 1024;
			    tCxt[loop].test_type = TCP_RX;
			    /* Print test results.*/
			    print_test_results (&(tCxt[loop]));
			    if(r_fd == 0)
			    {
				//printf("Test is over\n");
				bench_quit = 1;
				break;
			    }
			}
		    }

		    //listener sockets here 
		    if((tCxt[loop].sock_local) && (FD_IsSet(tCxt[loop].sock_local, r_fd_working_set)))
		    {
			tCxt[loop].last_bytes = 0;
			tCxt[loop].last_kbytes = 0;
			tCxt[loop].bytes = 0;
			tCxt[loop].kbytes = 0;
			tCxt[loop].sent_bytes = 0;
			//tCxt[loop].sock_local = sock_local[loop];
                
			/*Accept incoming connection*/
			//printf("Doing an accept here\n");
			if(!v6EnableFlag)
			{
			    addr_len = sizeof(u_addr.foreign_addr);
			    tCxt[loop].sock_peer = t_accept_nb((void*)handle, tCxt[loop].sock_local, &(u_addr.foreign_addr), addr_len);
			    if(tCxt[loop].sock_peer == A_OK)
			    {
				//printf("Accept returned A_OK\n");
			    }
			}
			else
			{
			    addr_len = sizeof(u_addr.foreign_addr6);
			    tCxt[loop].sock_peer = t_accept_nb((void*)handle, tCxt[loop].sock_local, &(u_addr.foreign_addr6), addr_len);
			    if(tCxt[loop].sock_peer == A_OK)
			    {
				//printf("Accept returned A_OK\n");
			    }
			}
			//if a valid socket is found
			if((tCxt[loop].sock_peer != A_ERROR) && (tCxt[loop].sock_peer != A_OK))
			{
			    //printf("new socket 0x%x\n", tCxt[loop].sock_peer);
			    ip_str = A_MALLOC(sizeof(A_UINT8) * 48, MALLOC_ID_TEMPORARY);
			    if (NULL == ip_str)
			    {
				printf("Out of Memory error\n");
				goto ERROR;
			    }
			    if(!v6EnableFlag)
			    {
				temp_addr = LONG_BE_TO_HOST(u_addr.foreign_addr.sin_addr);
				printf("Receiving from %s:%d, port:%d\n", inet_ntoa(*(A_UINT32 *)(&temp_addr), (char *)ip_str), u_addr.foreign_addr.sin_port, port[loop]);
			    }
			    else
			    {
				printf("Receiving from %s:%d, port:%d\n", inet6_ntoa((char *)&(u_addr.foreign_addr6.sin6_addr),(char *)ip_str), u_addr.foreign_addr6.sin6_port, port[loop]);
			    }
			    if (NULL != ip_str)
			    {
				A_FREE(ip_str, MALLOC_ID_TEMPORARY);
				ip_str = NULL;
			    }
			    FD_Set(tCxt[loop].sock_peer, &r_fd);
				FD_Clr(tCxt[loop].sock_local, &r_fd);
				printf("Shutting down socket 0x%x\n", tCxt[loop].sock_local);
				t_shutdown(handle, tCxt[loop].sock_local);
				tCxt[loop].sock_local = 0;
			}
			else if (tCxt[loop].sock_peer == A_ERROR) 
			{
			    printf("Error in t_accept for socket 0x%x\n", tCxt[loop].sock_local);
			    tCxt[loop].sock_peer  = 0;
			    FD_Clr(tCxt[loop].sock_local, &r_fd);
			    t_shutdown(handle, tCxt[loop].sock_local);
			    tCxt[loop].sock_local = 0;
			}
		    }
		}
            }
    }

	for(loop=0; loop<multi_socket_count; loop++)
	{
		app_time_get_elapsed(&tCxt[loop].last_time);
		tCxt[loop].kbytes = tCxt[loop].bytes/1024;
		tCxt[loop].bytes = tCxt[loop].bytes % 1024;
		tCxt[loop].test_type = TCP_RX;
		/* Print test results.*/
		print_test_results (&tCxt[loop]);
	}

ERROR:
#if 0
    for(loop = 0; loop < multi_socket_count; loop++)
    {
        if((0 != tCxt[loop].sock_local) && (A_ERROR != tCxt[loop].sock_local))
        {
            t_shutdown((void*)handle, tCxt[loop].sock_local);
        }
    }
#endif
	
    printf(CFG_COMPLETED_STR);
    printf("shell> ");
    
#if !ZERO_COPY
    if(tCxt.buffer)
        A_FREE(tCxt.buffer, MALLOC_ID_CONTEXT);
#endif
    //num_traffic_streams = 0;
    return;
}

#else

static void ath_tcp_rx_multi_socket (A_INT16 *port, A_INT32 multi_socket_count)
{
    
    A_UINT8 *ip_str = NULL;
    A_UINT16 addr_len=0;
    A_INT32 received =0, loop = 0, error = A_OK;
    A_INT32 sock_local[TCP_MULTI_SOCKET_MAX];
    A_INT32 conn_sock, is_first;
    _ip_address temp_addr;
    SOCKADDR_T foreign_addr;
    SOCKADDR_6_T foreign_addr6;
    TIME_STRUCT      block_time;
    THROUGHPUT_CXT    tCxt;
    
    bench_quit = 0;
    tCxt.sock_peer = A_ERROR;
    
#if !ZERO_COPY
    if((tCxt.buffer = A_MALLOC(CFG_PACKET_SIZE_MAX_RX,MALLOC_ID_CONTEXT)) == NULL)
    {
        printf("Out of memory error\n");
        goto ERROR;
    }
#endif
    
    for (loop = 0; loop < multi_socket_count; loop++)
    {
        if(!v6EnableFlag)
        {
            error = ath_create_socket_and_bind(&(sock_local[loop]), port[loop], SOCK_STREAM_TYPE);
        }
        else
        {
            error = ath_create_socket_and_bind6(&(sock_local[loop]), port[loop], SOCK_STREAM_TYPE);
        }
        
        if (error == A_ERROR)
        {
            goto ERROR;
        }
    }
    
    /* ------ Start test.----------- */
    printf("****************************************************\n");
    printf(" TCP RX Test\n" );
    printf("****************************************************\n");
    
    printf( "Local ports ");
    for (loop = 0; loop < multi_socket_count; loop++)
    	printf("%d ",port[loop]);
    printf("\n");
    printf("Type benchquit to terminate test\n");
    printf("****************************************************\n");
    
    
    app_time_get_elapsed(&tCxt.first_time);
    app_time_get_elapsed(&tCxt.last_time);
    tCxt.last_bytes = 0;
    tCxt.last_kbytes = 0;
    is_first = 1;
    
    for (loop = 0; loop < multi_socket_count; loop++)
    {
        if(t_listen((void*)handle, sock_local[loop], 1) == A_ERROR)
        {
            printf("ERROR: Socket listen error.\r\n");
            goto ERROR;
        }
    }
    
    while(test_for_quit() == 0)
    {
        for (loop = 0; loop < multi_socket_count; loop++)
        {
            if((conn_sock  =  t_select((void*)handle, sock_local[loop],UDP_CONNECTION_WAIT_TIME)) == A_OK)
            {
                tCxt.last_bytes = 0;
                tCxt.last_kbytes = 0;
                tCxt.bytes = 0;
                tCxt.kbytes = 0;
                tCxt.sent_bytes = 0;
                tCxt.sock_local = sock_local[loop];
                
                /*Accept incoming connection*/
                if(!v6EnableFlag)
                {
                    addr_len = sizeof(foreign_addr);
                    tCxt.sock_peer = t_accept((void*)handle, tCxt.sock_local, &foreign_addr, addr_len);
                }
                else
                {
                    addr_len = sizeof(foreign_addr6);
                    tCxt.sock_peer = t_accept((void*)handle, tCxt.sock_local, &foreign_addr6, addr_len);
                }
                
                if(tCxt.sock_peer != A_ERROR)
                {
                    ip_str = A_MALLOC(sizeof(A_UINT8) * 48, MALLOC_ID_TEMPORARY);
                    if (NULL == ip_str)
                    {
                        printf("Out of Memory error\n");
                        goto ERROR;
                    }
                    if(!v6EnableFlag)
                    {
                        temp_addr = LONG_BE_TO_HOST(foreign_addr.sin_addr);
                        printf("Receiving from %s:%d\n", inet_ntoa(*(A_UINT32 *)(&temp_addr), (char *)ip_str), foreign_addr.sin_port);
                    }
                    else
                    {
                        printf("Receiving from %s:%d\n", inet6_ntoa((char *)&foreign_addr6.sin6_addr,(char *)ip_str), foreign_addr6.sin6_port);
                    }
                    if (NULL != ip_str)
                        A_FREE(ip_str, MALLOC_ID_TEMPORARY);
                    
                    app_time_get_elapsed(&tCxt.first_time);
                    app_time_get_elapsed(&tCxt.last_time);
                    block_time = tCxt.first_time;
                    is_first = 1;
                    
                    while(test_for_quit() == 0) /* Receiving data.*/
                    {
                        
                        if(test_for_quit())
                        {
                            app_time_get_elapsed(&tCxt.last_time);
                            tCxt.kbytes = tCxt.bytes/1024;
                            tCxt.bytes = tCxt.bytes % 1024;
                            tCxt.test_type = TCP_RX;
                            /* Print test results.*/
                            print_test_results (&tCxt);
                            break;
                        }
                        
                        conn_sock = t_select((void*)handle, tCxt.sock_peer,UDP_CONNECTION_WAIT_TIME);
                        
                        if(conn_sock == A_OK)
                        {
                            /*Packet is available, receive it*/
#if ZERO_COPY
                            received = t_recv((void*)handle, tCxt.sock_peer, (char**)(&tCxt.buffer), CFG_PACKET_SIZE_MAX_RX, 0);
                            if(received > 0)
                                zero_copy_free(tCxt.buffer);
#else
                            received = t_recv((void*)handle, tCxt.sock_peer, (char*)(&tCxt.buffer[0]), CFG_PACKET_SIZE_MAX_RX, 0);
#endif
                            if(received == A_SOCK_INVALID)
                            {
                                /*Test ended, peer closed connection*/
                                tCxt.kbytes = tCxt.bytes/1024;
                                tCxt.bytes = tCxt.bytes % 1024;
                                tCxt.test_type = TCP_RX;
                                /* Print test results.*/
                                print_test_results (&tCxt);
                                break;
                                //goto ERROR;
                            }
                        }
                        
                        if(conn_sock == A_SOCK_INVALID)
                        {
                            /* Test ended, peer closed connection*/
                            tCxt.kbytes = tCxt.bytes/1024;
                            tCxt.bytes = tCxt.bytes % 1024;
                            tCxt.test_type = TCP_RX;
                            /* Print test results.*/
                            print_test_results (&tCxt);
                            if(t_listen((void*)handle, tCxt.sock_local, 1) == A_ERROR)
                            {
                                printf("ERROR: Socket listen error.\r\n");
                                goto ERROR;
                            }
                            //goto ERROR;
                            break;
                        } /* if(conn_sock == A_SOCK_INVALID) */
                        else
                        {
                            /*Valid packet received*/
                            if(is_first)
                            {
                                /*This is the first packet, set initial time used to calculate throughput*/
                                app_time_get_elapsed(&tCxt.first_time);
                                is_first = 0;
                            }
                            app_time_get_elapsed(&tCxt.last_time);
                            tCxt.bytes += received;
                            tCxt.sent_bytes += received;
                            
                            if(test_for_delay(&tCxt.last_time, &block_time))
                            {
                                /* block to give other tasks an opportunity to run */
                                app_time_delay(SMALL_TX_DELAY);
                                app_time_get_elapsed(&block_time);
                            }
                        } /* if(conn_sock == A_SOCK_INVALID) */
                    } /* while(test_for_quit() == 0) */
                } /* if(tCxt->sock_peer != A_ERROR) */
                
                if(test_for_quit())
                {
                    app_time_get_elapsed(&tCxt.last_time);
                    tCxt.kbytes = tCxt.bytes/1024;
                    tCxt.bytes = tCxt.bytes % 1024;
                    tCxt.test_type = TCP_RX;
                    /* Print test results.*/
                    print_test_results (&tCxt);
                }
            } /* if((conn_sock  =  t_select((void*)handle, tCxt[loop].sock_local,UDP_CONNECTION_WAIT_TIME_MULTI_SOCK)) == A_OK) */
        } /* for (loop = 0; loop < multi_socket_count; loop++) */
    } /* while(test_for_quit()==0) */
    
ERROR:
    
    for(loop = 0; loop < multi_socket_count; loop++)
    {
        if((0 != sock_local[loop]) && (A_ERROR != sock_local[loop]))
        {
            t_shutdown((void*)handle, sock_local[loop]);
        }
    }
    
    printf(CFG_COMPLETED_STR);
    printf("shell> ");
    
#if !ZERO_COPY
    if(tCxt.buffer)
        A_FREE(tCxt.buffer, MALLOC_ID_CONTEXT);
#endif
    //num_traffic_streams = 0;
    return;
}
#endif // #if t_select_ver1

#if T_SELECT_VER1

static void ath_tcp_tx_multi_socket (THROUGHPUT_CXT *p_tCxt, A_INT32 multi_socket_count)
{
    SOCKADDR_T local_addr;
    SOCKADDR_T foreign_addr;
    char ip_str[16];
    SOCKADDR_6_T foreign_addr6;
    char ip6_str[48];
    TIME_STRUCT start_time,current_time,block_time;
    A_UINT32 packet_size = p_tCxt->tx_params.packet_size;
    A_UINT32 cur_packet_number;
    A_UINT32 next_prog_report, prog_report_inc;
    A_UINT32 buffer_offset;
    A_INT32 send_result,result;
    _ip_address temp_addr;
	A_UINT32 w_fd, w_fd_working_set ;
	A_UINT32 loop;
	A_UINT32 sock_peer[MAX_SOCKETS_SUPPORTED];
	THROUGHPUT_CXT    tCxt[TCP_MULTI_SOCKET_MAX];
	
    //init quit flag
    bench_quit = 0;
    num_traffic_streams++;
	
    if(packet_size > CFG_PACKET_SIZE_MAX_TX) /* Check max size.*/
		packet_size = CFG_PACKET_SIZE_MAX_TX;
	
    /* ------ Start test.----------- */
    printf("**********************************************************\n");
    printf("IOT TCP TX Test\n" );
    printf("**********************************************************\n");
	
    if(!v6EnableFlag)
    {
        temp_addr = LONG_BE_TO_HOST(p_tCxt->tx_params.ip_address);
        printf("Remote IP addr. %s\n", inet_ntoa(*(A_UINT32 *)( &temp_addr), ip_str));
    }
    else
    {
        printf("Remote IP addr. %s\n", inet6_ntoa((char *)p_tCxt->tx_params.v6addr, (void *)ip6_str));
    }
    printf("Remote port: ");
	for (loop = 0; loop < multi_socket_count; loop++)
		printf("%d, ", p_tCxt->tx_params.port + loop);
    printf("\nMessage size%d\n", p_tCxt->tx_params.packet_size);
    printf("Number of messages %d\n", p_tCxt->tx_params.packet_number);
    printf("Type benchquit to cancel\n");
    printf("**********************************************************\n");
	
    if(p_tCxt->tx_params.test_mode == TIME_TEST)
    {
        app_time_get(&start_time);
        prog_report_inc = p_tCxt->tx_params.tx_time/20;
		
        if(prog_report_inc==0)
            prog_report_inc = 1;
		
        next_prog_report = start_time.SECONDS + prog_report_inc;
    }else if(p_tCxt->tx_params.test_mode == PACKET_TEST){
        /* generate 20 progress characters across screen to provide progress feedback */
        prog_report_inc = p_tCxt->tx_params.packet_number/20;
		
        if(prog_report_inc==0)
			prog_report_inc = 1;
		
        next_prog_report = prog_report_inc;
    }
	
	for (loop = 0; loop < multi_socket_count; loop++)
	{
		/* Create socket */
		if(!v6EnableFlag)
		{
			if((sock_peer[loop] = t_socket((void*)handle, ATH_AF_INET, SOCK_STREAM_TYPE, 0)) == A_ERROR)
			{
				printf("ERROR: Unable to create socket\n");
				goto ERROR_1;
			}
			memset(&foreign_addr, 0, sizeof(local_addr));
			foreign_addr.sin_addr = p_tCxt->tx_params.ip_address;
			foreign_addr.sin_port = p_tCxt->tx_params.port + loop;
			foreign_addr.sin_family = ATH_AF_INET;
			
			printf("Connecting.\n");
			/* Connect to the server.*/
			if(t_connect((void*)handle, sock_peer[loop], (&foreign_addr), sizeof(foreign_addr)) == A_ERROR)
			{
				printf("Connection failed.\n");
				goto ERROR_2;
			}
		}
		else
		{
			if((sock_peer[loop] = t_socket((void*)handle, ATH_AF_INET6, SOCK_STREAM_TYPE, 0)) == A_ERROR)
			{
				printf("ERROR: Unable to create socket\n");
				goto ERROR_1;
			}
			memset(&foreign_addr6, 0, sizeof(foreign_addr6));
			memcpy(&foreign_addr6.sin6_addr, p_tCxt->tx_params.v6addr,16);;
			foreign_addr6.sin6_port = p_tCxt->tx_params.port + loop;
			foreign_addr6.sin6_family = ATH_AF_INET6;
			
			printf("Connecting.\n");
			/* Connect to the server.*/
			if(t_connect((void*)handle, sock_peer[loop], (SOCKADDR_T *)(&foreign_addr6), sizeof(foreign_addr6)) == A_ERROR)
			{
				printf("Connection failed.\n");
				goto ERROR_2;
			}
		}
	}
	
	/* Sending.*/
	printf("Sending.\n");
	
	/*Reset all counters*/
	p_tCxt->bytes = 0;
	p_tCxt->kbytes = 0;
	p_tCxt->last_bytes = 0;
	p_tCxt->last_kbytes = 0;
	cur_packet_number = 0;
	buffer_offset = 0;
	
	app_time_get_elapsed(&p_tCxt->first_time);
	app_time_get_elapsed(&p_tCxt->last_time);
	block_time = p_tCxt->first_time;
	
	app_time_delay(20);
	
	//Form the w_fd mask
    w_fd = 0;
    for(loop=0; loop<multi_socket_count; loop++)
    {
		memcpy(&(tCxt[loop]), p_tCxt, sizeof(THROUGHPUT_CXT));
		FD_Set(sock_peer[loop], &w_fd);
    }
	
	while(1)
	{
		if(test_for_quit()){
			break;
		}
		
		w_fd_working_set = w_fd;
		
		/* Break when all the sockets are closed */
		if(!w_fd)
			break;
		
		if( (result = (t_select_ver1((void*)handle, multi_socket_count, NULL, &w_fd_working_set, NULL, 1 /*UDP_CONNECTION_WAIT_TIME_MULTI_SOCK*/))) > 0 )
		{
			for(loop = 0; loop < multi_socket_count; loop++)
			{
				/* If the select is not set */
				if(! FD_IsSet(sock_peer[loop], w_fd_working_set))
				{
					continue;
				}
				while((tCxt[loop].buffer = CUSTOM_ALLOC(packet_size)) == NULL)
				{
					//Wait till we get a buffer
					if(test_for_quit()){
						goto ERROR_2;
					}
					/*Allow small delay to allow other thread to run*/
					app_time_delay(SMALL_TX_DELAY);
				}
				
				if(!v6EnableFlag)
				{
					send_result = t_send((void*)handle, sock_peer[loop], (unsigned char*)(tCxt[loop].buffer), packet_size, 0);
				}
				else
				{
					send_result = t_send((void*)handle, sock_peer[loop], (unsigned char*)(tCxt[loop].buffer), packet_size, 0);
				}
#if !NON_BLOCKING_TX
				/*Free the buffer only if NON_BLOCKING_TX is not enabled*/
				if(tCxt[loop].buffer)
					CUSTOM_FREE(tCxt[loop].buffer);
#endif
				app_time_get_elapsed(&tCxt[loop].last_time);
				
				/****Bandwidth control- add delay if user has specified it************/
				if(tCxt[loop].tx_params.interval)
					app_time_delay(tCxt[loop].tx_params.interval);
				
				if ( send_result == A_ERROR )
				{
					printf("send packet error = %d\n", send_result);
					FD_Clr(sock_peer[loop], &w_fd);
					continue;
				}
				else if(send_result == A_SOCK_INVALID )
				{
					/*Socket has been closed by target due to some error, gracefully exit*/
					printf("Socket closed unexpectedly\n");
					FD_Clr(sock_peer[loop], &w_fd);
					continue;
				}
				else if(send_result)
				{
					tCxt[loop].bytes += send_result;
					tCxt[loop].sent_bytes += send_result;
					buffer_offset += send_result;
					
					if(buffer_offset == packet_size)
					{
						cur_packet_number ++;
						buffer_offset = 0;
					}
					
					if(tCxt[loop].tx_params.test_mode == PACKET_TEST)
					{
						/*Test based on number of packets, check if we have reached specified limit*/
						if(cur_packet_number >= next_prog_report){
							printf(".");
							next_prog_report += prog_report_inc;
						}
						
						if((cur_packet_number >= tCxt[loop].tx_params.packet_number))
						{
							/* Test completed, print throughput results.*/
							FD_Clr(sock_peer[loop], &w_fd);
							continue;
						}
					}
					else if(tCxt[loop].tx_params.test_mode == TIME_TEST)
					{
						/*Test based on time interval, check if we have reached specified limit*/
						app_time_get(&current_time);
						
						if(current_time.SECONDS >= next_prog_report){
							printf(".");
							next_prog_report += prog_report_inc;
						}
						
						if(check_test_time(&tCxt[loop]))
						{
							/* Test completed, print test results.*/
							FD_Clr(sock_peer[loop], &w_fd);
							continue;
						}
					}
				} /* send_result */
			}
		} /* select */
	} /* while loop */
	
ERROR_2:
	printf("\n"); // new line to separate from progress line
	
	for (loop = 0; loop < multi_socket_count; loop++)
	{
		tCxt[loop].kbytes = tCxt[loop].bytes/1024;
		tCxt[loop].bytes = tCxt[loop].bytes % 1024;
		tCxt[loop].test_type = TCP_TX;
		print_test_results(&tCxt[loop]);
		
		if(send_result != A_SOCK_INVALID )
			t_shutdown((void*)handle, sock_peer[loop]);
	}
	
	ERROR_1:
    printf(CFG_COMPLETED_STR);
    printf("\n");
	
	return;
}
#endif

#endif // #if MULTI_SOCKET_SUPPORT


/************************************************************************
* NAME: ath_tcp_rx
*
* DESCRIPTION: Start throughput TCP server.
************************************************************************/
static void ath_tcp_rx (THROUGHPUT_CXT *p_tCxt)
{
    A_UINT8 ip_str[16];
    A_UINT16 addr_len=0;
    A_UINT32 port;
    A_INT32 received =0;

    A_INT32 conn_sock,isFirst = 1,result;
    _ip_address temp_addr;
    SOCKADDR_T local_addr;
    SOCKADDR_T foreign_addr;
    SOCKADDR_6_T local_addr6;
    SOCKADDR_6_T foreign_addr6;
    A_UINT8 ip6_str[48];
    TIME_STRUCT block_time;
#if ENABLE_SSL
    SSL_INST *ssl = &ssl_inst[SSL_SERVER_INST];
#endif

    port = p_tCxt->rx_params.port;
    bench_quit = 0;
    p_tCxt->sock_peer = A_ERROR;

    num_traffic_streams++;

#if !ZERO_COPY
    /*Allocate buffer*/
    if((p_tCxt->buffer = A_MALLOC(CFG_PACKET_SIZE_MAX_RX, MALLOC_ID_CONTEXT)) == NULL)
    {
            printf("Out of memory error\n");
             goto ERROR_1;
    }
#endif

    if(!v6EnableFlag)
    {
    	/* Create listen socket */
        if((p_tCxt->sock_local = t_socket((void*)handle, ATH_AF_INET, SOCK_STREAM_TYPE, 0)) == A_ERROR)
        {
            printf("ERROR: Socket creation error.\r\n");
            goto ERROR_1;
        }

        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_port = port;
        local_addr.sin_family = ATH_AF_INET;

         /* Bind socket.*/
        if(t_bind((void*)handle, p_tCxt->sock_local, &local_addr, sizeof(local_addr)) == A_ERROR)
        {
            printf("ERROR: Socket bind error.\r\n");
            goto ERROR_2;
        }
    }
    else
    {
        if((p_tCxt->sock_local = t_socket((void*)handle, ATH_AF_INET6, SOCK_STREAM_TYPE, 0)) == A_ERROR)
        {
            printf("ERROR: Socket creation error.\r\n");
            goto ERROR_1;
        }

        memset(&local_addr6, 0, sizeof(local_addr6));
        local_addr6.sin6_port = port;
        local_addr6.sin6_family = ATH_AF_INET6;

        /* Bind socket.*/
        if(t_bind((void*)handle, p_tCxt->sock_local, (SOCKADDR_T *)&local_addr6, sizeof(local_addr6)) == A_ERROR)
        {
            printf("ERROR: Socket bind error.\r\n");
            goto ERROR_2;
        }
    }

    /* ------ Start test.----------- */
    printf("\n****************************************************\n");
#if ENABLE_SSL
    printf(" %s RX Test\n", p_tCxt->test_type == SSL_RX ? "SSL" : "TCP");
#else
    printf(" TCP RX Test\n" );
#endif
    printf("****************************************************\n");

    printf( "Local port %d\n", port);
    printf("Type benchquit to terminate test\n");
    printf("****************************************************\n");

    app_time_get_elapsed(&p_tCxt->first_time);
    app_time_get_elapsed(&p_tCxt->last_time);
    p_tCxt->last_bytes = 0;
    p_tCxt->last_kbytes = 0;

    while(1)
    {
        printf("Waiting.\n");

        /* Listen. */
        if(t_listen((void*)handle, p_tCxt->sock_local, 1) == A_ERROR)
        {
            printf("ERROR: Socket listen error.\r\n");
            goto ERROR_2;
        }

        p_tCxt->bytes = 0;
        p_tCxt->kbytes = 0;
        p_tCxt->sent_bytes = 0;

        do
        {
            if(test_for_quit()){
                goto tcp_rx_QUIT;
            }
            /* block for 500msec or until a packet is received */
            conn_sock = t_select((void*)handle, p_tCxt->sock_local,UDP_CONNECTION_WAIT_TIME);

            if(conn_sock == A_SOCK_INVALID)
              goto tcp_rx_QUIT;       //Peer closed connection, socket no longer valid

        }while(conn_sock == A_ERROR);

        /*Accept incoming connection*/
        if(!v6EnableFlag)
        {
          addr_len = sizeof(foreign_addr);
          p_tCxt->sock_peer = t_accept((void*)handle, p_tCxt->sock_local, &foreign_addr, addr_len);
        }
        else
        {
          addr_len = sizeof(foreign_addr6);
          p_tCxt->sock_peer = t_accept((void*)handle, p_tCxt->sock_local, &foreign_addr6, addr_len);
        }

        if(p_tCxt->sock_peer != A_ERROR)
        {
#if ENABLE_SSL
            if (p_tCxt->test_type == SSL_RX)
            {
                if (ssl->ssl == NULL)
                {
                    // Create SSL connection object
                    ssl->ssl = SSL_new(ssl->sslCtx);
                    if (ssl->ssl == NULL)
                    {
                        printf("ERROR: Unable to create SSL context\n");
                        goto tcp_rx_QUIT;
                    }

                    // configure the SSL connection
                    if (ssl->config_set)
                    {
                        result = SSL_configure(ssl->ssl, &ssl->config);
                        if (result < A_OK)
                        {
                            printf("ERROR: SSL configure failed (%d)\n", result);
                            goto tcp_rx_QUIT;
                        }
                    }

                }

                // Add socket handle to SSL connection
                result = SSL_set_fd(ssl->ssl, p_tCxt->sock_peer);
                if (result < A_OK)
                {
                    printf("ERROR: Unable to add socket handle to SSL\n");
                    goto tcp_rx_QUIT;
                }

                // SSL handshake with server
                result = SSL_accept(ssl->ssl);
                if (result < 0)
                {
                    printf("ERROR: SSL accept failed (%d)\n", result);
                    goto tcp_rx_QUIT;
                }

            }
#endif
            memset(ip_str, 0, sizeof(ip_str));
            if(!v6EnableFlag)
            {
                temp_addr = LONG_BE_TO_HOST(foreign_addr.sin_addr);
                printf("Receiving from %s:%d\n", inet_ntoa(*(A_UINT32 *)(&temp_addr), (char *)ip_str), foreign_addr.sin_port);
            }
            else
            {
                printf("Receiving from %s:%d\n", inet6_ntoa((char *)&foreign_addr6.sin6_addr,(char *)ip6_str), foreign_addr6.sin6_port);
            }
            
            app_time_get_elapsed(&p_tCxt->first_time);
            app_time_get_elapsed(&p_tCxt->last_time);
            block_time = p_tCxt->first_time;
            isFirst = 1;
            
            while(1) /* Receiving data.*/
            {
            	if(test_for_quit()){
                    t_shutdown((void*)handle, p_tCxt->sock_peer);
            	    app_time_get_elapsed(&p_tCxt->last_time);
                    goto tcp_rx_QUIT;
                }
                
                conn_sock = t_select((void*)handle, p_tCxt->sock_peer,UDP_CONNECTION_WAIT_TIME);
                
                if(conn_sock == A_OK)
                {
                    /*Packet is available, receive it*/
#if ZERO_COPY
#if ENABLE_SSL
                    if (p_tCxt->test_type == SSL_RX)
                    {
                        received = SSL_read(ssl->ssl, (void**)&p_tCxt->buffer, CFG_PACKET_SIZE_MAX_RX);
                    }
                    else
#endif
                    {
                        received = t_recv((void*)handle, p_tCxt->sock_peer, (void**)(&p_tCxt->buffer), CFG_PACKET_SIZE_MAX_RX, 0);
                    }
                    if(received > 0)
                    {
                        // Free the RX buffer again
                        zero_copy_free(p_tCxt->buffer);
                    }
#else
#if ENABLE_SSL
                    if (p_tCxt->ssl_test)
                    {
                        received = SSL_read(ssl->ssl, p_tCxt->buffer, CFG_PACKET_SIZE_MAX_RX);
                    }
                    else
#endif
                    {
                        received = t_recv((void*)handle, p_tCxt->sock_peer, (char*)(&p_tCxt->buffer[0]), CFG_PACKET_SIZE_MAX_RX, 0);
                    }
#endif

                    if(received == A_SOCK_INVALID)
                    {
                        /*Test ended, peer closed connection*/
                        p_tCxt->kbytes = p_tCxt->bytes/1024;
                        p_tCxt->bytes = p_tCxt->bytes % 1024;
#if ENABLE_SSL
                        if (ssl_role == SSL_SERVER && ssl->ssl != NULL)
                        {
                            SSL_shutdown(ssl->ssl);
                            ssl->ssl = NULL;
                        }
#endif
                        /* Print test results.*/
                        print_test_results (p_tCxt);
                        //goto ERROR_2;
                        break;
                    }
                }

                if(conn_sock == A_SOCK_INVALID)
                {
                    /* Test ended, peer closed connection*/
                    p_tCxt->kbytes = p_tCxt->bytes/1024;
                    p_tCxt->bytes = p_tCxt->bytes % 1024;
#if ENABLE_SSL
                    if (ssl_role == SSL_SERVER && ssl->ssl != NULL)
                    {
                        SSL_shutdown(ssl->ssl);
                        ssl->ssl = NULL;
                    }
#endif
                    /* Print test results.*/
                    print_test_results (p_tCxt);
                    //goto ERROR_2;
                    break;
                }
                else
                {
                    /*Valid packet received*/
                    //printf("RX: %d\n", received);
                    if(isFirst)
                    {
                        /*This is the first packet, set initial time used to calculate throughput*/
                        app_time_get_elapsed(&p_tCxt->first_time);
                        isFirst = 0;
                    }
                    app_time_get_elapsed(&p_tCxt->last_time);
                    p_tCxt->bytes += received;
                    p_tCxt->sent_bytes += received;
                    received = 0;
                    if(test_for_delay(&p_tCxt->last_time, &block_time)){
                        /* block to give other tasks an opportunity to run */
                        app_time_delay(TX_DELAY);
                        app_time_get_elapsed(&block_time);
            	    }
                }
            }
        }

        if(test_for_quit()){
        app_time_get_elapsed(&p_tCxt->last_time);
                goto tcp_rx_QUIT;
        }
    }

tcp_rx_QUIT:
    p_tCxt->kbytes = p_tCxt->bytes/1024;
    p_tCxt->bytes = p_tCxt->bytes % 1024;
    /* Print test results.*/
    print_test_results (p_tCxt);


    //t_shutdown((void*)handle, p_tCxt->sock_peer);

ERROR_2:
    t_shutdown((void*)handle, p_tCxt->sock_local);

ERROR_1:
#if ENABLE_SSL
    if (ssl_role == SSL_SERVER && ssl->ssl != NULL)
    {
        SSL_shutdown(ssl->ssl);
        ssl->ssl = NULL;
    }
#endif
    printf("*************IOT Throughput Test Completed **************\n");
    printf("Shell> ");
    num_traffic_streams--;
#if !ZERO_COPY
    if(p_tCxt->buffer)
    	A_FREE(p_tCxt->buffer,MALLOC_ID_CONTEXT);
#endif
}

typedef struct ip   {
   unsigned char   ip_ver_ihl;    /* 4 bit version, 4 bit hdr len in 32bit words */
   unsigned char   ip_tos;        /* Type of Service */
   unsigned short  ip_len;        /* Total packet length including header */
   unsigned short  ip_id;         /* ID for fragmentation */
   unsigned short  ip_flgs_foff;  /* mask in flags as needed */
   unsigned char   ip_time;       /* Time to live (secs) */
   unsigned char   ip_prot;       /* protocol */
   unsigned short  ip_chksum;     /* Header checksum */
   unsigned long  ip_src;        /* Source name */
   unsigned long  ip_dest;       /* Destination name */
}host_ip;
	

typedef struct ipv6
{
   unsigned char   ip_ver_flo[4];    /* 4 bits version (6), 8 bits class, & flow label */
   unsigned short  ip_len;           /* payload length */
   unsigned char   ip_nexthdr;       /* next header type 6 = TCP, etc */
   unsigned char   ip_hopcount;      /* hops left until expires */
   unsigned char   ip_src[16];           /* source and dest addresses */
   unsigned char   ip_dest[16];
} host_ip6;


/************************************************************************
* NAME: ath_udp_rx
*
* DESCRIPTION: Start throughput UDP server.
************************************************************************/
static void ath_udp_rx (THROUGHPUT_CXT *p_tCxt)
{
    A_UINT8           ip_str[16];
    A_UINT32          addr_len;
    A_UINT16          port;
    A_INT32 	      received;
    A_INT32 	      conn_sock;
    A_INT32 	      is_first = 1;
    SOCKADDR_T        addr;
    SOCKADDR_T        local_addr;
    SOCKADDR_6_T      addr6;
    SOCKADDR_6_T      local_addr6;
    A_UINT8           ip6_str[48];
    A_UINT32 	      temp_addr;
    TIME_STRUCT       block_time;
    int overhead = 0;

    // init quit flag
    bench_quit = 0;

    //Parse remaining parameters
    port = p_tCxt->rx_params.port;

     num_traffic_streams++;

#if !ZERO_COPY
    if((p_tCxt->buffer = A_MALLOC(CFG_PACKET_SIZE_MAX_RX,MALLOC_ID_CONTEXT)) == NULL)
    {
            printf("Out of memory error\n");
            return;
    }
#endif

    if(!v6EnableFlag)
    {
        if((p_tCxt->sock_local = t_socket((void*)handle,ATH_AF_INET,SOCK_DGRAM_TYPE,0))== A_ERROR)
        {
            printf("ERROR:: Socket creation error.\r\n");
            goto ERROR_1;
        }

        /* Bind.*/
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_port = port;
        local_addr.sin_family = ATH_AF_INET;

        if(t_bind((void*)handle, p_tCxt->sock_local, (&local_addr), sizeof(local_addr)) != A_OK)
        {
            printf("ERROR:: Socket bind error.\r\n");
            goto ERROR_2;
        }

        /*************Multicast support ************************/
	if(handle_mcast_param(p_tCxt) !=A_OK)
	  goto ERROR_2;
     }
     else
     {
        if((p_tCxt->sock_local = t_socket((void*)handle,ATH_AF_INET6,SOCK_DGRAM_TYPE,0))== A_ERROR)
        {
            printf("ERROR:: Socket creation error.\r\n");
            goto ERROR_1;
        }

	memset(&local_addr6, 0, sizeof(local_addr6));
        local_addr6.sin6_port = port;
        local_addr6.sin6_family = ATH_AF_INET6;

        if(t_bind((void*)handle, p_tCxt->sock_local,(SOCKADDR_T *) (&local_addr6), sizeof(local_addr6)) != A_OK)
        {
            printf("ERROR:: Socket bind error.\r\n");
            goto ERROR_2;
        }

        /*************Multicast support ************************/
	if(handle_mcast_param(p_tCxt) !=A_OK)
	  goto ERROR_2;
     }


     /* ------ Start test.----------- */
    printf("****************************************************\n");
    printf(" UDP RX Test\n" );
    printf("****************************************************\n");

    printf("Local port %d\n", port);
    printf("Type benchquit to termintate test\n");
    printf("****************************************************\n");

    /*Initilize start, stop times*/

    app_time_get_elapsed(&p_tCxt->last_time);
    app_time_get_elapsed(&p_tCxt->first_time);
    p_tCxt->last_bytes = 0;
    p_tCxt->last_kbytes = 0;
    block_time = p_tCxt->first_time;

    if(p_tCxt->rx_params.ip_hdr_inc ==1)
    {
       if(!v6EnableFlag)
	   overhead = 28;  /* IP_HDR_SIZE  + UDP_HDR_SIZE */
        else
	   overhead =  48;  /* IP6_HDR_SIZE + UDP_HDR_SIZE */ 	
    }	     

    while(test_for_quit()==0) /* Main loop */
    {
        printf("Waiting.\n");

        p_tCxt->bytes = 0;
        p_tCxt->kbytes = 0;

        p_tCxt->sent_bytes = 0;

        addr_len = sizeof(SOCKADDR_T);
        is_first = 1;

        while(test_for_quit()==0)
        {
              do
              {
                      if(test_for_quit()){
                              goto ERROR_3;
                      }
                      /* block for 500msec or until a packet is received */
                      conn_sock = t_select((void*)handle, p_tCxt->sock_local,UDP_CONNECTION_WAIT_TIME);

                      if(conn_sock == A_SOCK_INVALID)
                        goto ERROR_3;       // socket no longer valid

              }while(conn_sock == A_ERROR);

              /* Receive data */
#if ZERO_COPY
             if(!v6EnableFlag)
             {
                received = t_recvfrom((void*)handle, p_tCxt->sock_local,
            					(char**)(&p_tCxt->buffer),
            					CFG_PACKET_SIZE_MAX_RX, 0,
                				&addr, (A_UINT32*)&addr_len);
             }
             else
             {
                 addr_len = sizeof(SOCKADDR_6_T);
                 received = t_recvfrom((void*)handle, p_tCxt->sock_local,
            					(char**)(&p_tCxt->buffer),
            					CFG_PACKET_SIZE_MAX_RX, 0,
                				&addr6, (A_UINT32*)&addr_len);

             }
              if(received > 0)
                zero_copy_free(p_tCxt->buffer);
#else
            if(!v6EnableFlag)
            {
                 received = t_recvfrom((void*)handle, p_tCxt->sock_local,
            					(char*)(&p_tCxt->buffer[0]),
            					CFG_PACKET_SIZE_MAX_RX, 0,
                				&addr, (A_UINT32*)&addr_len);
            }
            else
            {
                 addr_len = sizeof(SOCKADDR_6_T);
                 received = t_recvfrom((void*)handle, p_tCxt->sock_local,
            					(char*)(&p_tCxt->buffer[0]),
            					CFG_PACKET_SIZE_MAX_RX, 0,
                				&addr6, (A_UINT32*)&addr_len);
            }
#endif


            if(received >= sizeof(EOT_PACKET) + overhead)
            {
                /* Reset timeout. */
                if( received > sizeof(EOT_PACKET) ){
                      app_time_get_elapsed(&p_tCxt->last_time);
                }
                	 
                if(is_first)
                {
                    if( received > (sizeof(EOT_PACKET) + overhead))
                    {
                       if(p_tCxt->rx_params.ip_hdr_inc ==1)
		      {
                        if(!v6EnableFlag)
	               {
                          host_ip *ip = NULL;
		          ip  = (host_ip *)(&p_tCxt->buffer[0]);
			  memset(ip_str,0,sizeof(ip_str));
                          printf("Src IP of RX Pkt %s \r\n", inet_ntoa(*(A_UINT32 *)(&ip->ip_dest),(char *)ip_str));
	               }		 
		       else
		       {
                         host_ip6 *ip = NULL;
		         ip  = (host_ip6 *)(&p_tCxt->buffer[0]);
			 memset(ip6_str,0,sizeof(ip6_str));
                         printf("SrcIP of RX PKT %s \r\n", inet6_ntoa((char *)&ip->ip_dest, (char *)ip6_str));
		       }	  
		      }

                      temp_addr = LONG_BE_TO_HOST(addr.sin_addr);

                        if(!v6EnableFlag)
                        {
                            printf("Receiving from %s:%d\n", inet_ntoa(*(A_UINT32 *)(&temp_addr), (char *)ip_str), addr.sin_port);
                        }
                        else
                        {
                            printf("Receiving from %s:%d\n", inet6_ntoa((char *)&addr6.sin6_addr, (char *)ip6_str), addr6.sin6_port);
                        }

                        app_time_get_elapsed(&p_tCxt->first_time);
                        is_first = 0;
                    }
                }
                else
                {
                    p_tCxt->bytes += received;
                    p_tCxt->sent_bytes += received;

                    if(test_for_delay(&p_tCxt->last_time, &block_time)){
                            /* block to give other tasks an opportunity to run */
                            app_time_delay(SMALL_TX_DELAY);
                            app_time_get_elapsed(&block_time);
                    }
                    if(received == sizeof(EOT_PACKET) + overhead) /* End of transfer. */
                    {
    			/* Send throughput results to Peer so that it can display correct results*/
                         /* Send throughput results to Peer so that it can display correct results*/
                        if(!v6EnableFlag)
                        {
                         sendAck( p_tCxt, (A_VOID*)&addr, port);
                        }
                        else
                        {
                         sendAck( p_tCxt, (A_VOID*)&addr6, port);
                        }

                        break;
                    }
                }
            }
    	}

ERROR_3:
        p_tCxt->kbytes = p_tCxt->bytes/1024;
        p_tCxt->bytes = p_tCxt->bytes % 1024;
	p_tCxt->test_type = UDP_RX;
	print_test_results (p_tCxt);
     }
ERROR_2:
    t_shutdown((void*)handle,p_tCxt->sock_local);

ERROR_1:

    printf(CFG_COMPLETED_STR);
    printf("shell> ");

#if !ZERO_COPY
    if(p_tCxt->buffer)
    	A_FREE(p_tCxt->buffer,MALLOC_ID_CONTEXT);
#endif

     num_traffic_streams--;
}




/************************************************************************
* NAME: ath_udp_echo
*
* DESCRIPTION: A reference implementation of UDP Echo server. It will echo
*              a packet received on specified port.
************************************************************************/
void ath_udp_echo (int_32 argc, char_ptr argv[])
{
    A_UINT32          addr_len;
    A_UINT16          port;
    A_INT32 	      received,result;
    A_INT32 	      conn_sock;
    SOCKADDR_T        addr;
    SOCKADDR_T        local_addr;
    SOCKADDR_6_T      addr6;
    SOCKADDR_6_T      local_addr6;
    A_UINT8*          rxBuffer, *txBuffer;

#if !ZERO_COPY
    printf("This example is only supported with zero copy feature\n");
    return;
#else
    // init quit flag
    bench_quit = 0;

    if(argc < 3){
      printf("Missing UDP port\n");
      return;
    }
    /*Get listening port*/
    port = atoi(argv[2]);

    if(!v6EnableFlag)
    {
        if((conn_sock = t_socket((void*)handle,ATH_AF_INET,SOCK_DGRAM_TYPE,0))== A_ERROR)
        {
            printf("ERROR:: Socket creation error.\r\n");
            goto ERROR_1;
        }

        /* Bind.*/
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_port = port;
        local_addr.sin_family = ATH_AF_INET;

        if(t_bind((void*)handle, conn_sock, (&local_addr), sizeof(local_addr)) != A_OK)
        {
            printf("ERROR:: Socket bind error.\r\n");
            goto ERROR_2;
        }
     }
     else
     {
        if((conn_sock = t_socket((void*)handle,ATH_AF_INET6,SOCK_DGRAM_TYPE,0))== A_ERROR)
        {
            printf("ERROR:: Socket creation error.\r\n");
            goto ERROR_1;
        }

	memset(&local_addr6, 0, sizeof(local_addr6));
        local_addr6.sin6_port = port;
        local_addr6.sin6_family = ATH_AF_INET6;

        if(t_bind((void*)handle, conn_sock,(SOCKADDR_T *) (&local_addr6), sizeof(local_addr6)) != A_OK)
        {
            printf("ERROR:: Socket bind error.\r\n");
            goto ERROR_2;
        }
     }

     /* ------ Start test.----------- */
    printf("****************************************************\n");
    printf(" UDP Echo Server\n" );
    printf("****************************************************\n");
    printf("Local port %d\n", port);
    printf("Type benchquit to termintate test\n");
    printf("****************************************************\n");

    addr_len = sizeof(SOCKADDR_T);

    while(test_for_quit()==0)
    {
          do
          {
                  if(test_for_quit()){
                          goto ERROR_2;
                  }
                  /* block for 500msec or until a packet is received */
                  result = t_select((void*)handle, conn_sock,UDP_CONNECTION_WAIT_TIME);

                  if(result == A_SOCK_INVALID)
                    goto ERROR_2;       // socket no longer valid

          }while(result == A_ERROR);

          /* Receive data */

         if(!v6EnableFlag)
         {
            received = t_recvfrom((void*)handle, conn_sock,
                                            (void**)(&rxBuffer),
                                            CFG_PACKET_SIZE_MAX_RX, 0,
                                            &addr, (A_UINT32*)&addr_len);
         }
         else
         {
             addr_len = sizeof(SOCKADDR_6_T);
             received = t_recvfrom((void*)handle, conn_sock,
                                            (void**)(&rxBuffer),
                                            CFG_PACKET_SIZE_MAX_RX, 0,
                                            &addr6, (A_UINT32*)&addr_len);

         }
          if(received > 0)
          {
              printf("Received %d bytes\n",received);
              while((txBuffer = CUSTOM_ALLOC(received)) == NULL)
              {
                  //Wait till we get a buffer
                  if(test_for_quit()){
                      goto ERROR_2;
                  }
                  /*Allow small delay to allow other thread to run*/
                  app_time_delay(SMALL_TX_DELAY);
              }
              /*Copy received contents in TX buffer*/
              A_MEMCPY(txBuffer,rxBuffer,received);
              /*Free the RX buffer*/
              zero_copy_free(rxBuffer);

              /*Send the received data back to sender*/
              if(!v6EnableFlag)
              {
                if(t_sendto((void*)handle, conn_sock, (unsigned char*)(txBuffer), received, 0,(&addr), sizeof(addr)) == A_ERROR){
                  printf("Send failed\n");
                }
              }
              else
              {
                if(t_sendto((void*)handle, conn_sock, (unsigned char*)(txBuffer), received, 0,(&addr6),sizeof(addr6)) == A_ERROR){
                  printf("Send failed\n");
                }
              }

#if !NON_BLOCKING_TX
                  /*Free the buffer only if NON_BLOCKING_TX is not enabled*/
                  if(txBuffer){
                      CUSTOM_FREE(txBuffer);
                  }
#endif
    	    }
     }
ERROR_2:
    t_shutdown((void*)handle,conn_sock);

ERROR_1:

    printf(CFG_COMPLETED_STR);
    printf("shell> ");
#endif
}



/************************************************************************
* NAME: test_for_delay
*
* DESCRIPTION:  delay for 1% of the time used by this task to give other
*               tasks an opportunity.
* Parameters: pointer to current and start time
************************************************************************/
int_32 test_for_delay(TIME_STRUCT *pCurr, TIME_STRUCT *pBase)
{
      uint_32 total = (pCurr->SECONDS - pBase->SECONDS)*1000;
      total += pCurr->MILLISECONDS - pBase->MILLISECONDS;

      if(total > TX_DELAY_INTERVAL){
              return 1;
      }
      return 0;
}





/************************************************************************
* NAME: check_test_time
*
* DESCRIPTION: If test mode is time, check if current time has exceeded
* test time limit
* Parameters: pointer to throughput context
************************************************************************/
uint_32 check_test_time(THROUGHPUT_CXT *p_tCxt)
{
    uint_32 sec_interval = (p_tCxt->last_time.SECONDS - p_tCxt->first_time.SECONDS);
    uint_32 ms_interval;
    uint_32 total_interval;

    if(sec_interval < p_tCxt->tx_params.tx_time)
    	return 0;

    ms_interval = (p_tCxt->last_time.MILLISECONDS - p_tCxt->first_time.MILLISECONDS);
    total_interval = sec_interval*1000 + ms_interval;

    if(total_interval > p_tCxt->tx_params.tx_time*1000)
        return 1;
    else
        return 0;
}



/************************************************************************
* NAME: wait_for_response
*
* DESCRIPTION: In UDP uplink test, the test is terminated by transmitting
* end-mark (single byte packet). We have implemented a feedback mechanism
* where the Peer will reply with receive stats allowing us to display correct
* test results.
* Parameters: pointer to throughput context
************************************************************************/

int_32 wait_for_response(THROUGHPUT_CXT *p_tCxt, SOCKADDR_T foreign_addr,SOCKADDR_6_T foreign_addr6)
{
    uint_32 received=0;
    int_32 error = A_ERROR;
    SOCKADDR_T local_addr;
    SOCKADDR_T addr;
    uint_32 addr_len;
    stat_packet_t* stat_packet;
    SOCKADDR_6_T local_addr6;
    SOCKADDR_6_T addr6;
    unsigned char* endmark;
    A_UINT16 retry_counter = MAX_END_MARK_RETRY;

#if !ZERO_COPY
    if((stat_packet = A_MALLOC(sizeof(stat_packet_t),MALLOC_ID_CONTEXT)) == NULL)
    {
            printf("Out of memory error\n");
            return A_ERROR;
    }
#endif

    /* Create listen socket & Bind.*/
    if(!v6EnableFlag)
    {

            if((p_tCxt->sock_local = t_socket((void*)handle,ATH_AF_INET,SOCK_DGRAM_TYPE, 0)) == A_ERROR)
            {
                printf("ERROR:: Socket creation error.\r\n");
                goto ERROR_1;
            }

            memset(&local_addr, 0, sizeof(local_addr));
            local_addr.sin_port = p_tCxt->tx_params.port;
            local_addr.sin_family = ATH_AF_INET;


            if(t_bind((void*)handle,/*p_tCxt->sock_peer*/p_tCxt->sock_local, (&local_addr), sizeof(local_addr)) != A_OK)
            {
                printf("ERROR:: Socket bind error.\r\n");
                goto ERROR_2;
            }
    }
    else
    {

            if((p_tCxt->sock_local = t_socket((void*)handle,ATH_AF_INET6,SOCK_DGRAM_TYPE, 0)) == A_ERROR)
            {
                printf("ERROR:: Socket creation error.\r\n");
                goto ERROR_1;
            }

            memset(&local_addr6, 0, sizeof(local_addr6));
            local_addr6.sin6_port = p_tCxt->tx_params.port;
            local_addr6.sin6_family = ATH_AF_INET6;


            if(t_bind((void*)handle,/*p_tCxt->sock_peer*/p_tCxt->sock_local, (&local_addr6), sizeof(local_addr6)) != A_OK)
            {
                printf("ERROR:: Socket bind error.\r\n");
                goto ERROR_2;
            }
    }


     while(retry_counter)
     {
            while((endmark = CUSTOM_ALLOC(sizeof(EOT_PACKET))) == NULL)
            {
              /*Allow small delay to allow other thread to run*/
              app_time_delay(SMALL_TX_DELAY);
            }
            /* Send End mark.*/

            ((EOT_PACKET*)endmark)->code = HOST_TO_LE_LONG(END_OF_TEST_CODE);
            //((EOT_PACKET*)endmark)->packet_count = HOST_TO_LE_LONG(cur_packet_number);
	    if(!v6EnableFlag)
            {
	          t_sendto((void*)handle, p_tCxt->sock_peer, (unsigned char*)(endmark), sizeof(EOT_PACKET), 0,(&foreign_addr), sizeof(foreign_addr)) ;
            }
            else
            {
	          t_sendto((void*)handle, p_tCxt->sock_peer, (unsigned char*)(endmark), sizeof(EOT_PACKET), 0,(SOCKADDR_T *)(&foreign_addr6), sizeof(foreign_addr6)) ;
            }

#if !NON_BLOCKING_TX
            /*Free the buffer only if NON_BLOCKING_TX is not enabled*/
            CUSTOM_FREE(endmark);
#endif

	/* block for xxx msec or until activity on socket */
	if(A_OK == t_select((void*)handle,p_tCxt->sock_local/*p_tCxt->sock_peer*/, 200)){
		/* Receive data */
#if ZERO_COPY
            if(!v6EnableFlag)
            {
                addr_len = sizeof(SOCKADDR_T);
                received = t_recvfrom((void*)handle, /*p_tCxt->sock_peer*/p_tCxt->sock_local, (void**)(&stat_packet), sizeof(stat_packet_t), 0,&addr, &addr_len );
            }
            else
            {
                addr_len = sizeof(SOCKADDR_6_T);
                received = t_recvfrom((void*)handle, /*p_tCxt->sock_peer*/p_tCxt->sock_local, (void**)(&stat_packet), sizeof(stat_packet_t), 0,&addr6, &addr_len );
            }
#else
            if(!v6EnableFlag)
            {
                addr_len = sizeof(SOCKADDR_T);
                received = t_recvfrom((void*)handle, /*p_tCxt->sock_peer*/p_tCxt->sock_local, (char*)(stat_packet), sizeof(stat_packet_t), 0,&addr, &addr_len );
            }
            else
            {
                addr_len = sizeof(SOCKADDR_6_T);
                received = t_recvfrom((void*)handle, /*p_tCxt->sock_peer*/p_tCxt->sock_local, (char*)(stat_packet), sizeof(stat_packet_t), 0,&addr6, &addr_len );
            }
#endif
	}
        if(received == sizeof(stat_packet_t))
        {
            printf("received statistics\n");
            error = A_OK;

            /*Response received from peer, extract test statistics*/
            stat_packet->msec = HOST_TO_LE_LONG(stat_packet->msec);
            stat_packet->kbytes = HOST_TO_LE_LONG(stat_packet->kbytes);
            stat_packet->bytes = HOST_TO_LE_LONG(stat_packet->bytes);
            stat_packet->numPackets = HOST_TO_LE_LONG(stat_packet->numPackets);

            p_tCxt->first_time.SECONDS = p_tCxt->last_time.SECONDS = 0;
            p_tCxt->first_time.MILLISECONDS = 0;
            p_tCxt->last_time.MILLISECONDS = stat_packet->msec;
            p_tCxt->bytes = stat_packet->bytes;
            p_tCxt->kbytes = stat_packet->kbytes;
            break;
        }
        else
        {
            error = A_ERROR;
            retry_counter--;
            //printf("Did not receive response\n");
        }
    }

#if ZERO_COPY
    if(received > 0)
        zero_copy_free(stat_packet);
#endif

ERROR_2:
#if !ZERO_COPY
    if(stat_packet)
       A_FREE(stat_packet,MALLOC_ID_CONTEXT);
#endif
  t_shutdown((void*)handle,p_tCxt->sock_local);

ERROR_1:
 return error;

}


/************************************************************************
* NAME: handle_mcast_param
*
* DESCRIPTION: Handler for multicast parameters in UDp Rx test
* Parameters: pointer to throughput context
************************************************************************/
int_32 handle_mcast_param(THROUGHPUT_CXT *p_tCxt)
{

	 int ip_hdr_inc  =0;
	if(p_tCxt->rx_params.ip_hdr_inc == 1)
	{
		    printf("set header include option %d \r\n",p_tCxt->rx_params.ip_hdr_inc);
		    ip_hdr_inc = A_CPU2BE32(p_tCxt->rx_params.ip_hdr_inc);
	            if(t_setsockopt((void*)handle, p_tCxt->sock_local, ATH_IPPROTO_IP, IP_HDRINCL,
	                             (A_UINT8*)(&(ip_hdr_inc)),sizeof(int)) != A_OK)
	            {
	                    printf("SetsockOPT error : unable to set ip hdr inc\r\n");
	                    return A_ERROR;
	            }

	}	
	if(!v6EnableFlag)
	{
		if(p_tCxt->rx_params.group.imr_multiaddr != 0)
		{

		    p_tCxt->rx_params.group.imr_multiaddr = A_CPU2BE32(p_tCxt->rx_params.group.imr_multiaddr);
		    p_tCxt->rx_params.group.imr_interface = A_CPU2BE32(p_tCxt->rx_params.group.imr_interface);
	            if(t_setsockopt((void*)handle, p_tCxt->sock_local, ATH_IPPROTO_IP, IP_ADD_MEMBERSHIP,
	                             (A_UINT8*)(&(p_tCxt->rx_params.group)),sizeof(IP_MREQ_T)) != A_OK)
	            {
	                    printf("SetsockOPT error : unable to add to multicast group\r\n");
	                    return A_ERROR;
	            }
		}
	}
	else
	{
		if(p_tCxt->rx_params.v6mcastEnabled)
		{
			if(t_setsockopt((void*)handle,p_tCxt->sock_local, ATH_IPPROTO_IP, IPV6_JOIN_GROUP,
			     (A_UINT8*)(&(p_tCxt->rx_params.group6)),sizeof(struct ipv6_mreq)) != A_OK)
			{
				printf("SetsockOPT error : unable to add to multicast group\r\n");
	                    return A_ERROR;
			}
		}
	}
	return A_OK;
}




/************************************************************************
* NAME: sendAck
*
* DESCRIPTION: In UDP receive test, the test is terminated on receiving
* end-mark (single byte packet). We have implemented a feedback mechanism
* where the Client will reply with receive stats allowing Peer to display correct
* test results. The Ack packet will contain time duration and number of bytes
* received. Implementation details-
* 1. Peer sends endMark packet, then waits for 500 ms for a response.
* 2. Client, on receiving endMark, sends ACK (containing RX stats), and waits for
*    1000 ms to check for more incoming packets.
* 3. If the Peer receives this ACK, it will stop sending endMark packets.
* 4. If the client does not see the endMark packet for 1000 ms, it will assume SUCCESS
*    and exit gracefully.
* 5. Each side makes 20 attempts.
* Parameters: pointer to throughput context, specified address, specified port
************************************************************************/

void sendAck(THROUGHPUT_CXT *p_tCxt, A_VOID * address, uint_16 port)
{
    int send_result;
    uint_32 received = 0;
    uint_32 addr_len=0;
    uint_16 retry = MAX_ACK_RETRY;
    stat_packet_t* statPacket;
    uint_32 sec_interval = (p_tCxt->last_time.SECONDS - p_tCxt->first_time.SECONDS);
    uint_32 ms_interval = (p_tCxt->last_time.MILLISECONDS - p_tCxt->first_time.MILLISECONDS);
    uint_32 total_interval = sec_interval*1000 + ms_interval;
    SOCKADDR_T* addr = (SOCKADDR_T*)address;
    SOCKADDR_6_T *addr6 = (SOCKADDR_6_T *)address;

    if(!v6EnableFlag)
    {
      addr_len = sizeof(addr);
    }
    else
    {
      addr_len = sizeof(addr6);
    }

    while(retry)
    {

       while((statPacket = (stat_packet_t*)CUSTOM_ALLOC(sizeof(stat_packet_t))) == NULL)
      {
        /*Allow small delay to allow other thread to run*/
         app_time_delay(SMALL_TX_DELAY);
         if(test_for_quit()){
           return;
         }
      }

      statPacket->kbytes = HOST_TO_LE_LONG(p_tCxt->kbytes);
      statPacket->bytes = HOST_TO_LE_LONG(p_tCxt->bytes);
      statPacket->msec = HOST_TO_LE_LONG(total_interval);

	if(!v6EnableFlag)
        {
            addr->sin_port = port;
	    send_result = t_sendto((void*)handle, p_tCxt->sock_local, (unsigned char*)(statPacket), sizeof(stat_packet_t), 0,addr,addr_len);
        }
        else
        {
            addr6->sin6_port = port;
	    send_result = t_sendto((void*)handle, p_tCxt->sock_local, (unsigned char*)(statPacket), sizeof(stat_packet_t), 0,(SOCKADDR_T *)addr6,addr_len);
        }

#if !NON_BLOCKING_TX
            /*Free the buffer only if NON_BLOCKING_TX is not enabled*/
            CUSTOM_FREE(statPacket);
#endif
	    if ( send_result == A_ERROR )
	    {
	        printf("error while sending stat packet\n");
	        break;
	    }
	    else
	    {
	    	if(t_select((void*)handle, p_tCxt->sock_local,1000) == A_OK)
	    	{
#if ZERO_COPY
                    if(!v6EnableFlag)
                    {
                        received = t_recvfrom((void*)handle, p_tCxt->sock_local,
            					(void**)(&p_tCxt->buffer),
            					CFG_PACKET_SIZE_MAX_RX, 0,
                				addr, &addr_len);
                    }
                    else
                    {
                        received = t_recvfrom((void*)handle, p_tCxt->sock_local,
            					(void**)(&p_tCxt->buffer),
            					CFG_PACKET_SIZE_MAX_RX, 0,
                				addr6, &addr_len);
                    }
                    zero_copy_free(p_tCxt->buffer);
#else
	    	    if(!v6EnableFlag)
                    {
	    		received = t_recvfrom((void*)handle, p_tCxt->sock_local,
            					(char*)(&p_tCxt->buffer[0]),
            					CFG_PACKET_SIZE_MAX_RX, 0,
                				addr, &addr_len);
                    }
                    else
                    {
	    		received = t_recvfrom((void*)handle, p_tCxt->sock_local,
            					(char*)(&p_tCxt->buffer[0]),
            					CFG_PACKET_SIZE_MAX_RX, 0,
                				(SOCKADDR_T *)addr6, &addr_len);
                    }
#endif
               	    printf("received %d\n",received);
	    	}
	    	else
	    	{
	    	    printf("ACK success\n");
            	    break;
	    	}

    	}
	    retry--;
    }
}






#if 0

/************************************************************************
* NAME: ath_tcp_rx
*
* DESCRIPTION: Start throughput TCP server.
************************************************************************/
void ath_tcp_rx_multi_TCP (int port)
{
    A_UINT8 ip_str[16];
    A_UINT16 addr_len=0;
    A_UINT16 open_connections = 0;
    A_INT32 conn_sock;
    _ip_address temp_addr;
    SOCKADDR_T local_addr;
    SOCKADDR_T foreign_addr;
    int i, call_listen = 1;
    THROUGHPUT_CXT tCxt;
    THROUGHPUT_CXT *p_tCxt = &tCxt;
    int incoming_socket[3] = {0};

    bench_quit = 0;
    p_tCxt->sock_peer = A_ERROR;

#if !ZERO_COPY
    /*Allocate buffer*/
    if((p_tCxt->buffer = A_MALLOC(CFG_PACKET_SIZE_MAX_RX, MALLOC_ID_CONTEXT)) == NULL)
    {
            printf("Out of memory error\n");
             goto ERROR_1;
    }
#endif


    /* Create listen socket */
    if((p_tCxt->sock_local = t_socket((void*)handle, ATH_AF_INET, SOCK_STREAM_TYPE, 0)) == A_ERROR)
    {
        printf("ERROR: Socket creation error.\r\n");
        goto ERROR_1;
    }

    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_port = port;
    local_addr.sin_family = ATH_AF_INET;

     /* Bind socket.*/
    if(t_bind((void*)handle, p_tCxt->sock_local, &local_addr, sizeof(local_addr)) == A_ERROR)
    {
        printf("ERROR: Socket bind error.\r\n");
        goto ERROR_2;
    }


    /* ------ Start test.----------- */
    printf("****************************************************\n");
    printf(" TCP RX Test\n" );
    printf("****************************************************\n");

    printf( "Local port %d\n", port);
    printf("Type benchquit to terminate test\n");
    printf("****************************************************\n");

    app_time_get_elapsed(&p_tCxt->first_time);
    app_time_get_elapsed(&p_tCxt->last_time);
    p_tCxt->last_bytes = 0;
    p_tCxt->last_kbytes = 0;

     printf("Waiting.\n");
       
        while(1)
        {
            if(test_for_quit()){
                goto tcp_rx_QUIT;
            }
            
            if(call_listen){
              /* Listen. */
              if(t_listen((void*)handle, p_tCxt->sock_local, 1) == A_ERROR)
              {
                  printf("ERROR: Socket listen error.\r\n");
                  goto ERROR_2;
              }
              call_listen = 0;
            }
            /* block for 100msec or until a connection is received */
            conn_sock = t_select((void*)handle, p_tCxt->sock_local,100);
    
            if(conn_sock == A_SOCK_INVALID)
              goto tcp_rx_QUIT;       //Peer closed connection, socket no longer valid
            
            if(conn_sock == A_OK){
                /*Accept incoming connection*/
                addr_len = sizeof(foreign_addr);
                incoming_socket[open_connections] = t_accept((void*)handle, p_tCxt->sock_local, &foreign_addr, addr_len);
                
                if(incoming_socket[open_connections] != A_ERROR)
                {
               
                    temp_addr = LONG_BE_TO_HOST(foreign_addr.sin_addr);
                    printf("Receiving from %s:%d\n", inet_ntoa(*(A_UINT32 *)(&temp_addr), (char *)ip_str), foreign_addr.sin_port);
                    open_connections++;
                    call_listen = 1;
                }
            }
              
            if(open_connections){
                if(receive_incoming(p_tCxt,incoming_socket, &open_connections) != A_OK)
                  break;
            }           
        }      

tcp_rx_QUIT:
   
  for(i=0;i<open_connections; i++){

    t_shutdown((void*)handle, incoming_socket[i]);
  }
ERROR_2:
    t_shutdown((void*)handle, p_tCxt->sock_local);

ERROR_1:

    printf("Shell> ");

#if !ZERO_COPY
    if(p_tCxt->buffer)
    	A_FREE(p_tCxt->buffer,MALLOC_ID_CONTEXT);
#endif
}



A_INT32 receive_incoming(THROUGHPUT_CXT *p_tCxt, A_UINT32* incoming_connections, A_UINT16* num_connections)
{
    int i=0, conn_sock;
    A_STATUS result = A_OK;
    int received = 0;
    
    for(i=0; i< 3; i++) /* Receiving data.*/
    {
        if(test_for_quit()){
            result = A_ERROR;
            break;
        }
        if(incoming_connections[i] > 0){
            /*Check for packet*/
            conn_sock = t_select((void*)handle,incoming_connections[i],100);
    
            if(conn_sock == A_OK)
            {
               /*Packet is available, receive it*/
    #if ZERO_COPY
                received = t_recv((void*)handle, incoming_connections[i], (void**)(&p_tCxt->buffer), CFG_PACKET_SIZE_MAX_RX, 0);
                if(received > 0)
                  zero_copy_free(p_tCxt->buffer);
    #else
                received = t_recv((void*)handle, incoming_connections[i], (char*)(&p_tCxt->buffer[0]), CFG_PACKET_SIZE_MAX_RX, 0);
    #endif
                if(received == A_SOCK_INVALID)
                {
                    /*socket closed*/
                    printf("Socket %d closed\n",incoming_connections[i]);
                    incoming_connections[i] = 0;
                    *num_connections--;
                    break;
                }
            }
    
            if(conn_sock == A_SOCK_INVALID)
            {
                printf("Socket %d closed\n",incoming_connections[i]);
                incoming_connections[i] = 0;
                *num_connections--;
                break;
            }
            else
            {
                /*Valid packaet received*/
                printf("Rx of socket %d\n",incoming_connections[i]);
               
            }
        } 
    }
    
    return result;
}

#endif

#if ENABLE_HTTP_CLIENT
/*
Command will be
httpc get 192.1.168.1.10 /index.html 
*/
/************************************************************************
* NAME: httpc_get_page
*
* DESCRIPTION:
************************************************************************/
HTTPC_PARAMS      httpc; // Since stack size is less

A_INT32 httpc_command_parser(A_INT32 argc, char_ptr argv[] )
{ /* Body */
    A_INT32           return_code = A_OK;
    	    
    if (argc < 3)
    {
        /*Incorrect number of params, exit*/
        return_code = A_ERROR;
    }
    else
    {
        memset ((void *)&httpc, 0, sizeof(HTTPC_PARAMS));
        
        if(strlen((char*)httpc.url) >= 64)
        {
            printf("Maximum 64 bytes supported as argument\n");
            return A_ERROR;
        }
        
        if (argv[3])
            strcpy((char*)httpc.url, argv[3]);
        
        if (argv[4])
            strcpy((char*)httpc.data, argv[4]);
        
        if(ATH_STRCMP(argv[2], "connect") == 0)
        {
            httpc.command = 0;
            if(argc >= 4)  
                httpc_connect(&httpc);
            else
                return_code = A_ERROR;
        }
        else if(ATH_STRCMP(argv[2], "get") == 0)
        {
            httpc.command = 1;
            if(argc >= 4)  
                httpc_method(&httpc);
            else
                return_code = A_ERROR;
        }
        else if(ATH_STRCMP(argv[2], "post") == 0)
        {
            httpc.command = 2;
            if(argc >= 4)  
                httpc_method(&httpc);
            else
                return_code = A_ERROR;
        }
        else if(ATH_STRCMP(argv[2], "query") == 0)
        {
            httpc.command = 3;
            if(argc >= 5)  
                httpc_method(&httpc);
            else
                return_code = A_ERROR;
        }
        else if(ATH_STRCMP(argv[2], "disc") == 0)
        {
            httpc.command = 4;  
            httpc_connect(&httpc);
        }
        else
        {
            printf("Unknown Command \"%s\"\n", argv[2]);
            return_code = A_ERROR;
        }
    }
    if (return_code == A_ERROR)
    {
        printf ("USAGE: wmiconfig --ip_http_client [<connect/get/post/query> <data1> <data2>]\n");
    }
    return return_code;
} /* Endbody */

/************************************************************************
* NAME: httpc_connect
*
* DESCRIPTION: Process a HTTP connect request.
************************************************************************/

static void httpc_connect(HTTPC_PARAMS *p_httpc)
{
    int error = A_OK;
     
    //printf("Sedning command %s %s\n", p_httpc->url, p_httpc->data);
    error = custom_httpc_method((void*) handle, p_httpc->command, p_httpc->url, p_httpc->data, NULL);
    
    printf("HTTPClient cmd %s\n", (error == A_ERROR)?"failed":"succeeded");
}

/************************************************************************
* NAME: httpc_method
*
* DESCRIPTION: Process a HTTP connect request.
*              For request, output will come as last argument
*              Application must free the buffer using zero_copy_http_free
************************************************************************/
static void httpc_method(HTTPC_PARAMS *p_httpc)
{
    A_UINT8   *output = NULL;
    A_UINT32  error = A_OK;
    HTTPC_RESPONSE *temp = NULL;
    int i;
    
    error = custom_httpc_method((void*) handle, p_httpc->command, p_httpc->url, p_httpc->data, &output);
    
    if(error != A_OK)
        printf("HTTPC Command failed\n");
    else if(output != NULL)
    {
        temp = (HTTPC_RESPONSE *)output;
        
        /* For GET and POST alone, print the output */
        if ((p_httpc->command == 1) || (p_httpc->command == 2))
        {
            printf("Size:%u Resp_code:%u\n\n", temp->length, temp->resp_code);
            //printf("Flags %u Packet: %u\n", temp->flags, strlen(temp->data));
            if (temp->length)
            {
                printf("Packet:\n");
                for(i = 0; i < temp->length; i++)
                    printf("%c", temp->data[i]);
            }
        }
        
        zero_copy_http_free((A_VOID *)output);
    }
    
    return;
}

#endif /* ENABLE_HTTP_CLIENT */


#if ENABLE_SSL

A_INT32 ssl_get_cert_handler(A_INT32 argc, char* argv[])
{
    A_INT32 res = A_ERROR;
    DNC_CFG_CMD dnsCfg;
    DNC_RESP_INFO dnsRespInfo;
    SOCKADDR_T hostAddr;
    A_UINT32 socketHandle = 0;
    int reqLen;
    CERT_HEADER_T *req;
    CERT_HEADER_T *header;
    A_UINT8 *buf;
    int certNameLen, numRead = 0, index, port = 1443;
    char *host, *certName, *flashName = NULL;

    // Free certificate buffer if allocated
    if (ssl_cert_data_buf)
    {
      A_FREE(ssl_cert_data_buf, MALLOC_ID_TEMPORARY);
    }
    ssl_cert_data_buf_len = 0;


    // Parse the arguments
    if(argc < 3)
    {
        if (argc > 1)
        {
          printf("Incomplete parameters\n");
        }
        printf("Usage: %s <name> <host> -p <port -s <fname>\n", argv[0]);
        printf("  <name>  = Name of the certificate or CA list file to retrieve\n");
        printf("  <host>  = Host name or IP address of certificate server\n");
        printf("  <port>  = Optional TCP port number\n");
        printf("  <fname> = Optional file name used if certificate is stored in FLASH\n");
        return A_ERROR;
    }
    certName = argv[1];
    host = argv[2];
    for(index = 3; index < argc ; index++)
    {
        if(argv[index][0] == '-')
        {
            switch(argv[index][1])
            {
            case 'p':
                index++;
                port = atoi(argv[index]);
                break;
            case 's':
                index++;
                flashName = argv[index];
                break;
            default:
                printf("Unknown option: %s\n", argv[index]);
                return A_ERROR;
            }
        }
    }

    do
    {
        // resolve the IP address of the certificate server
        if (0 == ath_inet_aton(host, &dnsRespInfo.ipaddrs_list[0]))
        {
            if (strlen(host) >= sizeof(dnsCfg.ahostname))
            {
                printf("ERROR: host name too long\n");
                break;
            }
            strcpy((char*)dnsCfg.ahostname, host);
            dnsCfg.domain = ATH_AF_INET;
            dnsCfg.mode =  RESOLVEHOSTNAME;
            if (A_OK != custom_ip_resolve_hostname(handle, &dnsCfg, &dnsRespInfo))
            {
                printf("ERROR: Unable to resolve server name\r\n");
                break;
            }
        }

        // Create socket
        if((socketHandle = t_socket((void*)handle, ATH_AF_INET, SOCK_STREAM_TYPE, 0)) == A_ERROR)
        {
            printf("ERROR: Unable to create socket\n");
            break;
        }

        // Connect to certificate server
        memset(&hostAddr, 0, sizeof(hostAddr));
        hostAddr.sin_addr = dnsRespInfo.ipaddrs_list[0];
        hostAddr.sin_port = port;
        hostAddr.sin_family = ATH_AF_INET;
        res = t_connect((void*)handle, socketHandle, (&hostAddr), sizeof(hostAddr));
        if(res != A_OK)
        {
            printf("ERROR: Connection failed (%d).\n", res);
            break;
        }

        // Build and send request
        certNameLen = strlen(certName);
        reqLen = CERT_HEADER_LEN + certNameLen;
        req = (CERT_HEADER_T*) CUSTOM_ALLOC(reqLen);

        if (req == NULL)
        {
            printf("ERROR: Out of memory.\n");
            break;
        }
        req->id[0] = 'C';
        req->id[1] = 'R';
        req->id[2] = 'T';
        req->id[3] = '0';
        req->length = A_CPU2BE32(certNameLen);
        memcpy(&req->data[0], certName, certNameLen);
        res = t_send(handle, socketHandle, (A_UINT8*)req, reqLen, 0);
#if !NON_BLOCKING_TX
        /*Free the buffer only if NON_BLOCKING_TX is not enabled*/
        CUSTOM_FREE(req);
#endif
        if (res == A_ERROR )
        {
            printf("ERROR: send error = %d\n", res);
            break;
        }
        else if(res == A_SOCK_INVALID )
        {
           /*Socket has been closed by target due to some error, gracefully exit*/
           printf("ERROR: Socket closed unexpectedly\n");
           break;
        }
    } while (0);

    // Read the response
    do
    {
        res = t_select((void*)handle, socketHandle, CLIENT_WAIT_TIME);
        if(res == A_OK)
        {
            res = t_recv(handle, socketHandle, (void**)&buf, CFG_PACKET_SIZE_MAX_RX, 0);
            printf("RX: %d\n", res);
            if (res > 0)
            {
                if (ssl_cert_data_buf_len == 0)
                {
                    if (buf[0] != 'C' || buf[1] != 'R' || buf[2] != 'T')
                    {
                        printf("ERROR: Bad MAGIC received in header\n");
                        break;
                    }
                    header = (CERT_HEADER_T*)buf;
                    header->length =  A_BE2CPU32(header->length);
                    if (header->length == 0)
                    {
                        zero_copy_free(buf);
                        break;
                    }
                    ssl_cert_data_buf = A_MALLOC(header->length, MALLOC_ID_TEMPORARY);
                    if(ssl_cert_data_buf == NULL)
                    {
                        zero_copy_free(buf);
                        printf("ERROR: Out of memory error\n");
                        res = A_ERROR;
                        break;
                    }
                    ssl_cert_data_buf_len = header->length;
                    res -= OFFSETOF(CERT_HEADER_T, data);
                    memcpy(ssl_cert_data_buf, header->data, res);
                    numRead = res;
                }
                else
                {
                    if (res + numRead <= ssl_cert_data_buf_len)
                    {
                        memcpy(&ssl_cert_data_buf[numRead], buf, res);
                        numRead += res;
                        res = ssl_cert_data_buf_len;
                    }
                    else
                    {
                        zero_copy_free(buf);
                        printf("ERROR: read failed\n");
                        res = A_ERROR;
                        break;
                    }
                }
                zero_copy_free(buf);
            }
            else
            {
                printf("ERROR: no response\n");
                res = A_ERROR;
                break;
            }
        }
        else
        {
            if (res == A_SOCK_INVALID)
            {
                printf("ERROR: no response\n");
                res = A_ERROR;
                break;
            }
        }
    } while (numRead < ssl_cert_data_buf_len);

    if (socketHandle)
    {
        t_shutdown((void*)handle, socketHandle);
    }

    if (res == ssl_cert_data_buf_len)
    {
        printf("Received %d bytes from %s:%d\n", ssl_cert_data_buf_len, host, port);

        if (flashName != NULL)
        {
            // store certificate in FLASH
            if (A_OK == SSL_storeCert(flashName, ssl_cert_data_buf, ssl_cert_data_buf_len))
            {
                printf("'%s' is stored in FLASH\n", flashName);
            }
            else
            {
                printf("ERROR: failed to store in %s\n", flashName);
                res = A_ERROR;
            }
        }
    }
    return res;
}

#endif // ENABLE_SSL

#endif //ENABLE_STACK_OFFLOAD
