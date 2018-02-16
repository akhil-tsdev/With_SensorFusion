/*
 * Copyright TuringSense, Inc © 2015
 * hub_main_loop.c
 *
 *  Created on: May 15, 2015
 *      Author: cwati
 *
 *  Added cbuf handling.  But, right now it's categorized per
 *  timestamp.  Should add handling of cbuf to group data per
 *  10ms.
 *
 */

#include "main.h"
#include "common_types.h" /* Turing Sense specific */
#include "atheros_stack_offload.h"

#include "hub_to_nordic.h"
#include "throughput.h"

/*************************** GLOBAL ************************/

#define DEBUG				1

cloud_to_hub_t	cloud_to_hub;
hub_to_nordic_t hub_to_nordic;

nordic_to_hub_t nordic_to_hub;
hub_to_cloud_t hub_to_cloud;

hub_to_cloud_t packet_empty;

TIME_STRUCT		latest_rtc;
uint32_t 	cur_num_of_sat = 0;
uint32_t 	cur_sat_ids[MAX_SENSORS - 1]; /* currently valid satellite IDs */
static uint32_t	current_state = HUB_NOT_ACTIVE;
static uint32_t	hub_can_send = HUB_CANNOT_SEND;

cbuf_t cbuf;

THROUGHPUT_CXT  tCxt;
THROUGHPUT_CXT	*p_tCxt;

/***********************************************************/

static A_INT32 hub_init_send_params (THROUGHPUT_CXT *tCxt, A_INT32 argc, char_ptr argv[]) {
    A_INT32			retval = -1;
    A_INT32 max_packet_size = 0;
    A_INT32            return_code = A_OK;

	/* Port.*/
	if (argc == 2) {
		tCxt->tx_params.port = atoi(argv[1]);
	} else {
		tCxt->tx_params.port = atoi(default_destPort);
	}
	/* Packet size.*/
	tCxt->tx_params.packet_size = txSize;
	tCxt->tx_params.test_mode = txMode;
	tCxt->tx_params.packet_number = txPktNum;

	/* Inter packet interval for Bandwidth control*/
	tCxt->tx_params.interval = txInterval;
	tCxt->test_type = TCP_TX;

	return return_code;
}

/*
 * Function:	process_data_from_cloud
 * Parameters:	cloud_to_hub,
 * 					which is data received from cloud
 * Returns: 	none
 * Comments:	Process data from cloud and sets current status and sets status for Nordic.
 * 				Status for Nordic will be sent when we're talking to Nordic.
 */
void process_data_from_cloud(cloud_to_hub_t cloud_data) {

	uint32_t 	cmd = cloud_data.command;
	uint8_t		cnt;

	/* Processing received command */
	if (COMMAND_WIFI_START(cmd)) {
		current_state = HUB_ACTIVE;
		hub_to_nordic.command |= NORDIC_START;

#if DEBUG
		printf("Current state is HUB_ACTIVE\n");
#endif
	} else if (COMMAND_WIFI_STOP(cmd)) {
		current_state = HUB_NOT_ACTIVE;
		hub_to_nordic.command &= ~NORDIC_START;

#if DEBUG
		printf("Current state is HUB_NOT_ACTIVE\n");
#endif
	}

	if (COMMAND_WIFI_WAIT(cmd)) {
		hub_can_send = HUB_CANNOT_SEND;
#if DEBUG
		printf("Current hub_can_send is HUB_CANNOT_SEND\n");
#endif
	} else {
		hub_can_send = HUB_CAN_SEND;
#if DEBUG
		printf("Current hub_can_send is HUB_CAN_SEND\n");
#endif
	}

	if (COMMAND_WIFI_SET_SAT(cmd)) {
		cur_num_of_sat = cloud_data.payload.num_of_sat;
		hub_to_nordic.payload.num_of_sat = cloud_data.payload.num_of_sat;
#if DEBUG
		printf("Current num of sat: %d\n", hub_to_nordic.payload.num_of_sat);
#endif
		for (cnt = 0; cnt < cur_num_of_sat; cnt++) {
			cur_sat_ids[cnt] = cloud_data.payload.satellite_ids[cnt];
			hub_to_nordic.payload.satellite_ids[cnt] = cloud_data.payload.satellite_ids[cnt];
#if DEBUG
		printf("Sat[%d]: 0x%x\n", cnt, hub_to_nordic.payload.satellite_ids[cnt]);
#endif
		}
	}

	if (COMMAND_WIFI_SET_RTC(cmd)) {
		/* TODO cwati need to process RTC */
		hub_to_nordic.command |= NORDIC_SET_RTC;
		hub_to_nordic.payload.rtc_value = cloud_data.payload.rtc_value;

		/* Set our own RTC.  The cloud is sending in milliseconds,
		 * we set our own seconds/milliseconds values. */
		if (cloud_data.payload.rtc_value > 1000) {
			latest_rtc.SECONDS = cloud_data.payload.rtc_value / 1000;
			latest_rtc.MILLISECONDS = cloud_data.payload.rtc_value % 1000;

#if DEBUG
			printf("RTC received = %d.  Our latest SECONDS: %d MS: %d\n",
					cloud_data.payload.rtc_value,
					latest_rtc.SECONDS, latest_rtc.MILLISECONDS);
#endif
		}
	}
}


/*
 * Function: 			talk_to_wifi
 * Parameters:			none
 * Returns:				A_OK, or error codes
 * Comments:
 * 		This function will receive command from Wifi.
 * 		It will call a function to process the command.
 * 		Then if it can send, it will send.
 *
 */
A_INT32 talk_to_wifi (void) {
    A_INT32 		received, send_result;
	uint32_t		cnt, timeout = 10;
	static uint32_t	sent_package = 0;
	hub_to_cloud_t	tmp_cbuf_to_cloud;
    A_INT32 		conn_sock;

#if !ZERO_COPY
	if((tCxt.buffer = A_MALLOC(CFG_PACKET_SIZE_MAX_RX,MALLOC_ID_CONTEXT)) == NULL)
	{
		printf("Out of memory error\n");
		goto ERROR;
	}
#endif

#if DEBUG
	printf("I am trying to hear from server...\n");
#endif

//    conn_sock = t_select((void*)handle, p_tCxt->sock_peer, TCP_CONNECTION_WAIT_TIME);
//
//    if(conn_sock == A_OK)
//    {
#if ZERO_COPY
		received = t_recv((void*)handle, tCxt.sock_peer, (char**)(&(tCxt.buffer)), sizeof(cloud_to_hub_t), 0);
#else
		received = t_recv((void*)handle, tCxt[loop].sock_peer, (char*)(&tCxt[loop].buffer[0]), CFG_PACKET_SIZE_MAX_RX, 0);
#endif
		if (received == A_ERROR) {
#if DEBUG
			printf("No data received yet...\n");
#endif
		} else if (received == A_SOCK_INVALID) {
			printf("Invalid socket...\n");
			/* TODO CWATI need to figure out when server has closed socket.
			 * Right now server must close connection before ending. */
		} else if (received < 0) {
			printf("Other receive error...\n");
		} else if (received > 0) { /* This should receive some good value */

			/* Process incoming data */
	//		uint8_t	*ptr8 =(uint8_t *) &cloud_to_hub;
	//		for (uint8_t size = 0; size < sizeof(cloud_to_hub_t); size++) {
	//			*(ptr8 + size) = *(tCxt.buffer + size);
	//		}
			memcpy(&cloud_to_hub, tCxt.buffer, sizeof(cloud_to_hub_t));
			printf("Received %d bytes out of expected %d bytes\n", received, sizeof(cloud_to_hub_t));

#if DEBUG
		printf("Received command from cloud: 0x%x\n", cloud_to_hub.command);
		printf("Received RTC from cloud: 0x%x\n", cloud_to_hub.payload.rtc_value);
#endif
			process_data_from_cloud(cloud_to_hub);

#if ZERO_COPY
			zero_copy_free(tCxt.buffer);
#endif
		}

		if (hub_can_send == HUB_CAN_SEND) {
			/* Send message to the server */
#if DEBUG
			printf("Current state is active.  I am sending message to the server...\n");
#endif

			/* CWATI TODO should I alloc this every time? Seems like a waste of time */
			while((p_tCxt->buffer = CUSTOM_ALLOC(txSize)) == NULL)
			{
				/* Wait till we get a buffer */
				for (cnt = 0; cnt < timeout; cnt++) {
					app_time_delay(SMALL_TX_DELAY);
				}
			}

			if (cnt == timeout) {
				printf("ERROR Gasp! Alloc failed!\n");
				return A_NO_MEMORY;
			}

			/* Check cbuf */
			if (cbufIsEmpty(&cbuf)) {
#if DEBUG
				memcpy(&tmp_cbuf_to_cloud, &packet_empty, sizeof(packet_empty));
				printf("Cbuf is empty!\n");
#endif
				memcpy(p_tCxt->buffer, &packet_empty, sizeof(packet_empty));
			} else {
				cbufPop(&cbuf, &tmp_cbuf_to_cloud);
#if DEBUG
				printf("cbuf element is now: %d\n", cbufNum(&cbuf));
#endif
				memcpy(p_tCxt->buffer, &tmp_cbuf_to_cloud, sizeof(tmp_cbuf_to_cloud));
			}

			send_result = t_send((void*)handle, p_tCxt->sock_peer, (unsigned char*)(p_tCxt->buffer), txSize, 0);

#if !NON_BLOCKING_TX
			/*Free the buffer only if NON_BLOCKING_TX is not enabled*/
			if (p_tCxt->buffer)
				CUSTOM_FREE(p_tCxt->buffer);
#endif
			if (send_result != txSize) {
				printf("Sent seems to have failed, bytes sent: %d expected: %d\n", send_result, txSize);
			}
			/* The driver returns num of bytes sent.  But if error happens, it also sends positive numbers.
			 * This is a major bug in the driver.
			 */
			else if(send_result == txSize)
			{
#if DEBUG
			printf("Here is the message: Timestamp 0x%x\n"
				  "                     bitmap    0x%x\n"
				  "                     0.satID   0x%x\n"
				  "                     0.data[0]   %f\n"
				  "                     0.data[1]   %f\n"
				  "                     0.data[2]   %f\n"
				  "                     0.data[3]   %f\n",
				  tmp_cbuf_to_cloud.timestamp, tmp_cbuf_to_cloud.data[0].satellite_id,
				  tmp_cbuf_to_cloud.data[0].record.quat_w,
				  tmp_cbuf_to_cloud.data[0].record.quat_x,
				  tmp_cbuf_to_cloud.data[0].record.quat_y,
				  tmp_cbuf_to_cloud.data[0].record.quat_z);

			sent_package++;
			printf("Sent this many bytes: %d.  Total packages sent: %d\n",send_result, sent_package);
#endif
			}
		}
//#if DEBUG
//		else {
//			printf("Current state hub_can_send = 0x%x\n", hub_can_send);
//		}
//#endif
//    } else {
//    	printf("t_select is not A_OK\n");
//    }

	return A_OK;
}

/*
 * Initializing connection to Wifi.
 *
 * Returns A_OK if connection is established.
 *
 */
A_INT32 init_wifi (A_INT32 argc, char_ptr argv[])
{
    boolean     print_usage=0;
    A_INT32     return_code = A_OK;
    A_INT32     retval = -1;
    A_INT32 	max_packet_size = 0;
    A_UINT16 	addr_len=0;
    SOCKADDR_T 	local_addr, foreign_addr;
    A_UINT32 	dummy32 = 0, tv = 2000 /* 2 seconds */;
    A_INT32 	conn_sock;

    _ip_address temp_addr;
    char ip_str[16];
    uint8_t			i, timeout;

    if (argc != 2) {
    	printf("Warning!! Will be using server port %s\n"
    			"usage: starttx <dest port>\n", default_destPort);
    }

    memset(&tCxt, 0, sizeof(tCxt));
    p_tCxt = &tCxt;
	hub_init_send_params(p_tCxt, argc, argv);

	/* CWATI TODO this will be replaced with data from Nordic */
	packet_empty.timestamp = 0x01234567;
	packet_empty.data[0].record.quat_w = 0.0;
	packet_empty.data[0].record.quat_x = 0.1;
	packet_empty.data[0].record.quat_y  = 0.2;
	packet_empty.data[0].record.quat_z  = 0.3;

	/* Printing to shell */
//    temp_addr = LONG_BE_TO_HOST(p_tCxt->tx_params.ip_address);
//    memset(ip_str, 0, sizeof(ip_str));
//    printf("Remote IP addr. %s\n", inet_ntoa(*(A_UINT32 *)( &temp_addr), ip_str));
    printf("Remote port %d\n", p_tCxt->tx_params.port);
    printf("Message size %d\n", p_tCxt->tx_params.packet_size);
    printf("Number of messages %d\n", p_tCxt->tx_params.packet_number);

    if((p_tCxt->sock_local = t_socket((void*)handle, ATH_AF_INET, SOCK_STREAM_TYPE, 0)) == A_ERROR)
    {
        printf("ERROR: Unable to create socket\n");
        return A_ERROR;
    }
#if DEBUG
    else {
    	printf("Create socket OK...\n");
    }
#endif

    /*Allow small delay to allow other thread to run*/
    app_time_delay(TX_DELAY);

    printf("Connecting.\n");

    memset(&local_addr, 0, sizeof(local_addr));
//    local_addr.sin_addr = p_tCxt->tx_params.ip_address;
    local_addr.sin_addr = INADDR_ANY;
    local_addr.sin_port = p_tCxt->tx_params.port;
    local_addr.sin_family = ATH_AF_INET;

    /* Bind socket.*/
    return_code = t_bind((void*)handle, p_tCxt->sock_local, &local_addr, sizeof(local_addr));
    if	(return_code != A_OK)
	{
	   printf("ERROR: Socket bind %d.\r\n", return_code);
	   return return_code;
	}
#if DEBUG
    else {
    	printf("Bind success!\n");
    }

    printf("Listening to connection on port %d\n", p_tCxt->tx_params.port);
#endif

    i = 0;
    timeout = 60;
    while (1) {
		/* Listen. */
		return_code = t_listen((void*)handle, p_tCxt->sock_local, 1);
		if (return_code == A_ERROR)
		{
			printf("ERROR: Socket listen error. ERR:%d\r\n", return_code);
			return return_code;
		}

        do
        {
            /* block for 50 msec or until a packet is received */
            conn_sock = t_select((void*)handle, p_tCxt->sock_local, TCP_CONNECTION_WAIT_TIME);

            if(conn_sock == A_SOCK_INVALID) {
              printf("Peer sock closed connection.  Socket is no longer valid. Exiting...\n");
              return A_ERROR;
            }

        } while(conn_sock == A_ERROR);


        addr_len = sizeof(foreign_addr);

        /* Must call t_select before t_accept for this driver */
        p_tCxt->sock_peer = t_accept((void*)handle, p_tCxt->sock_local, &foreign_addr, addr_len);

		if (tCxt.sock_peer != A_ERROR) {
#if DEBUG
			printf("new socket 0x%x\n", tCxt.sock_peer);
#endif
			memset(ip_str, 0, sizeof(ip_str));
			temp_addr = LONG_BE_TO_HOST(foreign_addr.sin_addr);
			printf("Accept connection from %s port %d\n", inet_ntoa(*(A_UINT32 *)(&temp_addr), (char *)ip_str), foreign_addr.sin_port);
			printf("Connection established!\n");
			return_code = A_OK;
			break;
		}

		if (++i == timeout) {
			return_code = A_ERROR;
			break;
		}
		app_time_delay(1000);	/* Wait, in milliseconds */
    }

    return return_code;
}

/*TASK*-------------------------------------------------------------------
*
* Task Name : hub_main_loop
* Comments  : Called when user types in "starttx"
*
*END*----------------------------------------------------------------------*/
A_INT32 hub_main_loop(A_INT32 argc, char* argv[] )
{
    A_INT32                 return_code;

	init_nordic();
	cbufInit(&cbuf);

	if(strcmp(argv[0], "starttx") == 0){

		if(wifi_connected_flag) {
			printf("Wifi connection OK\n");
		} else {
			printf("ERROR: No WiFi connection available, please connect to an Access Point\n");
			return A_ERROR;
		}
	}

	latest_rtc.MILLISECONDS = 0;
	latest_rtc.SECONDS = 0;

	/* Start sending to host */
	return_code = init_wifi(argc, argv);
	if (return_code != A_OK) {
		/* Failed to establish connection to Wifi! */
		printf("Failed to initialize connection to Access Point!\n");
		return return_code;
	}

	/* TODO remove this.  This is just for temporary testing with Nordic. */
	cur_sat_ids[0] = 0xAA001122;
	cur_num_of_sat = 1;


	while (1) {
		talk_to_wifi();

		talk_to_nordic(&nordic_to_hub);

		app_time_delay(3000);	/* Wait in millisecond */

	}

    t_shutdown((void*)handle, p_tCxt->sock_peer);

	return return_code;
}
