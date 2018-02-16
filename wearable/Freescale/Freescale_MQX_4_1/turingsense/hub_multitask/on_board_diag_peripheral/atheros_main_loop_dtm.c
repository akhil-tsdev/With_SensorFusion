/*
 * Copyright TuringSense, Inc © 2015-2015
 * atheros_main_loop_dtm.c
 *
 *  Created on: June 25, 2016
 *      Author: cwati
 *
 *
 */

#include <mqx.h>
#include "atheros_stack_offload.h"
#include "common_types.h" /* Turing Sense specific */
#include "hub_main_loop_dtm.h"
#include "main.h"
#include "throughput.h"
#include "wmiconfig_ts.h"

#define DEBUG0				1		/* Log */

/* 100 Hz --> wait 3 cycles
   Wait this many times for bitmap completion. */
const uint8_t		wait_for_complete_max= (100 / SAT_SENDING_FREQ) * 3;
/* For 100Hz -> group per 10ms. */
const uint32_t		timestamp_group = (100 / SAT_SENDING_FREQ) * 10;

#if WIFI_DIRECT
#define				SSID_NAME	    "h32_cecylMAG"
#define				SSID_PASSWORD	    "hubhubhub"
/* This is the default setting from Atheros.  We're not going to change them
 *  #define				MY_IP			"192.168.1.10"
 *  #define				MY_NETMASK		"255.255.255.0"
 *  #define				MY_GATEWAY		"192.168.1.1"
 *
 *  The first device connected to this hub's SoftAP will get IP:
 *  192.168.1.100, then
 *  192.168.1.101, etc..
 */
#define				MY_PORT			12345
#define				MY_IP			"192.168.1.10"
#define				MY_NETMASK		"255.255.255.0"
#define				MY_GATEWAY		"192.168.1.1"

#else

//EU Mobile Wifi
//#define				SSID_NAME		"TSwind"
//#define				SSID_PASSWORD	"Kinetis22F"
//#if ENABLE_MAG
//#define				MY_IP			"192.168.0.117"
//#else
//#define				MY_IP			"192.168.0.118"
//#endif
//#define				MY_NETMASK		"255.255.255.0"
//#define				MY_GATEWAY		"192.168.0.255"

//TS wifi
//#define				SSID_PASSWORD	"Kinetis22F"
//#define				SSID_NAME		"TSWIFI"
//#define				MY_IP			"192.168.1.117"
//#define				MY_NETMASK		"255.255.255.0"
//#define				MY_GATEWAY		"192.168.1.255"

//cw test
//#define				SSID_PASSWORD	"engkong3780EMAK@"
//#define				SSID_NAME		"LG505"
//#define				MY_IP			"192.168.1.117"
//#define				MY_NETMASK		"255.255.255.0"
//#define				MY_GATEWAY		"192.168.1.255"

#define				MY_PORT			12345
#endif /* WIFI_DIRECT */

/* DTM Testing.  Configurable Wifi settings. */
static uint32_t                         wifi_channel = 3;
static uint8_t                          wifi_direct = true;
static uint8_t                          enable_wifi = true;
static char                             ssid_name[MAX_SSID_LENGTH] = "Turingsense DTM";
static char                             ssid_pass[MAX_PASSPHRASE_SIZE] = "turingsense";
static char                             my_ip[20] = MY_IP;
static char                             my_netmask[20] = MY_NETMASK;
static char                             my_gateway[20] = MY_GATEWAY;
static unsigned short                   my_port = MY_PORT;
static uint32_t                         transmit_power = 1; /* dBm */
static char                             phy_mode[5] = {'b','\0','\0','\0','\0'};

const uint32_t wifi_timeout = 20; /* seconds */
uint32_t ack_to_client = 0x0;	/* Latest RTC from satellites */

/* Synchronization variables among tasks */
MUTEX_STRUCT cbuf_mutex;
LWEVENT_STRUCT atheros_task_event;
uint_8 hvac_init = 0;

TS_THROUGHPUT_CXT tCxt;
TS_THROUGHPUT_CXT *p_tCxt;

TIME_STRUCT latest_rtc;

err_t mutex_cbuf_pop(cbuf_t *cb, CB_TYPE *pkeyval) {
	err_t ret;

	_mutex_lock(&cbuf_mutex);
	ret = cbufPop(cb, pkeyval);
	_mutex_unlock(&cbuf_mutex);

	return ret;

}
err_t mutex_cbuf_popdiscard(cbuf_t *cb) {
	err_t ret;

	_mutex_lock(&cbuf_mutex);
	ret = cbufPopDiscard(cb);
	_mutex_unlock(&cbuf_mutex);

	return ret;

}
static A_INT32 hub_init_send_params(TS_THROUGHPUT_CXT *tCxt) {
	A_INT32 retval = -1;
	A_INT32 max_packet_size = 0;
	A_INT32 return_code = A_OK;

	/* Port.*/
	tCxt->tx_params.port = MY_PORT;

	/* Packet size.*/
	tCxt->tx_params.packet_size = txSize;
	tCxt->tx_params.test_mode = txMode;
	tCxt->tx_params.packet_number = txPktNum;

	/* Inter packet interval for Bandwidth control*/
	tCxt->tx_params.interval = txInterval;
	tCxt->test_type = txType;

	return return_code;
}

static sensor_record_wifi_t decompactPacket(int satNr, sensor_record_t data){
  sensor_record_wifi_t result;
  
  result.accel_x = data.accel_x;
  result.accel_y = data.accel_y;
  result.accel_z = data.accel_z;
  result.gyro_x = data.gyro_x;
  result.gyro_y = data.gyro_y;
  result.gyro_z = data.gyro_z;
#if ENABLE_MAG
  result.mag_x = data.mag_x;
  result.mag_y = data.mag_y;
  result.mag_z = data.mag_z;
#endif /* ENABLE_MAG */    

  int16_t tmp=(int16_t)((data.quat_w)/CPI_QUATERNION16_BIAS_MULTIPLIER);
  if(tmp==(int16_t)CPI_QUATERNION16_COUNTER_COMMAND){
 	result.quat_w = ((float)tmp);
 	result.quat_x = ((float)data.quat_x);
 	result.quat_y = ((float)data.quat_y);
 	result.quat_z = ((float)data.quat_z);
  }else if(tmp==(int16_t)CPI_QUATERNION16_BIASSEN_COMMAND || tmp==104 || tmp==107 || tmp==110){
 	result.quat_w = ((float)tmp);
 	result.quat_x = ((float)data.quat_x)/CPI_QUATERNION16_BIAS_MULTIPLIER;
 	result.quat_y = ((float)data.quat_y)/CPI_QUATERNION16_BIAS_MULTIPLIER;
 	result.quat_z = ((float)data.quat_z)/CPI_QUATERNION16_BIAS_MULTIPLIER;
  }else if(tmp==(int16_t)CPI_QUATERNION16_AGBISEN_COMMAND){
 	result.quat_w = ((float)tmp);
 	result.quat_x = ((float)data.quat_x);
 	result.quat_y = ((float)data.quat_y);
 	result.quat_z = ((float)data.quat_z);
  }else if(tmp==(int16_t)CPI_QUATERNION16_VERSION_COMMAND){
 	result.quat_w = ((float)tmp);
 	result.gyro_y = ((int16_t)REVIS_VERSION_HUB22F);
  }else{//in this case is sent the MAG BIAS values 
 	result.quat_w = ((float)data.quat_w)/CPI_QUATERNION16_MULTIPLIER;
 	result.quat_x = ((float)data.quat_x)/CPI_QUATERNION16_MULTIPLIER;
 	result.quat_y = ((float)data.quat_y)/CPI_QUATERNION16_MULTIPLIER;
 	result.quat_z = ((float)data.quat_z)/CPI_QUATERNION16_MULTIPLIER;
  }
  
  return result; 
}

/* Function: hub_send_to_wifi
 * This function sends the data.
 */
static A_INT32 hub_send_to_wifi(hub_to_cloud_t *tmp_cbuf_to_cloud) {
	uint32_t 		cnt, timeout = 3000 * 100; /* Maximum seconds waiting for malloc */
	int32_t 		send_result;

#if !ENABLE_WIFI_TX
	return E_OK;
#endif

	/* Should alloc this every time.  The driver will decide when it will be released. */
	p_tCxt->txbuffer = CUSTOM_ALLOC(txSize);
	if (p_tCxt->txbuffer == NULL) {
		/* Wait till we get a buffer */
		for (cnt = 0; cnt < timeout; cnt++) {
			app_time_delay(SMALL_TX_DELAY * 10);
			p_tCxt->txbuffer = CUSTOM_ALLOC(txSize);
			if (p_tCxt->txbuffer == NULL)
				continue;
			else
				break;
		}
	}

	/* Unrecoverable error */
	if (cnt == timeout) {
		ts_blinkLEDerror(redLed, 3);
	}

	memcpy(p_tCxt->txbuffer, tmp_cbuf_to_cloud, sizeof(*tmp_cbuf_to_cloud));

	send_result = t_send((void*) handle, p_tCxt->sock_peer,
			(unsigned char*) (p_tCxt->txbuffer), txSize, 0);

#if !NON_BLOCKING_TX
	/*Free the buffer only if NON_BLOCKING_TX is not enabled*/
	if (p_tCxt->txbuffer)
		CUSTOM_FREE(p_tCxt->txbuffer);
#endif

//	/* Bandwidth control CWATI CWATI CWATI NEED TO REMOVE THIS LATER?? TODO!*/
//	app_time_delay(p_tCxt->tx_params.interval);

	static uint32_t failed_send = 0;
	if (send_result != txSize) {
		failed_send++;
		if (failed_send == 1000) {
			/* Consecutive 10 fails, then deem failure and request power cycle. */
			ts_blinkLEDerror(purpleLed, 3);
		}
	} else {
		failed_send = 0;
	}
	/* The driver returns num of bytes sent.  But if error happens, it also sends positive numbers.
	 * This is a major bug in the driver.
	 */
	if (send_result == txSize) {
#if DEBUG
		uint8_t cnt;
		printf("Here is the message: Timestamp 0x%x\n"
				"                     bitmap    0x%x\n\n",
				tmp_cbuf_to_cloud.timestamp, tmp_cbuf_to_cloud.bitmap);
		for (cnt = 0; cnt < cur_num_of_sat; cnt++) {
			printf("                     %d.satID   0x%x\n"
					"                     %d.data[0]   %f\n"
					"                     %d.data[1]   %f\n"
					"                     %d.data[2]   %f\n"
					"                     %d.data[3]   %f\n",
					cnt, tmp_cbuf_to_cloud.data[cnt].satellite_id,
					cnt, tmp_cbuf_to_cloud.data[cnt].record.quat_w,
					cnt, tmp_cbuf_to_cloud.data[cnt].record.quat_x,
					cnt, tmp_cbuf_to_cloud.data[cnt].record.quat_y,
					cnt, tmp_cbuf_to_cloud.data[cnt].record.quat_z);
		}
		sent_package++;
		printf("Sent this many bytes: %d.  Total packages sent: %d\n",send_result, sent_package);
#endif
	}

	return E_OK;
}

/*
 * Function: hub_send_to_wifi_flush()
 * Sending the data whether or not it's complete from all satellites.
 */
static A_INT32 hub_send_to_wifi_flush() {
	hub_to_cloud_t tmp_cbuf_to_cloud;
	uint32_t lowest_ts = 0;
	uint8_t i, j;
	CB_TYPE popped_data;
	uint32_t ret;

#if DEBUG
	static uint32_t sent_package = 0;
#endif

	if (!cbufs_not_empty()) {
		return E_NO_RESOURCE;
	}

	lowest_ts = INVALID_TIMESTAMP;

	/* Find lowest timestamp */
	for (i = 0; i < cur_num_of_sat; i++) {
		if (cbufIsEmpty(&cbuf[i])) {
			continue;
		}

		if (cbuf[i].A[cbuf[i].head].timestamp < lowest_ts) {
			lowest_ts = cbuf[i].A[cbuf[i].head].timestamp;
		}
	}

	/* all of cbuf is empty, return */
	if (lowest_ts == INVALID_TIMESTAMP) {
		return E_NO_RESOURCE;
	}

	/* Now we prepare the data to send */
	tmp_cbuf_to_cloud.hubId = cur_hub_id;
	tmp_cbuf_to_cloud.timestamp = lowest_ts;
	tmp_cbuf_to_cloud.ack = ack_to_client;
	tmp_cbuf_to_cloud.bitmap = 0;

	for (i = 0; i < cur_num_of_sat; i++) {
		if ((cbuf[i].A[cbuf[i].head].timestamp - lowest_ts) < 10) {
			tmp_cbuf_to_cloud.bitmap |= (1 << i);
			mutex_cbuf_pop(&cbuf[i], &popped_data);
			tmp_cbuf_to_cloud.data[i].record = decompactPacket(i,popped_data.record);
			tmp_cbuf_to_cloud.data[i].satellite_id = cur_sat_ids[i];
		}
	}

	/* No Resource, shouldn't have arrived here! */
	if (tmp_cbuf_to_cloud.bitmap == 0) {
		return E_NO_RESOURCE;
	}

	ret = hub_send_to_wifi(&tmp_cbuf_to_cloud);
	if (ret == E_OK) {
		/* Reset ACK after we've sent it. */
		ack_to_client &= ~HUB_RTC_ACK;
	}
	return (ret);
}
/*
 * Function: hub_send_to_wifi_batch()
 * Will categorize data into 10ms buckets.
 *
 * If this 10ms bucket is not full yet, it will return E_INCOMPLETE.
 *
 * If this function could gather 10ms bucket, it will send and return the send result.
 *
 */
static A_INT32 hub_send_to_wifi_batch() {
	hub_to_cloud_t tmp_cbuf_to_cloud;
	uint32_t lowest_ts = 0;
	uint8_t i, j;
	CB_TYPE popped_data;
	static uint8_t wait_for_complete = 0;
	uint32_t ret = E_OK, ret2 = E_OK;

#if DEBUG
	static uint32_t sent_package = 0;
#endif

	lowest_ts = INVALID_TIMESTAMP;

	/* Now we prepare the data to send */
	tmp_cbuf_to_cloud.hubId = cur_hub_id;
	tmp_cbuf_to_cloud.timestamp = lowest_ts;
	tmp_cbuf_to_cloud.ack = ack_to_client;
	tmp_cbuf_to_cloud.bitmap = 0;

	if (!cbufs_not_empty()) {
		ret = E_NO_RESOURCE;
		goto SEND_TO_WIFI_BATCH_ACK;
	}

	/* Find lowest timestamp */
	for (i = 0; i < cur_num_of_sat; i++) {
		if (cbufIsEmpty(&cbuf[i])) {
			continue;
		}

		if (cbuf[i].A[cbuf[i].head].timestamp < lowest_ts) {
			lowest_ts = cbuf[i].A[cbuf[i].head].timestamp;
		}
	}
	tmp_cbuf_to_cloud.timestamp = lowest_ts;

	/* all of cbuf is empty, return */
	if (lowest_ts == INVALID_TIMESTAMP) {
		ret = E_NO_RESOURCE;
		goto SEND_TO_WIFI_BATCH_ACK;
	}

	for (i = 0; i < cur_num_of_sat; i++) {
		if (cbufIsEmpty(&cbuf[i])) {
			/* If this particular sat has no data, either wait for next round,
			 * or continue if we're running out of wait time.
			 */
			if (wait_for_complete == wait_for_complete_max) {
				continue;
			} else {
				wait_for_complete++;
				ret = E_INCOMPLETE;
				goto SEND_TO_WIFI_BATCH_ACK;
			}
		} else if ((cbuf[i].A[cbuf[i].head].timestamp - lowest_ts) < timestamp_group) {
			/* If this particular sat has data, only include the data if it's within
			 * 10ms.  If it's newer data, then send later.
			 */
			tmp_cbuf_to_cloud.bitmap |= (1 << i);
			mutex_cbuf_pop(&cbuf[i], &popped_data);
			tmp_cbuf_to_cloud.data[i].record = decompactPacket(i,popped_data.record);
			tmp_cbuf_to_cloud.data[i].satellite_id = cur_sat_ids[i];
		}
	}

	/* Reset the wait if it has reached max. */
	if (wait_for_complete == wait_for_complete_max) {
		wait_for_complete = 0;
	}

	/* No Resource, shouldn't have arrived here! */
	if (tmp_cbuf_to_cloud.bitmap == 0) {
		ret = E_NO_RESOURCE;
		goto SEND_TO_WIFI_BATCH_ACK;
	}

SEND_TO_WIFI_BATCH_ACK:
	if (ret != E_OK) {
		/* If we don't have data, but we need to send ack, let's send it */
		if (ack_to_client) {
			/* We don't return the return value of this sending (ret2), but the return
			 * value of the buffer status (ret).
			 */
			ret2 = hub_send_to_wifi(&tmp_cbuf_to_cloud);
			if (ret2 == E_OK) {
				/* Reset ACK after we've sent it. */
				ack_to_client &= ~HUB_RTC_ACK;
			}
		}
	} else {
		ret = hub_send_to_wifi(&tmp_cbuf_to_cloud);
		if (ret == E_OK) {
			/* Reset ACK after we've sent it. */
			ack_to_client &= ~HUB_RTC_ACK;
		}
	}
	return ret;
}
///*
// * Function: hub_send_ack_to_wifi()
// * Will send ACK only.
// *
// */
//static A_INT32 hub_send_ack_to_wifi() {
//	hub_to_cloud_t tmp_cbuf_to_cloud;
//
//	/* Now we prepare the data to send */
//	tmp_cbuf_to_cloud.hubId = cur_hub_id;
//	tmp_cbuf_to_cloud.timestamp = UINT32_MAX;
//	tmp_cbuf_to_cloud.ack = ack_to_client;
//	tmp_cbuf_to_cloud.bitmap = 0;
//
//	return (hub_send_to_wifi(&tmp_cbuf_to_cloud));
//}
/*
 * Function:	process_data_from_cloud
 * Parameters:	cloud_to_hub,
 * 					which is data received from cloud
 * Returns: 	none
 * Comments:	Process data from cloud and sets current status and sets status for Nordic.
 * 				Status for Nordic will be sent when we're talking to Nordic.
 *
 * 				Here are what the clients should send:
 * 				STOP: not active | not sending.  LED Green.
 * 				RTC: not active | sending | set rtc | set satellites.  LED blue.  Once ready
 * 					(receives acknowledgment from all satellites, LED will turn yellow.
 * 				START: active | sending.  LED blue/
 *
 */
static void process_data_from_cloud(cloud_to_hub_t cloud_data) {
//
//	uint32_t 	cmd = cloud_data.command;
//	uint8_t 	cnt;
//	uint32_t	j;
//
//	/* Processing received command */
//	if (COMMAND_WIFI_START(cmd)) {
//		set_hub_active(&current_state);
////		ts_turnOnOneLED(blueLed, 1);
////		ts_turnOnOneLED(redLed, 0);
//	} else {
//		set_hub_not_active(&current_state);
////		ts_turnOnOneLED(blueLed, 0);
//	}
//
//	/* No LED designated for this for now */
//	if (COMMAND_WIFI_WAIT(cmd)) {
//		set_hub_cannot_send(&current_state);
//	//	ts_turnOnOneLED(redLed, 1);
//	} else {
//		set_hub_can_send(&current_state);
////		ts_turnOnOneLED(redLed, 0);
//	}
//
//	if (COMMAND_WIFI_SET_SAT(cmd)) {
//		cur_num_of_sat = cloud_data.payload.cmd_field2;
//		hub_to_nordic.payload.cmd_field2 = cloud_data.payload.cmd_field2;
//		hub_to_nordic.command |= NORDIC_SET_SATELLITES;
//		for (cnt = 0; cnt < cur_num_of_sat; cnt++) {
//			cur_sat_ids[cnt] = cloud_data.payload.satellite_ids[cnt];
//			hub_to_nordic.payload.satellite_ids[cnt] =
//					cloud_data.payload.satellite_ids[cnt];
//		}
//	}
//
//	if (COMMAND_WIFI_SET_RTC(cmd)) {
//		/* TODO cwati need to process RTC.  Always means set RTC to 0*/
//		hub_to_nordic.command |= NORDIC_SET_RTC;
//
//		/* Upon receiving set RTC, empty buffer */
//		for (uint8_t sat_idx = 0; sat_idx < MAX_SENSORS; sat_idx++) {
//			cbufInit(&cbuf[sat_idx]);
//		}
////
////		ack_to_client &= ~HUB_RTC_ACK;
//
//#if DEBUG0
//		_time_get(&latest_rtc);
//		printf("RTC 0 received.  Our latest SECONDS: %u MS: %u\n",
//				latest_rtc.SECONDS,
//				latest_rtc.MILLISECONDS);
//#endif
//	}
//
//	if (cmd & WIFI_DIAG) {
//		if (enable_stats & 0x1) {
//			hub_to_cloud_t	dbg_htc;
//
//			printf("Hub %s, %s send\n", hub_active(current_state) ? "active" : "NOT active",
//					hub_cannot_send(current_state) ? "can NOT" : "can");
//		}
//
//	}
//        
//        if (COMMAND_WIFI_CALIBRATE(cmd)) {
//          hub_to_nordic.command |= NORDIC_CALIBRATE;
//          for (uint8_t cnt = 0; cnt < MAX_SAT_COMMSTRUCT; cnt++) {
//            hub_to_nordic.payload.satellite_ids[cnt] = cloud_data.payload.satellite_ids[cnt];
//          }
//        }
//            
//        
//    if (COMMAND_WIFI_SET_LCP(cmd)) {
//    	/* TODO cwati need to set LCP */
//        hub_to_nordic.command |= NORDIC_SET_LCP;
//        hub_to_nordic.payload.cmd_field1 = cloud_data.payload.cmd_field1;
//	}
//
//    if (COMMAND_WIFI_SET_CHANNEL(cmd)) {
//
//#if WIFI_DIRECT
//        if (cloud_data.payload.cmd_field1 <= MAX_WIFI_CHANNEL) {
//            wifi_channel = cloud_data.payload.cmd_field1;
//
//			/* Set channel
//			 * wmiconfig --channel <NUM> */
//			wmiconfig_set_channel(wifi_channel);
//			return;
//        }
//#endif
//	}
//
//    /* LED */
//    /* READY state.  Getting ready for active transmission.  Green then later yellow. */
//	if (COMMAND_WIFI_SET_RTC(cmd)) {
//		ts_turnOnOneLED(blueLed, 0);			/* Only connection LED is green, so this will yield green color */
//		ts_turnOnOneLED(redLed, 0);
//    }
//
//	/* Start state */
//    if (hub_active(current_state) && hub_can_send(current_state)) {
//		ts_turnOnOneLED(blueLed, 1);			/* Connection LED is green, so this will yield turqoise color */
//		ts_turnOnOneLED(redLed, 0);
//    }
//    
//    	/* Start state */
//    if (hub_not_active(current_state) && hub_cannot_send(current_state)) {
//		ts_turnOnOneLED(blueLed, 0);			/* Connection LED is green, so this will yield turqoise color */
//		ts_turnOnOneLED(redLed, 0);
//    }
}


/*
 * Function: 			recv_from_wifi
 * Parameters:			none
 * Returns:				A_OK, or error codes
 * Comments:
 * 		This function will receive command from Wifi.
 * 		It will call a function to process the command.
 *
 */
A_INT32 recv_from_wifi(void) {
	A_INT32 received;

#if !ZERO_COPY
	if((tCxt.rxbuffer = A_MALLOC(CFG_PACKET_SIZE_MAX_RX,MALLOC_ID_CONTEXT)) == NULL)
	{
		printf("Out of memory error\n");
		goto ERROR;
	}
#endif

#if DEBUG
	printf("I am trying to hear from server...\n");
#endif

#if ZERO_COPY
	received = t_recv((void*) handle, tCxt.sock_peer, (char**) (&(tCxt.rxbuffer)),
			CFG_PACKET_SIZE_MAX_RX, 0);

    if(received > 0)
    {
        // Free the RX buffer again
        zero_copy_free(tCxt.rxbuffer);
    }
#else
	received = t_recv((void*)handle, tCxt[loop].sock_peer, (char*)(&tCxt[loop].buffer[0]), CFG_PACKET_SIZE_MAX_RX, 0);
#endif /* ZERO_COPY */
	if (received == A_ERROR) {
#if DEBUG
		printf("No data received yet...\n");
#endif
	} else if ((received == A_SOCK_INVALID) || (received == 0)) {
		/* Peer socket issue */
#if DEBUG
		printf("Invalid socket... ret=%d\n", received);
#endif
		return A_SOCK_INVALID;
	} else if (received == sizeof(cloud_to_hub_t)) { /* This should receive some good value */

		memcpy(&cloud_to_hub, tCxt.rxbuffer, sizeof(cloud_to_hub_t));
#if DEBUG
		printf("Received %d bytes out of expected %d bytes\n", received, sizeof(cloud_to_hub_t));
		printf("Received command from cloud: 0x%x\n", cloud_to_hub.command);
//		printf("Received RTC from cloud: 0x%x\n", cloud_to_hub.payload.rtc_value);
#endif
		process_data_from_cloud(cloud_to_hub);
	//	cloud_to_hub.command &= ~(WIFI_DIAG);

	} else if (received != sizeof(cloud_to_hub_t)) {
		printf("ERROR! Received %d bytes out of expected %d bytes\n",
				received, sizeof(cloud_to_hub_t));
	}

	return received;
}

/*
 * Establish connection to peer socket
 */
A_INT32 connect_to_peer(TS_THROUGHPUT_CXT *p_tCxt) {
	A_UINT16 addr_len = 0;
	SOCKADDR_T foreign_addr;
	A_INT32 return_code = A_OK;
	char ip_str[16];
	_ip_address temp_addr;

	addr_len = sizeof(foreign_addr);

	/* Must call t_select before t_accept for this driver */
	p_tCxt->sock_peer = t_accept((void*) handle, p_tCxt->sock_local,
			&foreign_addr, addr_len);

	if (tCxt.sock_peer != A_ERROR) {
		memset(ip_str, 0, sizeof(ip_str));
		temp_addr = LONG_BE_TO_HOST(foreign_addr.sin_addr);
		debug_printf("\r\nAccepted connection from %s port %d\n",
				inet_ntoa(*(A_UINT32 *) (&temp_addr), (char *) ip_str),
				foreign_addr.sin_port);
		debug_printf("\r\nConnection established!\n");
		return_code = A_OK;
	} else {
		return A_ERROR;
	}
	return return_code;
}

/*
 * Trying to establish connection to peer.
 * Parameters:	p_tCxt 		the context
 * 				timeout		in s, how many seconds we should wait for listen.
 * 							The same value is also used to wait for "select",
 * 							i.e., until we receive first packet.
 *
 * Return values: A_ERROR	error
 * 				  A_OK		connection established
 * 				  A_SOCK_UNAVAILABLE	timeout has been reached
 */
A_INT32 waiting_for_connection(TS_THROUGHPUT_CXT *p_tCxt, A_INT32 timeout) {
	uint8_t i;
	A_INT32 return_code = A_OK;
	A_INT32 conn_sock;

#if DEBUG
	printf("Listening for connection...\n");
#endif

	i = 0;
	while (1) {
		ts_turnOnOneLED(greenLed, 1);
		/* Listen. */
		return_code = t_listen((void*) handle, p_tCxt->sock_local, 1);
		if (return_code == A_ERROR) {
			printf("ERROR: Socket listen error. ERR:%d\r\n", return_code);
			return return_code;
		} else if (return_code == A_OK) {
			break;
		}

		if (++i == timeout) {
			return_code = A_SOCK_UNAVAILABLE;
			return return_code;
		}
		ts_turnOnOneLED(greenLed, 0);
		app_time_delay(1000); /* Wait, in milliseconds */
	}

	i = 0;
	while (1) {
		ts_turnOnOneLED(greenLed, 1);
		/* block for 50 msec or until a packet is received */
		conn_sock = t_select((void*) handle, p_tCxt->sock_local,
				TCP_CONNECTION_WAIT_TIME);

		if (conn_sock == A_SOCK_INVALID) {
			printf(
					"Peer sock closed connection.  Socket is no longer valid. Exiting...\n");
			return A_ERROR;
		} else if (conn_sock == A_OK) {
#if DEBUG
			printf("Select successful.  Going to connect...\n");
#endif
			break;
		}

		if (++i == timeout) {
			return_code = A_SOCK_UNAVAILABLE;
			return return_code;
		}
		ts_turnOnOneLED(greenLed, 0);
		app_time_delay(1000); /* Wait, in milliseconds */
	}

	return_code = connect_to_peer(p_tCxt);
	/* If communication is established fine, then return OK */
	if (return_code == A_OK) {
		ts_turnOnOneLED(greenLed, 1);
	} else {
		ts_turnOnOneLED(greenLed, 0);
	}
	return return_code;

}

/*
 * Initializing connection to Wifi.
 *
 * Returns A_OK if connection is established.
 *
 */
A_INT32 init_wifi(void) {
	boolean print_usage = 0;
	A_INT32 return_code = A_OK;
	A_INT32 retval = -1;
	A_INT32 max_packet_size = 0;
	SOCKADDR_T local_addr;
	A_UINT32 dummy32 = 0, tv = 2000 /* 2 seconds */;
	A_INT32 conn_sock;

	memset(&tCxt, 0, sizeof(tCxt));
	p_tCxt = &tCxt;
	hub_init_send_params(p_tCxt);

	/* Printing to shell */
//    temp_addr = LONG_BE_TO_HOST(p_tCxt->tx_params.ip_address);
//    memset(ip_str, 0, sizeof(ip_str));
//    printf("Remote IP addr. %s\n", inet_ntoa(*(A_UINT32 *)( &temp_addr), ip_str));
	printf("Transmit port %d\n", p_tCxt->tx_params.port);
	printf("Message size %d\n", p_tCxt->tx_params.packet_size);
	printf("Number of messages %d\n", p_tCxt->tx_params.packet_number);

	if ((p_tCxt->sock_local = t_socket((void*) handle, ATH_AF_INET,
			SOCK_STREAM_TYPE, 0)) == A_ERROR) {
		debug_printf("ERROR: Unable to create socket\n");
		return A_ERROR;
	}
#if DEBUG
	else {
		printf("Create socket OK...\n");
	}
#endif

	/*Allow small delay to allow other thread to run*/
	//cwati todo remove this
	app_time_delay(TX_DELAY);

	memset(&local_addr, 0, sizeof(local_addr));
//    local_addr.sin_addr = p_tCxt->tx_params.ip_address;
	local_addr.sin_addr = INADDR_ANY;
	local_addr.sin_port = p_tCxt->tx_params.port;
	local_addr.sin_family = ATH_AF_INET;

	/* Bind socket.*/
	return_code = t_bind((void*) handle, p_tCxt->sock_local, &local_addr,
			sizeof(local_addr));
	if (return_code != A_OK) {
		debug_printf("ERROR: Socket bind %d.\r\n", return_code);
		return return_code;
	}
#if DEBUG
	else {
		printf("Bind success!\n");
	}
#endif

	return return_code;
}

uint32_t init_wmiconfig(void) {
	uint32_t	ret;
	uint8_t		cnt;
#if WIFI_DIRECT
	char*		argv[] = {"wmiconfig", "--ap", "bconint", "100"};

	/* "wmiconfig --mode ap" */
	ret = wmiconfig_set_mode_ap();
#endif
	/* "wmiconfig --p <password>" */
	ret = wmiconfig_set_passphrase(SSID_PASSWORD);
	if (ret != A_OK) {
		return ret;
	}

	/* Set WPA version and cipher
	 * "wmiconfig --wpa 2 CCMP CCMP" */
	ret = wmiconfig_set_wpa(2, "CCMP", "CCMP");
	if (ret != A_OK) {
		return ret;
	}

#if WIFI_DIRECT
	/* Set AP beacon interval to 100, which is the default value.
	 * I notice that if we set this then AP shows on the peer's
	 * list of wifi.
	 * "wmiconfig --ap bconint 100"
	 */
	wmiconfig_ap_handler(2, 4, argv);
#endif

	cnt = 0;
	ret = is_driver_initialized();
	while (ret != A_OK) {
		if (!(cnt % 10)) {
			printf("Waiting for driver to be initialized!\n");
		}
		app_time_delay(1000); /* Wait in millisecond */
		ret = is_driver_initialized();
	}

	/* Set TX power
	 * wmiconfig --settxpower 1 */
	ret = wmiconfig_set_tx_power(1);
	if (ret != A_OK) {
		return ret;
	}

#if WIFI_DIRECT
        if (wifi_channel > MAX_WIFI_CHANNEL) {
#if EVAL_BOARD
            printf("Error wifi_channel %u is bigger than max %u\n", wifi_channel, MAX_WIFI_CHANNEL);
#endif
            ts_blinkLEDerror(purpleLed, 1);
        }
        
	/* Set channel
	 * wmiconfig --channel <NUM> */
	ret = 	wmiconfig_set_channel(wifi_channel);
	if (ret != A_OK) {
		return ret;
	}
        
#if EVAL_BOARD
        printf("Using wifi channel: %d\n", wifi_channel);
#endif /* EVAL_BOARD */
#endif
	/* Check wifi connectivity.
	 * "wmiconfig --connect <SSID name>"
	 */
	ret = wmiconfig_connect_handler(SSID_NAME);
	if (ret != A_OK) {
		return ret;
	}

	cnt = 0;
	while (((ret = wifi_connected()) != 1) && (cnt++ != wifi_timeout)) {

		/* Try to reconnect after each 10s */
		if (!(cnt % 10)) {
			ret = wmiconfig_connect_handler(SSID_NAME);
			if (ret != A_OK) {
				return ret;
			}
		}
		app_time_delay(1000); /* Wait in millisecond */

	}
	if (ret != 1) {
		return A_ERROR;
	}

#if !WIFI_DIRECT
	/* Requesting static IP for now */
	ret = wmiconfig_ipconfig_static(MY_IP, MY_NETMASK, MY_GATEWAY);
	if (ret != A_OK) {
		return A_ERROR;
	}
#endif

	return A_OK;

}

uint32_t init_wmiconfig_dtm(void) {
	uint32_t	ret;
	uint8_t		cnt;
        char*		argv[] = {"wmiconfig", "--ap", "bconint", "100"};

        if (wifi_direct) {

          /* "wmiconfig --mode ap" */
          ret = wmiconfig_set_mode_ap();
        }

	/* "wmiconfig --p <password>" */
	ret = wmiconfig_set_passphrase(ssid_pass);
	if (ret != A_OK) {
		return ret;
	}

	/* Set WPA version and cipher
	 * "wmiconfig --wpa 2 CCMP CCMP" */
	ret = wmiconfig_set_wpa(2, "CCMP", "CCMP");
	if (ret != A_OK) {
		return ret;
	}

        if (wifi_direct) {
          /* Set AP beacon interval to 100, which is the default value.
           * I notice that if we set this then AP shows on the peer's
           * list of wifi.
           * "wmiconfig --ap bconint 100"
           */
          wmiconfig_ap_handler(2, 4, argv);
        }

	cnt = 0;
	ret = is_driver_initialized();
	while (ret != A_OK) {
		if (!(cnt % 10)) {
			printf("Waiting for driver to be initialized!\n");
		}
		app_time_delay(1000); /* Wait in millisecond */
		ret = is_driver_initialized();
	}

	/* Set TX power
	 * wmiconfig --settxpower 1 */
	ret = wmiconfig_set_tx_power(transmit_power);
	if (ret != A_OK) {
		return ret;
	}
        
	/* Set Phy Mode
	 */
	ret = wmiconfig_set_phy_mode(&phy_mode);
	if (ret != A_OK) {
		return ret;
	}
        
        if (wifi_direct) {
                  
          /* Set channel
           * wmiconfig --channel <NUM> */
          ret = wmiconfig_set_channel(wifi_channel);
          if (ret != A_OK) {
                  return ret;
          }
        }

	/* Check wifi connectivity.
	 * "wmiconfig --connect <SSID name>"
	 */
	ret = wmiconfig_connect_handler(ssid_name);
	if (ret != A_OK) {
		return ret;
	}

	cnt = 0;
	while (((ret = wifi_connected()) != 1) && (cnt++ != wifi_timeout)) {

		/* Try to reconnect after each 10s */
		if (!(cnt % 10)) {
			ret = wmiconfig_connect_handler(ssid_name);
			if (ret != A_OK) {
				return ret;
			}
		}
		app_time_delay(1000); /* Wait in millisecond */

	}
	if (ret != 1) {
          return A_ERROR;
	} else {
          debug_printf("Successfully connected to \"%s\"!\r\n", ssid_name);
        }

        if (!wifi_direct) {
          /* Requesting static IP for now */
          ret = wmiconfig_ipconfig_static(my_ip, my_netmask, my_gateway);
          if (ret != A_OK) {
                  return A_ERROR;
          }
        }
	return A_OK;

}

/*
 * Function: hub_send_to_wifi_dtm()
 */
static A_INT32 hub_send_to_wifi_dtm(void) {
    hub_to_cloud_t tmp_cbuf_to_cloud;
    uint32_t 		cnt, timeout = 3000 * 100; /* Maximum seconds waiting for malloc */
    int32_t 		send_result;

    /* Should alloc this every time.  The driver will decide when it will be released. */
    p_tCxt->txbuffer = CUSTOM_ALLOC(txSize);
    if (p_tCxt->txbuffer == NULL) {
            /* Wait till we get a buffer */
            for (cnt = 0; cnt < timeout; cnt++) {
                    app_time_delay(SMALL_TX_DELAY * 10);
                    p_tCxt->txbuffer = CUSTOM_ALLOC(txSize);
                    if (p_tCxt->txbuffer == NULL)
                            continue;
                    else
                            break;
            }
    }

    /* Unrecoverable error */
    if (cnt == timeout) {
            ts_blinkLEDerror(redLed, 3);
    }

    memcpy(p_tCxt->txbuffer, &tmp_cbuf_to_cloud, sizeof(tmp_cbuf_to_cloud));

    send_result = t_send((void*) handle, p_tCxt->sock_peer,
                    (unsigned char*) (p_tCxt->txbuffer), txSize, 0);

#if !NON_BLOCKING_TX
    /*Free the buffer only if NON_BLOCKING_TX is not enabled*/
    if (p_tCxt->txbuffer)
            CUSTOM_FREE(p_tCxt->txbuffer);
#endif

    //	/* Bandwidth control CWATI CWATI CWATI NEED TO REMOVE THIS LATER?? TODO!*/
    //	app_time_delay(p_tCxt->tx_params.interval);

    static uint32_t failed_send = 0;
    if (send_result != txSize) {
            failed_send++;
            if (failed_send == 1000) {
              debug_printf("\r\nFailed send %u\n", failed_send);
            }
    } else {
            failed_send = 0;
    }
    /* The driver returns num of bytes sent.  But if error happens, it also sends positive numbers.
     * This is a major bug in the driver.
     */
    if (send_result == txSize) {
        printf("Sent this many bytes: %u.\n",send_result);
    }

    return E_OK;
}

void setWiFiParameters(void)
{
    debug_printf("S| WIFI SETUP ************************\r\n");
    debug_printf("S|    Enable Wifi? 0 for No, 1 for Yes : \r\n");
    debug_printf("E|\r\n");
    debug_scanf("%u", &enable_wifi);
    debug_printf("S|    %s", enable_wifi ? "yes\r\n" : "no\r\n");
    debug_printf("E|\r\n");
    
    if (enable_wifi) {
      debug_printf("S|    Wifi direct? 0 for No, 1 for Yes : \r\n");
      debug_printf("E|\r\n");
      debug_scanf("%u", &wifi_direct);
      debug_printf("S|    %s", wifi_direct ? "yes\r\n" : "no\r\n");
      debug_printf("E|\r\n");
    
      debug_printf("S|    Enter SSID name: \r\n");
      debug_printf("E|\r\n");
      debug_scanf("%s", &ssid_name);
      debug_printf("S|    %s\r\n", ssid_name);
      debug_printf("E|\r\n");
    
      debug_printf("S|    Enter SSID password: \r\n");
      debug_printf("E|\r\n");
      debug_scanf("%s", &ssid_pass);
      debug_printf("S|    %s\r\n", ssid_pass);
      debug_printf("E|\r\n");
      
      debug_printf("S|    Enter 802.11 mode n/b/g/ht40: \r\n");
      debug_printf("E|\r\n");
      debug_scanf("%s", &phy_mode); 
      debug_printf("%S|    %s\r\n", phy_mode);
      debug_printf("E|\r\n"); 
      
      if (wifi_direct) {
          debug_printf("S|    Wifi Direct selected\r\n"
                       "Default IP Wifi Direct: 192.168.1.10\r\n"
                       "               Netmask: 255.255.255.0\r\n"
                       "               Gateway: 192.168.1.1\r\n");         
       } else {
          debug_printf("S|    Enter IP (xxx.xxx.xxx.xxx): \r\n");
          debug_printf("E|\r\n");
          debug_scanf("%s", &my_ip);
          debug_printf("S|    %s\r\n", my_ip);
          debug_printf("E|\r\n");
          
          debug_printf("S|    Enter Netmask (xxx.xxx.xxx.xxx): \r\n");
          debug_printf("E|\r\n");
          debug_scanf("%s", &my_netmask);
          debug_printf("S|    %s\r\n", my_netmask);
          debug_printf("E|\r\n");
          
          debug_printf("S|    Enter Gateway (xxx.xxx.xxx.xxx): \r\n");
          debug_printf("E|\r\n");
          debug_scanf("%s", &my_gateway); 
          debug_printf("%S|    %s\r\n", my_gateway);
          debug_printf("E|\r\n");
         
        }
      
    }
}

static void TsDoWork_dtm(void) {
  uint32_t ret32;
  //static bool first_time = true;
  A_INT32 timeout = UNITY_CONNECT_TOUT;

  //if (first_time) {

    if (enable_wifi) {
   
        debug_printf("S|    Wifi transmit power in dBm (default 1) : \r\n");
        debug_printf("E|\r\n");
        debug_scanf("%u", &transmit_power);
        debug_printf("S|    %u\r\n", transmit_power);
        debug_printf("E|\r\n");
    
        debug_printf("S|    Enter wifi channel (max %u): \r\n", MAX_WIFI_CHANNEL);
        debug_printf("E|\r\n");
        debug_scanf("%u", &wifi_channel);
        debug_printf("S|    %u\r\n", wifi_channel);
        debug_printf("E|\r\n");
        
        if (wifi_channel > MAX_WIFI_CHANNEL) {
            debug_printf("E| ERROR: Channel should be between 0-11.  Please restart test. \r\n");
            ts_blinkLEDerror(purpleLed, 1);
        }
           
        /* Initializing parameters */
        ret32 = init_wmiconfig_dtm();
        if (ret32 != A_OK) {
                debug_printf("ERROR initializing connection to wifi!\r\n");
                ts_blinkLEDerror(redLed, 1);
        }

//        while ((ret32 = init_wifi()) != E_OK) {
//                debug_printf("ERROR to connect to client, keep trying..\r\n");
//                app_time_delay(1000); /* Wait in millisecond */
//        }
//
//        debug_printf("Waiting for client connection... Please connect your PC to \"%s\"\r\n"
//                     "and run SocketTest to connect to %s port %u!\r\n", ssid_name, my_ip, my_port);
//        
//        /* Waiting for connection... WILL NOT GET OUT OF THIS LOOP UNTIL CONNECTED!!! */
//        timeout = UNITY_CONNECT_TOUT;
//
//        while ((ret32 = waiting_for_connection(p_tCxt, timeout)) != A_OK) {
//                /* Failed to establish connection to Wifi! */
//                debug_printf("No connection available after %d seconds.  Keep trying!\r\n", timeout);
//        }
//        
//        debug_printf("DTM test is running... Power cycle if you want to change configuration.\r\n");
    } /* enable_wifi */
        
    //first_time = false;
  //}
  
  //dummy send to wherever...
//  hub_send_to_wifi_dtm();
}

static void TsDoWork(void) {
	A_INT32 return_code, ret;
	/* Waiting for connection... WILL NOT GET OUT OF THIS LOOP UNTIL CONNECTED!!! */
	A_INT32 timeout = UNITY_CONNECT_TOUT;
	uint32_t		max_cbuf_fill;

#if ENABLE_WIFI
	/* Check if there's any command from wifi */
	return_code = recv_from_wifi();


	if (return_code == A_SOCK_INVALID) {
		/* Peer has closed connection or other socket issues.
		 * Need to re-establish connection!
		 */
#if !TMPDEBUG
		ts_turn_on_LED(greenLed);
#endif
		printf("Peer socket has closed connection! \n"
				"Trying to re-establish communication with peer...\n");

		set_hub_not_active(&current_state);
		set_hub_cannot_send(&current_state);

		/* Waiting for connection... WILL NOT GET OUT OF THIS LOOP UNTIL CONNECTED!!! */
		timeout = UNITY_CONNECT_TOUT;
		while ((return_code = waiting_for_connection(p_tCxt, timeout)) != A_OK) {
			/* Failed to establish connection to Wifi! */
			printf("No connection available after %d seconds.  Keep trying!\n", timeout);
		}
#if !TMPDEBUG
		ts_turn_on_LED(turqoiseLed);
#endif
	}

#if ENABLE_WIFI_TX
	if (hub_can_send(current_state)) {
#if DEBUG
		printf("Current state is can send.  I am sending message to the server...\n");
#endif

//		ret = hub_send_to_wifi_flush();
		ret = hub_send_to_wifi_batch();
	}
#endif /* ENABLE_WIFI_TX */
#endif /* ENABLE_WIFI */

#if ((!ENABLE_WIFI) || (!ENABLE_WIFI_TX))

	//just drain it
	hub_to_cloud_t tmp_cbuf_to_cloud;
	uint32_t lowest_ts;
	uint8_t i;
	CB_TYPE popped_data;
	for (int qq = 0; qq < 5; qq++) {
		for (i = 0; i < cur_num_of_sat; i++) {
			mutex_cbuf_pop(&cbuf[i], &popped_data);
		}
	}

#endif /* ENABLE_WIFI */
}

static void quit_atheros_loop() {
	t_shutdown((void*) handle, p_tCxt->sock_peer);
	t_shutdown((void*) handle, p_tCxt->sock_local);
}

/*TASK*-------------------------------------------------------------------
 *
 * Task Name : hub_main_loop
 * Comments  : Called when user types in "starttx"
 *
 *END*----------------------------------------------------------------------*/
void atheros_loop(uint32_t dummy) {
   	uint_32 flags = 0, i=0;
   	A_INT32 return_code;
   	A_INT32 timeout = UNITY_CONNECT_TOUT;

   	_mutex_init(&cbuf_mutex, NULL);

	atheros_driver_setup();

#if DEMOCFG_ENABLE_RTCS
   	HVAC_initialize_networking();
#endif

	//Setting the default power mode to REC_POWER
   	/* cwati I don't think we can (should) set power_mode to be REC_POWER in AP mode.
   	 * Instead we should set it to MAX_PERF_POWER. TODO check this.
   	 * just want it to work properly for now.
   	 */
	SET_POWER_MODE("0");	//MAX_PERF_POWER
	hvac_init = 1;

	for(;;){
		_lwevent_create(&atheros_task_event, 1/* autoclear */);

		/* block for events from other tasks */
		for(;;){
			switch(_lwevent_wait_ticks(&atheros_task_event, 0x01, TRUE, MSEC_HEARTBEAT))
			{
			case MQX_OK:
				TsDoWork_dtm();
				_lwevent_clear(&atheros_task_event, 0x01);
//				ath_cwati_cnt++;
				//	/* Bandwidth control CWATI CWATI CWATI NEED TO REMOVE THIS LATER?? TODO!*/
		//		app_time_delay(1);
				break;
			case LWEVENT_WAIT_TIMEOUT:
				/* perform periodic tasks */
				//cwati todo should I change power here?
				break;
			default:
				printf("worker task error\n");
				break;
			}
		}
	}

	_mutex_destroy(&cbuf_mutex);
	_lwevent_destroy(&atheros_task_event);
	quit_atheros_loop();
}

