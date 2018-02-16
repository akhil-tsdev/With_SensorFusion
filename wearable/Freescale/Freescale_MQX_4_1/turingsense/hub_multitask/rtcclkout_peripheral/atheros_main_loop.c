/*
 * Copyright TuringSense, Inc © 2015
 * atheros_main_loop.c
 *
 *  Created on: Nov 17, 2015
 *      Author: cwati
 *
 *
 */

#include <mqx.h>
#include "atheros_stack_offload.h"
#include "common_types.h" /* Turing Sense specific */
#include "hub_main_loop.h"
#include "main.h"
#include "throughput.h"
#include "wmiconfig_ts.h"

#define DEBUG0				1		/* Log */

uint32_t			wifi_channel = 3;

/* 100 Hz --> wait 3 cycles
   Wait this many times for bitmap completion. */
uint8_t		wait_for_complete_max = WAIT_FOR_COMPLETE_DEF;
uint32_t        nordic_repeat = NORDIC_REPEAT_DEFAULT;
uint32_t        wifi_repeat = WIFI_REPEAT_DEFAULT;
static uint32_t communicationTestCounter[MAX_SENSORS]={0}; //CF: communication analysis
static uint32_t communicationTestCounter_lastSAT22F[MAX_SENSORS]={0}; //CF: communication analysis

/* For 100Hz -> group per 10ms. */
const uint32_t		timestamp_group = (uint32_t) (((float)100.00 / (float)SAT_SENDING_FREQ) * 10.0);

#if WIFI_DIRECT
//TODO change me
#if PRODUCTION1
#define				SSID_NAME	    "HUB0_12_0_MAG50Hz_ch3" //"H29_50Hz_ch4"
#elif EVAL_BOARD
#define				SSID_NAME	    "hubEval2"
#endif
#define				SSID_PASSWORD	"hubhubhub"
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

const uint32_t wifi_timeout = 20; /* seconds */
uint32_t ack_to_client = 0x0;	/* Latest RTC from satellites */

/* Synchronization variables among tasks */
MUTEX_STRUCT cbuf_mutex;
MUTEX_STRUCT ack_to_client_mutex;
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

	for(int i = 0;i<MAX_SENSORS;i++){//cf: packet counter
		if(tmp_cbuf_to_cloud->data[i].record.quat_w==CPI_QUATERNION16_COUNTER_COMMAND){
                  if(communicationTestCounter_lastSAT22F[i]<tmp_cbuf_to_cloud->data[i].record.accel_y){
                        communicationTestCounter_lastSAT22F[i]=tmp_cbuf_to_cloud->data[i].record.accel_y;
			communicationTestCounter[i]++;
                  }
		  tmp_cbuf_to_cloud->data[i].record.quat_y = (int16_t)(communicationTestCounter[i]/CPI_QUATERNION16_COUNTER_MAXVALU);
		  tmp_cbuf_to_cloud->data[i].record.quat_z = (int16_t)(communicationTestCounter[i]%CPI_QUATERNION16_COUNTER_MAXVALU);
                }else {
                  communicationTestCounter[i] = 0;
                  communicationTestCounter_lastSAT22F[i]=0;
                }

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

boolean changeWifiChannel(uint32_t newWifi_channel){
   uint32_t        ret32, timeout;
#if WIFI_DIRECT
        if (newWifi_channel <= MAX_WIFI_CHANNEL) {
            wifi_channel = newWifi_channel;

            /* Close previous connection */
            t_shutdown((void*) handle, p_tCxt->sock_peer);
            t_shutdown((void*) handle, p_tCxt->sock_local);
            
#if ENABLE_WIFI
            ret32 = init_wmiconfig();
            if (ret32 != A_OK) {
                    printf("ERROR initializing connection to wifi!\n");
                    ts_blinkLEDerror(redLed, 1);
            }
            /* Connecting to client.  WILL NOT GET OUT OF THIS LOOP UNTIL CONNECTED!!! */
            while ((ret32 = init_wifi()) != E_OK) {
                    printf("ERROR to connect to client, keep trying..\n");
                    app_time_delay(1000); /* Wait in millisecond */
            }

            if (ret32 != A_OK) {
                    /* Failed to establish connection to Wifi! */
                    printf("Failed to initialize socket!\n");
                    ts_blinkLEDerror(redLed, 2);
            }

            /* Waiting for connection... WILL NOT GET OUT OF THIS LOOP UNTIL CONNECTED!!! */
            timeout = UNITY_CONNECT_TOUT;

            while ((ret32 = waiting_for_connection(p_tCxt, timeout)) != A_OK) {
                    /* Failed to establish connection to Wifi! */
                    printf("No connection available after %d seconds.  Keep trying!\n", timeout);
            }
#endif /* ENABLE_WIFI */
            return true;
        }
        
        return false;
#endif /* WIFI_DIRECT */
}

/*
 * Function: hub_send_to_wifi_flush()
 * Sending the data whether or not it's complete from all satellites.
 */
static A_INT32 hub_send_to_wifi_flush() {
	static hub_to_cloud_t tmp_cbuf_to_cloud;
	uint32_t lowest_ts;
	uint8_t sat_idx, i, j;
	CB_TYPE popped_data;
	uint32_t ret = E_OK, ret2 = E_OK;
  
        /* Find lowest timestamp only if we're starting a new packet */
        lowest_ts = INVALID_TIMESTAMP;

        /* Now we prepare the data to send */
        tmp_cbuf_to_cloud.hubId = cur_hub_id;
        tmp_cbuf_to_cloud.timestamp = lowest_ts;
        tmp_cbuf_to_cloud.ack = 0;
        tmp_cbuf_to_cloud.bitmap = 0;

        if (!cbufs_not_empty()) {
                ret = E_NO_RESOURCE;
                goto SEND_TO_WIFI_BATCH_ACK;
        }

        static uint8_t lowest_ts_sat_idx_selected = 0xFF;
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
                ret = E_NO_RESOURCE;
                goto SEND_TO_WIFI_BATCH_ACK;
        }
        tmp_cbuf_to_cloud.timestamp = lowest_ts;
        
	for (sat_idx = 0; sat_idx < cur_num_of_sat; sat_idx++) {
          if (cbufIsEmpty(&cbuf[sat_idx])) {
            ; /* Do nothing */
          } else if ((cbuf[sat_idx].A[cbuf[sat_idx].head].timestamp - tmp_cbuf_to_cloud.timestamp) < timestamp_group) {
                  /* If this particular sat has data, only include the data if it's within
                   * timestamp_group ms.  If it's newer data, then send later.
                   */
                  tmp_cbuf_to_cloud.bitmap |= (1 << sat_idx);
                  mutex_cbuf_pop(&cbuf[sat_idx], &popped_data);
				  
				  tmp_cbuf_to_cloud.data[sat_idx].record = decompactPacket(i,popped_data.record);

                  tmp_cbuf_to_cloud.data[sat_idx].satellite_id = cur_sat_ids[sat_idx];
          }
	}

	/* No Resource, shouldn't have arrived here! */
	if (tmp_cbuf_to_cloud.bitmap == 0) {
		ret = E_NO_RESOURCE;
		goto SEND_TO_WIFI_BATCH_ACK;
	}

SEND_TO_WIFI_BATCH_ACK:
	if (ret != E_OK) {
            /* If we don't have data, but we need to send ack, let's send it */
            _mutex_lock(&ack_to_client_mutex);
            if (ack_to_client) {
                    tmp_cbuf_to_cloud.ack = ack_to_client;
                    /* We don't return the return value of this sending (ret2), but the return
                     * value of the buffer status (ret).
                     */
                    ret2 = hub_send_to_wifi(&tmp_cbuf_to_cloud);

                    if (ret2 == E_OK) {
                            /* Reset ACK after we've sent it. */
                            ack_to_client &= ~HUB_RTC_ACK;
                    }
            }
           _mutex_unlock(&ack_to_client_mutex);
	} else {
                _mutex_lock(&ack_to_client_mutex);
		if (ack_to_client) {
                        tmp_cbuf_to_cloud.ack = ack_to_client;
			/* We don't return the return value of this sending (ret2), but the return
			 * value of the buffer status (ret).
			 */
			ret2 = hub_send_to_wifi(&tmp_cbuf_to_cloud);
                        
			if (ret2 == E_OK) {
				/* Reset ACK after we've sent it. */
				ack_to_client &= ~HUB_RTC_ACK;
			}
		} else {
                  tmp_cbuf_to_cloud.ack = 0;
                         
                  ret = hub_send_to_wifi(&tmp_cbuf_to_cloud);
                }
                _mutex_unlock(&ack_to_client_mutex);
	}
	return ret;
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
	static hub_to_cloud_t tmp_cbuf_to_cloud;
	uint32_t lowest_ts;
	uint8_t sat_idx, i, j;
	CB_TYPE popped_data;
	static uint8_t wait_for_complete = 0;
	uint32_t ret = E_OK, ret2 = E_OK;
        
        //debug for the following
        static uint32_t send_from_sat[MAX_SENSORS] = {0}, send_from_sat1[MAX_SENSORS] = {0};
        static uint32_t lowest_ts_selected[MAX_SENSORS] = {0};
        static uint32_t complete_bitmap = 0, incomplete_bitmap = 0;
        //end of debug
        
        /* If we have incomplete packets, mark it */
        static uint8_t start_new_packet = 1;
        static uint8_t last_sat_idx_pkt_incomplete = INVALID_SAT_IDX;
        
        /* Find lowest timestamp only if we're starting a new packet */
        if (start_new_packet) {
          lowest_ts = INVALID_TIMESTAMP;

          /* Now we prepare the data to send */
          tmp_cbuf_to_cloud.hubId = cur_hub_id;
          tmp_cbuf_to_cloud.timestamp = lowest_ts;
          tmp_cbuf_to_cloud.ack = 0;
          tmp_cbuf_to_cloud.bitmap = 0;

          if (!cbufs_not_empty()) {
                  ret = E_NO_RESOURCE;
                  goto SEND_TO_WIFI_BATCH_ACK;
          }

          static uint8_t lowest_ts_sat_idx_selected = 0xFF;
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
                  ret = E_NO_RESOURCE;
                  goto SEND_TO_WIFI_BATCH_ACK;
          }
          tmp_cbuf_to_cloud.timestamp = lowest_ts;
        }
        
        /* If we're continuing a previous packet, start from there.. */
        if (start_new_packet) {
          sat_idx = 0;
        } else {
          sat_idx = last_sat_idx_pkt_incomplete;
        }
        
	for ( /* Condition is set above */ ; sat_idx < cur_num_of_sat; sat_idx++) {
          if (cbufIsEmpty(&cbuf[sat_idx])) {
                  /* If this particular sat has no data, either wait for next round,
                   * or continue if we're running out of wait time.
                   */
                  if (wait_for_complete == wait_for_complete_max) {
                          wait_for_complete = 0;
                          start_new_packet = 1;
                          continue;
                  } else {
                          wait_for_complete++;
                          start_new_packet = 0;
                          last_sat_idx_pkt_incomplete = sat_idx;
                          ret = E_INCOMPLETE;
                          goto SEND_TO_WIFI_BATCH_ACK;
                  }
          } else if ((cbuf[sat_idx].A[cbuf[sat_idx].head].timestamp - tmp_cbuf_to_cloud.timestamp) < timestamp_group) {
                  /* If this particular sat has data, only include the data if it's within
                   * timestamp_group ms.  If it's newer data, then send later.
                   */
                  tmp_cbuf_to_cloud.bitmap |= (1 << sat_idx);
                  mutex_cbuf_pop(&cbuf[sat_idx], &popped_data);

				  tmp_cbuf_to_cloud.data[sat_idx].record = decompactPacket(i,popped_data.record);

                  tmp_cbuf_to_cloud.data[sat_idx].satellite_id = cur_sat_ids[sat_idx];
          }
	}

	/* No Resource.  Arriving here means stale old timestamp. */
	if (tmp_cbuf_to_cloud.bitmap == 0) {
		ret = E_NO_RESOURCE;
                start_new_packet = 1;
                last_sat_idx_pkt_incomplete = INVALID_SAT_IDX;
		goto SEND_TO_WIFI_BATCH_ACK;
	}

SEND_TO_WIFI_BATCH_ACK:
	if (ret != E_OK) {
            /* If we don't have data, but we need to send ack, let's send it */
            _mutex_lock(&ack_to_client_mutex);
            if (ack_to_client) {
                    tmp_cbuf_to_cloud.ack = ack_to_client;
                    /* We don't return the return value of this sending (ret2), but the return
                     * value of the buffer status (ret).
                     */
                    ret2 = hub_send_to_wifi(&tmp_cbuf_to_cloud);
                    start_new_packet = 1;

                    if (ret2 == E_OK) {
                            /* Reset ACK after we've sent it. */
                            ack_to_client &= ~HUB_RTC_ACK;
                    }
            }
           _mutex_unlock(&ack_to_client_mutex);
	} else {
                _mutex_lock(&ack_to_client_mutex);
		if (ack_to_client) {
                        tmp_cbuf_to_cloud.ack = ack_to_client;
			/* We don't return the return value of this sending (ret2), but the return
			 * value of the buffer status (ret).
			 */
			ret2 = hub_send_to_wifi(&tmp_cbuf_to_cloud);
                        start_new_packet = 1;

			if (ret2 == E_OK) {
				/* Reset ACK after we've sent it. */
				ack_to_client &= ~HUB_RTC_ACK;
			}
		} else {
                  tmp_cbuf_to_cloud.ack = 0;
                         
                  ret = hub_send_to_wifi(&tmp_cbuf_to_cloud);
                  start_new_packet = 1;
                }
                _mutex_unlock(&ack_to_client_mutex);
	}
	return ret;
}

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

	uint32_t 	cmd = cloud_data.command;
	uint8_t 	cnt;
	uint32_t	j;
        uint32_t        ret32, timeout;

	/* Processing received command */
	if (COMMAND_WIFI_START(cmd)) {
		set_hub_active(&current_state);
//		ts_turnOnOneLED(blueLed, 1);
//		ts_turnOnOneLED(redLed, 0);
	} else {
		set_hub_not_active(&current_state);
//		ts_turnOnOneLED(blueLed, 0);
	}

	/* No LED designated for this for now */
	if (COMMAND_WIFI_WAIT(cmd)) {
		set_hub_cannot_send(&current_state);
	//	ts_turnOnOneLED(redLed, 1);
	} else {
		set_hub_can_send(&current_state);
//		ts_turnOnOneLED(redLed, 0);
	}

	if (COMMAND_WIFI_SET_SAT(cmd)) {
		cur_num_of_sat = cloud_data.payload.cmd_field2;
                
                if (cur_num_of_sat == 5) {
                  wait_for_complete_max = WAIT_FOR_COMPLETE_5;
                  nordic_repeat = NORDIC_REPEAT_5;
                  wifi_repeat = WIFI_REPEAT_5;
                } else if (cur_num_of_sat == 6) {
                  wait_for_complete_max = WAIT_FOR_COMPLETE_6;
                  nordic_repeat = NORDIC_REPEAT_6;
                  wifi_repeat = WIFI_REPEAT_6;
                } else {
                  wait_for_complete_max = WAIT_FOR_COMPLETE_DEF;
                  nordic_repeat = NORDIC_REPEAT_DEFAULT;
                  wifi_repeat = WIFI_REPEAT_DEFAULT;                  
                }
                  
		hub_to_nordic.payload.cmd_field2 = cloud_data.payload.cmd_field2;
		hub_to_nordic.command |= NORDIC_SET_SATELLITES;
		for (cnt = 0; cnt < cur_num_of_sat; cnt++) {
			cur_sat_ids[cnt] = cloud_data.payload.satellite_ids[cnt];
			hub_to_nordic.payload.satellite_ids[cnt] =
					cloud_data.payload.satellite_ids[cnt];
		}
	}

	if (COMMAND_WIFI_SET_RTC(cmd)) {
		/* TODO cwati need to process RTC.  Always means set RTC to 0*/
		hub_to_nordic.command |= NORDIC_SET_RTC;

		/* Upon receiving set RTC, empty buffer */
		for (uint8_t sat_idx = 0; sat_idx < MAX_SENSORS; sat_idx++) {
			cbufInit(&cbuf[sat_idx]);
		}
//
//		ack_to_client &= ~HUB_RTC_ACK;

#if DEBUG0
		_time_get(&latest_rtc);
		printf("RTC 0 received.  Our latest SECONDS: %u MS: %u\n",
				latest_rtc.SECONDS,
				latest_rtc.MILLISECONDS);
#endif
	}

	if (cmd & WIFI_DIAG) {
		if (enable_stats & 0x1) {
			hub_to_cloud_t	dbg_htc;

			printf("Hub %s, %s send\n", hub_active(current_state) ? "active" : "NOT active",
					hub_cannot_send(current_state) ? "can NOT" : "can");
			printf("STATISTICS: Cbuf sizes: ");
			for (j = 0; j < cur_num_of_sat; j++) {
				printf("c[%d]=%u	", j, cbufNum(&(cbuf[j])));
			}
			printf("\n1st:  ");
			for (j = 0; j < cur_num_of_sat; j++) {
				if (cbufIsEmpty(&(cbuf[j]))) {
					printf("c[%d]= N/A ", j);
				} else {
					printf("c[%d]= 0x%x	", j, cbuf[j].A[cbuf[j].head].timestamp);
				}
			}
			printf("\n2nd:  ");
			for (j = 0; j < cur_num_of_sat; j++) {

				if (cbufNum(&(cbuf[j])) == 1) {
					printf("c[%d]= N/A	", j);
				} else {
					printf("c[%d]= 0x%x	", j, cbuf[j].A[(cbuf[j].head+1) % CB_SIZE].timestamp);
				}
			}
			printf("\n\n");
			printf("TS diff array:\n");
			for(int ab = 0; ab < 10; ab++) {
				for (int ba = 0; ba < 10; ba++) {
					printf("[%02d]:%07u ",(10*ab) + ba,ts_diff_arr[(10*ab) + ba]);
				}
				printf("\n");
			}
			printf("[100]:%u\n\n", ts_diff_arr[100]);
		}

	}
        
        if (COMMAND_WIFI_CALIBRATE(cmd)) {
          hub_to_nordic.command |= NORDIC_CALIBRATE;
          for (uint8_t cnt = 0; cnt < MAX_SAT_COMMSTRUCT; cnt++) {
            hub_to_nordic.payload.satellite_ids[cnt] = cloud_data.payload.satellite_ids[cnt];
          }
          
          //cf 20160802 - change communication channel
          if(hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_COMMA] == SAT_COMM_SETWFCOMCH){
                hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALY2] = changeWifiChannel(hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALX1]);  //the result is sent to the SAT's 22F
          }
        }
            
        
    if (COMMAND_WIFI_SET_LCP(cmd)) {
    	/* TODO cwati need to set LCP */
        hub_to_nordic.command |= NORDIC_SET_LCP;
        hub_to_nordic.payload.cmd_field1 = cloud_data.payload.cmd_field1;
	}

    if (COMMAND_WIFI_SET_CHANNEL(cmd)) {
        if (cloud_data.payload.cmd_field1 <= MAX_WIFI_CHANNEL) {
            wifi_channel = cloud_data.payload.cmd_field1;

            changeWifiChannel(wifi_channel);
         }
    }

    /* LED */
    /* READY state.  Getting ready for active transmission.  Green then later yellow. */
	if (COMMAND_WIFI_SET_RTC(cmd)) {
		ts_turnOnOneLED(blueLed, 0);			/* Only connection LED is green, so this will yield green color */
		ts_turnOnOneLED(redLed, 0);
    }

	/* Start state */
    if (hub_active(current_state) && hub_can_send(current_state)) {
		ts_turnOnOneLED(blueLed, 1);			/* Connection LED is green, so this will yield turqoise color */
		ts_turnOnOneLED(redLed, 0);
    }
    
    	/* Start state */
    if (hub_not_active(current_state) && hub_cannot_send(current_state)) {
		ts_turnOnOneLED(blueLed, 0);			/* Connection LED is green, so this will yield turqoise color */
		ts_turnOnOneLED(redLed, 0);
    }
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
		cloud_to_hub.command &= ~(WIFI_DIAG);

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
		printf("Accept connection from %s port %d\n",
				inet_ntoa(*(A_UINT32 *) (&temp_addr), (char *) ip_str),
				foreign_addr.sin_port);
		printf("Connection established!\n");
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
		printf("ERROR: Unable to create socket\n");
		return A_ERROR;
	}
#if DEBUG
	else {
		printf("Create socket OK...\n");
	}
#endif

	/*Allow small delay to allow other thread to run*/
	printf("Connecting.\n");

	memset(&local_addr, 0, sizeof(local_addr));
//    local_addr.sin_addr = p_tCxt->tx_params.ip_address;
	local_addr.sin_addr = INADDR_ANY;
	local_addr.sin_port = p_tCxt->tx_params.port;
	local_addr.sin_family = ATH_AF_INET;

	/* Bind socket.*/
	return_code = t_bind((void*) handle, p_tCxt->sock_local, &local_addr,
			sizeof(local_addr));
	if (return_code != A_OK) {
		printf("ERROR: Socket bind %d.\r\n", return_code);
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
                for (uint8_t qq = 0; qq < wifi_repeat ; qq++) {
                    ret = hub_send_to_wifi_batch();
                    //ret = hub_send_to_wifi_flush();
                    if (ret != E_OK) {
                      break; //wait for nordic
                    }
                }
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
  // Dummy project just for RTC clockout
  while(1) {
    ;
  }
}

