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
#include "adc_ts.h"

/* PIVOT 3.0 POC */
#include "pivot3_0_poc.h"
 
#define DEBUG0				1		/* Log */

uint32_t wifi_channel = WIFI_START_CHANNEL;

//cwati
static volatile bool client_major_error = 0;

/* 100 Hz --> wait 3 cycles
   Wait this many times for bitmap completion. */
uint8_t		wait_for_complete_max = WAIT_FOR_COMPLETE_DEF;
uint32_t        nordic_repeat = NORDIC_REPEAT_DEFAULT;
uint32_t        wifi_repeat = WIFI_REPEAT_DEFAULT;
static uint32_t communicationTestCounter[MAX_SENSORS]={0}; //CF: communication analysis
static uint32_t communicationTestCounter_lastSAT22F[MAX_SENSORS]={0}; //CF: communication analysis

/* Heartbeat from client */
static TIME_STRUCT last_heartbeat; /* Last time hearing from hub */
const uint32_t  max_client_no_heartbeat = 15000; /* How many ms that we have no heartbeat before disconnecting */
volatile bool   client_connected = false;

/* For 100Hz -> group per 10ms. */
 uint32_t		timestamp_group = 20; //initially set at 50 Hz //(uint32_t) (((float)100.00 / (float)SAT_SENDING_FREQ) * 10.0);

static void reinit_wifi_connection();

char ssid_name[100] = {0};

//TODO change me
#if PRODUCTION1
#define				SSID_STR	    "HUB_"STR(MAJOR_VERSION)"_"STR(MINOR_VERSION)"_"STR(REVIS_VERSION_SAT22F)"."STR(REVIS_VERSION_SATNRD)"."STR(REVIS_VERSION_HUB22F)"."STR(REVIS_VERSION_HUBNRD)"_ch"STR(WIFI_START_CHANNEL)

#elif EVAL_BOARD
#define				SSID_STR	    "hubEval2"
#endif

#define				SSID_PASSWORD_STR	    "pivotapp"
/* This is the default setting from Atheros.  We're not going to change them
 *  #define				MY_IP			"192.168.1.10"
 *  #define				MY_NETMASK		"255.255.255.0"
 *  #define				MY_GATEWAY		"192.168.1.1"
 *
 *  The first device connected to this hub's SoftAP will get IP:
 *  192.168.1.100, then
 *  192.168.1.101, etc..
 */
#define				MY_PORT_STR			12345
//cf test
#define				SSID_PASSWORD_RT	"Kinetis22F"
#define				SSID_NAME_RT		"PivotConnect"
#define				MY_IP_RT		"192.168.1.123"
#define				MY_NETMASK_RT		"255.255.255.0"
#define				MY_GATEWAY_RT		"192.168.1.255"

//#define				MY_PORT_RT		12345


const uint32_t wifi_timeout = 20; /* seconds */
uint32_t ack_to_client = 0x0;	/* Latest RTC from satellites */

/* Synchronization variables among tasks */
MUTEX_STRUCT cbuf_mutex;
MUTEX_STRUCT ack_to_client_mutex;
LWEVENT_STRUCT atheros_task_event;
//uint_8 hvac_init = 0;

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

	/* Port.  If using Wifi Direct, use the default Port MY_PORT_STR,
         * otherwise, client must connect to us to port specified in the flash. */
       if(hubType_comPort == HUB_TYPE_DIRECT)
         tCxt->tx_params.port = MY_PORT_STR;
       else
         tCxt->tx_params.port = hubType_comPort;
         
	/* Packet size.*/
	tCxt->tx_params.packet_size = txSize;
	tCxt->tx_params.test_mode = txMode;
	tCxt->tx_params.packet_number = txPktNum;

	/* Inter packet interval for Bandwidth control*/
	tCxt->tx_params.interval = txInterval;
	tCxt->test_type = txType;

	return return_code;
}

void onlyTest(bool isCommand){
 	app_time_delay(100);
	if(isCommand)
		ts_turn_on_LED(redLed);
	else
		ts_turn_on_LED(yellowLed);

	app_time_delay(100);
	ts_turn_on_LED(greenLed);
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

  result.quat_w = (data.quat_w);
  result.quat_x = (data.quat_x);
  result.quat_y = (data.quat_y);
  result.quat_z = (data.quat_z);
  
  int16_t tmp=(int16_t)((data.quat_w)/CPI_QUATERNION16_BIAS_MULTIPLIER);
        
  if(tmp==(int16_t)CPI_QUATERNION16_BATTERY_COMMAND){
   
   uint16_t batLevFlagHUB = 0;
   float readBattery = readBatteryApplyFilter((adc_read_1(0)));
   
   result.quat_y = (int16_t)(CPI_QUATERNION16_MULTIPLIER*readBattery);
   
   if (readBattery > 4.0)
     batLevFlagHUB = 3;
   else if (readBattery < 3.6)
     batLevFlagHUB = 1;
   else
     batLevFlagHUB = 2;
     
   result.accel_y = batLevFlagHUB;
  }else  if(tmp==(int16_t)CPI_QUATERNION16_VERSION_COMMAND){
	 result.gyro_y = REVIS_VERSION_HUB22F; 
  }
  
  if(tmp >= CPI_QUATERNION16_BIASSEN_COMMAND){
	 result.quat_w = (tmp); 
  }
  
  return result; 
}

/* Function: hub_send_to_wifi
* This function sends the data.
* 
* Return values:
* E_FAIL_SEND: fail to send packet.  Usually it's because peer socket is disconnected.
* E_SOCK_DISCONNECTED: fail to send 1000 times.  
* E_OK: Sent bytes ok.
*
*/
static A_INT32 hub_send_to_wifi(hub_to_cloud_t *tmp_cbuf_to_cloud) {
	uint32_t 		cnt, timeout = 3000 * 100; /* Maximum seconds waiting for malloc */
	int32_t 		send_result;
        uint32_t                ret = E_OK;
        
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
                  if(communicationTestCounter_lastSAT22F[i]<tmp_cbuf_to_cloud->data[i].record.accel_x){
                        communicationTestCounter_lastSAT22F[i]=tmp_cbuf_to_cloud->data[i].record.accel_x;
			communicationTestCounter[i]++;
                  }else if(tmp_cbuf_to_cloud->data[i].record.accel_x <= 0){
                     communicationTestCounter[i] = 0;
                     communicationTestCounter_lastSAT22F[i]=0;
                  }else if(communicationTestCounter_lastSAT22F[i]>(tmp_cbuf_to_cloud->data[i].record.accel_x)){
                    communicationTestCounter_lastSAT22F[i] = tmp_cbuf_to_cloud->data[i].record.accel_x;
                  }
                  
		  tmp_cbuf_to_cloud->data[i].record.quat_y = (int16_t)(communicationTestCounter[i]/CPI_QUATERNION16_COUNTER_MAXVALU);
		  tmp_cbuf_to_cloud->data[i].record.quat_z = (int16_t)(communicationTestCounter[i]%CPI_QUATERNION16_COUNTER_MAXVALU);
                  
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
          ret = E_FAIL_SEND;
          failed_send++;
          if (failed_send == 1000) {
//                  /* Consecutive 10 fails, then deem failure and request power cycle. */
//                  ts_blinkLEDerror(purpleLed, 3);
            ret = E_SOCK_DISCONNECTED;
            failed_send = 0;
          }
	} else {
          failed_send = 0;
	}

#if DEBUG
	/* The driver returns num of bytes sent.  But if error happens, it also sends positive numbers.
	 * This is a major bug in the driver.
	 */
	if (send_result == txSize) {
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
	}
#endif
	return ret;
}

#if PIVOT_CONNECT
/* 
 * If hub loses wifi connection, it should try to send not-active/not-sending to satellites,
 * and then try to re-establish the wifi connection.
 *
 * This loop will never exit until it's connected to client.
 */
static void reinit_wifi_connection() {
   uint32_t        ret32, timeout;

   /* First we send not active/not sending and empty our cbufs */
  set_hub_not_active(&current_state);
  //set_hub_cannot_send(&current_state);
  //empty_cbuf();
  
  /* Reset LED to off */
  ts_turnOnOneLED(blueLed, 0);			/* Only connection LED is green, so this will yield green color*/
  ts_turnOnOneLED(redLed, 0);  
  ts_turnOnOneLED(greenLed, 1);  
 
#if ENABLE_WIFI
  /* Close previous connection */
  t_shutdown((void*) handle, p_tCxt->sock_peer);
  t_shutdown((void*) handle, p_tCxt->sock_local);
              
  /* Wait for an hour to get connected to wifi.  Afterwards it will blink red. */
  ret32 = init_wmiconfig(3600);
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
}
#elif PIVOT_TENNIS
/* 
 * If hub loses wifi connection, it should try to send not-active/not-sending to satellites,
 * and then try to re-establish the wifi connection.
 *
 * This loop will never exit until it's connected to client.
 */
static void reinit_wifi_connection() {
   uint32_t        ret32, timeout;

   /* First we send not active/not sending and empty our cbufs */
  set_hub_not_active(&current_state);
  
  /* Reset LED to off */
  ts_turnOnOneLED(blueLed, 0);			/* Only connection LED is green, so this will yield green color*/
  ts_turnOnOneLED(redLed, 0);  
  ts_turnOnOneLED(greenLed, 1);  
 
  /* Close previous connection.  For PIVOT_TENNIS we don't need to close local socket. */
  t_shutdown((void*) handle, p_tCxt->sock_peer);
  client_connected = false;

  /* Waiting for connection... WILL NOT GET OUT OF THIS LOOP UNTIL CONNECTED!!! */
  timeout = UNITY_CONNECT_TOUT;

  while ((ret32 = waiting_for_connection(p_tCxt, timeout)) != A_OK) {
    /* Failed to establish connection to Wifi! */
    printf("No connection available after %d seconds.  Keep trying!\n", timeout);
  }
}
#endif /* PIVOT_CONNECT or PIVOT_TENNIS */

boolean changeWifiChannel(uint32_t newWifi_channel){
   uint32_t        ret32, timeout;
   if(hubType_comPort == HUB_TYPE_DIRECT){
        if (newWifi_channel <= MAX_WIFI_CHANNEL) {
            wifi_channel = newWifi_channel;
#if ENABLE_WIFI
            /* Close previous connection */
            t_shutdown((void*) handle, p_tCxt->sock_peer);
            t_shutdown((void*) handle, p_tCxt->sock_local);
            
            ret32 = init_wmiconfig(300);
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
   }
        
   return false;
}

/*
 * Function: hub_send_to_wifi_batch()
 * Will categorize data into 10ms buckets.
 *
 * Return values:
 *     E_INCOMPLETE: If this 10ms bucket is not full yet
 *     E_NO_RESOURCE: No resource inside the buffer
 *     E_FAIL_SEND: Fail to send to client
 *     E_SOCK_DISCONNECTED: hub_send_to_wifi() fails and likely it's because the
 *                          peer socket is disconnected.
 * If this function could gather 10ms bucket, it will send and return the send result.
 *
 */
static A_INT32 hub_send_to_wifi_batch() {
	static hub_to_cloud_t tmp_cbuf_to_cloud;
	uint32_t lowest_ts, highest_ts;
	uint8_t sensor_idx, i, j;
	CB_TYPE popped_data;
	static uint8_t wait_for_complete = 0;
	uint32_t ret = E_OK, ret2 = E_OK;
        uint32_t sat_idx, sat_id;
        
        /* If we have incomplete packets, mark it */
        static uint8_t start_new_packet = 1;
        static uint8_t last_sensor_idx_pkt_incomplete = INVALID_SENSOR_IDX;
        
        /* Find lowest timestamp only if we're starting a new packet */
        if (start_new_packet) {
          lowest_ts = INVALID_TIMESTAMP;
          highest_ts = 0;

          /* Now we prepare the data to send */
          tmp_cbuf_to_cloud.hubId = cur_hub_id;
          tmp_cbuf_to_cloud.timestamp = lowest_ts;
          tmp_cbuf_to_cloud.ack = 0;
          tmp_cbuf_to_cloud.bitmap = 0;

          if (!cbufs_not_empty()) {
                  ret = E_NO_RESOURCE;
                  goto SEND_TO_WIFI_BATCH_ACK;
          }

          static uint8_t lowest_ts_sensor_idx_selected = UINT8_MAX;
          /* Find lowest timestamp */
          for (i = 0; i < cur_num_of_sensors; i++) {
                  if (cbufIsEmpty(&cbuf[i])) {
                          continue;
                  }

                  //if (cbuf[i].A[cbuf[i].head].timestamp > 0)
                  {
                      if (cbuf[i].A[cbuf[i].head].timestamp < lowest_ts) {
                        lowest_ts = cbuf[i].A[cbuf[i].head].timestamp;
                        //if(lowest_ts > highest_ts)
                        //  highest_ts = lowest_ts;
                      }
                      
                      if(cbuf[i].A[cbuf[i].head].timestamp > highest_ts){
                        highest_ts = cbuf[i].A[cbuf[i].head].timestamp;
                      }
                  }
          }

          /* all of cbuf is empty, return */
          if (lowest_ts == INVALID_TIMESTAMP) {
                  ret = E_NO_RESOURCE;
                  goto SEND_TO_WIFI_BATCH_ACK;
          }
          tmp_cbuf_to_cloud.timestamp = (lowest_ts+highest_ts)/2;
        }
        
        /* If we're continuing a previous packet, start from there.. */
        if (start_new_packet) {
          sensor_idx = 0;
        } else {
          sensor_idx = last_sensor_idx_pkt_incomplete;
        }
        
	for ( /* Condition is set above */ ; sensor_idx < cur_num_of_sensors; sensor_idx++) {
          if (cbufIsEmpty(&cbuf[sensor_idx])) {
                  /* If this particular sensor_idx has no data, either wait for next round,
                   * or continue if we're running out of wait time.
                   */
                  if (wait_for_complete == wait_for_complete_max) {
                          wait_for_complete = 0;
                          start_new_packet = 1;
                          continue;
                  } else {
                          wait_for_complete++;
                          start_new_packet = 0;
                          last_sensor_idx_pkt_incomplete = sensor_idx;
                          ret = E_INCOMPLETE;
                          goto SEND_TO_WIFI_BATCH_ACK;
                  }
          } else if (1){ //(abs(cbuf[sat_idx].A[cbuf[sat_idx].head].timestamp - tmp_cbuf_to_cloud.timestamp) < (timestamp_group/2+timestamp_group/4)) {
                  /* If this particular sat has data, only include the data if it's within
                   * timestamp_group ms.  If it's newer data, then send later.
                   */
                  tmp_cbuf_to_cloud.bitmap |= (1 << sensor_idx);
                  mutex_cbuf_pop(&cbuf[sensor_idx], &popped_data);

                  tmp_cbuf_to_cloud.data[sensor_idx].record = decompactPacket(i,popped_data.record);

                  sat_idx = get_sat_idx_of_sensor_idx(sensor_idx);
                  if (sat_idx != UINT8_MAX) {
                    sat_id = cur_sat_ids[sat_idx];
                  } else {
                    sat_id = UINT32_MAX;
                  }
                  tmp_cbuf_to_cloud.data[sensor_idx].satellite_id = sat_id;
          }
	}

	/* No Resource.  Arriving here means stale old timestamp. */
	if (tmp_cbuf_to_cloud.bitmap == 0) {
		ret = E_NO_RESOURCE;
                start_new_packet = 1;
                last_sensor_idx_pkt_incomplete = INVALID_SENSOR_IDX;
		goto SEND_TO_WIFI_BATCH_ACK;
	}

SEND_TO_WIFI_BATCH_ACK:
	if (ret != E_OK) {
            /* If we don't have data, but we need to send ack, let's send it */
            _mutex_lock(&ack_to_client_mutex);
            if (ack_to_client) {
                    tmp_cbuf_to_cloud.ack = ack_to_client;
                    /* PV-1022 This is a hack until we get FW 2.0 */
                    SET_HUB_LCP(tmp_cbuf_to_cloud.ack, cur_hub_lcp);
                                
                    /* We don't return the return value of this sending (ret2), but the return
                     * value of the buffer status (ret).
                     */
                    ret2 = hub_send_to_wifi(&tmp_cbuf_to_cloud);
                    start_new_packet = 1;

                    if (ret2 == E_OK) {
                      /* Reset ACK after we've sent it. */
                      ack_to_client = 0;
                    } else {
                      ret = ret2;
                    }
            }
           _mutex_unlock(&ack_to_client_mutex);
	} else {
                _mutex_lock(&ack_to_client_mutex);
		if (ack_to_client) {
                        tmp_cbuf_to_cloud.ack = ack_to_client;
                        
                        /* PV-1022 This is a hack until we get FW 2.0 */
                        SET_HUB_LCP(tmp_cbuf_to_cloud.ack, cur_hub_lcp);
                    
			/* We don't return the return value of this sending (ret2), but the return
			 * value of the buffer status (ret).
			 */
			ret2 = hub_send_to_wifi(&tmp_cbuf_to_cloud);
                        start_new_packet = 1;

			if (ret2 == E_OK) {
				/* Reset ACK after we've sent it. */
				ack_to_client = 0;
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

static uint32_t calculateSATID() {
            //uint32_t MOD=1073741824;
            //return SIM_UIDH%MOD+SIM_UIDMH%MOD+SIM_UIDML%MOD+SIM_UIDL%MOD;
	return (SIM_UIDL & 0x003F) | (((SIM_UIDMH >> 8) & 0x003F) << 6) | (((SIM_UIDH >> 16) & 0x003F) << 12);
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

    uint32_t 	cmd = cloud_data.payload.satellite_ids[SAT_COMMSTRUCT_COMMA];
    uint32_t 	crc = cloud_data.crc;
    uint8_t 	cnt;
    uint32_t	j;
    uint32_t    ret32, timeout;
    uint32_t    satTMP; 
    uint8_t     num_of_sensors;
    int         sat_idx;
    
     _time_get_elapsed(&last_heartbeat);
    
    
    /* If we don't need to forward, don't forward. */
    if (!command_is_for_hub22f_only(cmd)) {
      for(int i = 0; i < MAX_SAT_COMMSTRUCT; i++){
        hub_to_nordic.payload.satellite_ids[i] = cloud_data.payload.satellite_ids[i];
      }
    }
     
	switch(cmd){
        case SAT_COMM_MBIASSENDG:
          break;
          
        case SAT_COMM_WIFI_PING:
              /* cwati todo
           * do nothing? Pretty much any command, and not just heartbeat command can count
           * as heartbeat */
        break;
        
        case SAT_COMM_WIF_SETSAT:
              onlyTest(false);
              /* For this command
               * 
               * X1 = sat index
               * Y2 = sat ID
               * Z3 = (unused) but this allocates the total number of satellites
               * T4 = number of sensors for this satellite
               *
               */
              // The client wants to use the sat id list in the hub flash
              if (cloud_data.payload.satellite_ids[SAT_COMMSTRUCT_VALX1] == 0 && cloud_data.payload.satellite_ids[SAT_COMMSTRUCT_VALY2] == 0) 
                readAllSatIdList();

              //  The client wants to use his own sat list
              else
              {
                sat_idx = cloud_data.payload.satellite_ids[SAT_COMMSTRUCT_VALX1]; /* Satellite index */
                satTMP = cloud_data.payload.satellite_ids[SAT_COMMSTRUCT_VALY2]; /* Satellite ID */
                
                /* If field T4 is not 0, then this is the number of sensors for this satellite.  Let's update. */
                if (cloud_data.payload.satellite_ids[SAT_COMMSTRUCT_VALT4]) {
                  cur_sensors_per_sat[sat_idx] = cloud_data.payload.satellite_ids[SAT_COMMSTRUCT_VALT4];
                }
                
                cur_sat_ids[sat_idx] = satTMP;
                
                for(int t=sat_idx+1;t<MAX_SENSORS;t++) {
                    cur_sat_ids[t] = 0;
                }
                
                cur_num_of_sat=0;
                cur_num_of_sensors=0;
                for(int t=0;t<MAX_SENSORS;t++) {
                  if(cur_sat_ids[t]>0) {
                      cur_num_of_sat += 1;
                      cur_num_of_sensors += cur_sensors_per_sat[t];
                  }
                }
                       
                //cwati I'm commenting this out because at the end of this case statement,
                //i.e., after talking to Nordic, it writes again to flash??
                // I think this is redundant.  Is it a bug??
                // Please uncomment and write explicitly if we need to write to flash twice.
//                /* Only write to flash if client supplies request for it */
//                writeHubFlash(cloud_data.payload.satellite_ids[SAT_COMMSTRUCT_VALX1] + 2, \
//                  cloud_data.payload.satellite_ids[SAT_COMMSTRUCT_VALY2],cloud_data.payload.satellite_ids[SAT_COMMSTRUCT_VALZ3]);
              }
              
              // Send the sat id list to the nordic
              for (uint8_t cnt1 = 0; cnt1 < cur_num_of_sat; cnt1++) {
                  sendSatIdToNordic(cnt1,cur_sat_ids[cnt1],cur_num_of_sat);
                  app_time_delay(25);
                  printf("Sat[%d]: 0x%x\n", cnt1, hub_to_nordic.payload.satellite_ids[cnt1]);
                                      
              }  
                                
              // TODO PIVOT_3_0: Write number of sensors for each satellite in the flash!
              writeHubFlash(cloud_data.payload.satellite_ids[SAT_COMMSTRUCT_VALX1] + 2, \
                cloud_data.payload.satellite_ids[SAT_COMMSTRUCT_VALY2],cloud_data.payload.satellite_ids[SAT_COMMSTRUCT_VALZ3]);
        break;
        
        case SAT_COMM_WIF_SETRTC:
          onlyTest(false);
          empty_cbuf();
        break;
                
        case SAT_COMM_CWIFI_DIAG:
          if (enable_stats & 0x1) {
            hub_to_cloud_t	dbg_htc;

            printf("Hub %s\n", hub_active(current_state) ? "active, can send" : \
            "NOT active, cannot send");
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
            }
          }
        break;
        //cf 20160802 - change communication channel
       	case SAT_COMM_SETWFCOMCH:
          hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALY2] = changeWifiChannel(hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALX1]);  //the result is sent to the SAT's 22F
          onlyTest(hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALY2]);
        break;
        //gl 20160929- send HUB MPU9250 ID to sat during the GET ALL IDS
       	case SAT_COMM_GET_ALLIDS:
          hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALX1] = cur_hub_id;
          hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALY2] = calculateSATID();
          onlyTest(false);
        break;
        //gl 20161111- write hub flash
       	case SAT_COMM_WRHUBFLASH:
          writeHubFlash(hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALX1],hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALY2],hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALZ3]);
          onlyTest(false);
        break;
        //gl 20161111- read hub flash
       	case SAT_COMM_RDHUBFLASH:               
          readHubFlash();
          sendHubFlashInfoToNordic();
          onlyTest(false);
        break;
        //gl 20161111- Erase hub flash
       	case SAT_COMM_ERHUBFLASH:              
          onlyTest(false);
          if ( hub_flash_erase_ts_sector() != E_OK) {
            printf("\nERROR! Could not erase the flash. Exiting...");
            ts_blinkLEDerror(redLed, 4);
          }
        break;
        case SAT_COMM_FRQDIVIDER:
          float tmp = (float)hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALX1];
          onlyTest(false);
          
          uint32_t tmp2 = hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALY2];
          if(tmp2>0)
              wifi_repeat = tmp2;
          else
              wifi_repeat = WIFI_MAX_REPEAT; //il the value of wifi_repeat is low cannot use FW > 100Hz

          tmp2 = hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALZ3]%100;
          if(tmp2>0)
              nordic_repeat = tmp2;
          else
              nordic_repeat = NORDIC_REPEAT_DEFAULT;
          
          tmp2 = hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALT4]/100;
          if(tmp2>0)
              wait_for_complete_max = tmp2;
          else
              wait_for_complete_max = WAIT_FOR_COMPLETE_DEF;
          
          
                
	  if(tmp>0 && tmp<256)
             timestamp_group = (uint32_t) (tmp*((float)100.00 / (float)SAT_SENDING_FREQ) * 10.0);
          else
            onlyTest(true);
        break;
        case SAT_COMM_START_DUMP:
          set_hub_active(&current_state);
        break;
        case SAT_COMM_STOP_DUMPI:
          set_hub_not_active(&current_state);
#if FAST_DRAIN
          empty_cbuf();
#endif   /* FAST_DRAIN */
        break;
        /* Change bluetooth channel */
        case SAT_COMM_WIF_SETLCP:
        case SAT_COMM_SETBTCOMCH:
          cur_hub_lcp = cloud_data.payload.satellite_ids[SAT_COMMSTRUCT_VALX1];
          break;
        case SAT_COMM_TESTVALUE0:
           //hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_COMMA] = 12345;
           onlyTest(false);
        break;
        case SAT_COMM_TESTVALUE1:
          //hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_COMMA] = SAT_COMM_TESTVALUE1;
          onlyTest(false);
        break;
        case SAT_COMM_BATTERY_LV:
          onlyTest(false);
          break;
        case SAT_COMM_TESTCOMSTC:
            	for(int i = 0;i<MAX_SENSORS;i++)//cf: packet counter
		{
                  communicationTestCounter[i] = 0;
                  communicationTestCounter_lastSAT22F[i]=0;
                }
            onlyTest(false);
            break;
	default:
          if (command_is_for_sat(cmd)) {
            onlyTest(false);
          } else {
            onlyTest(true);            
          }
    }
    
	/* Start state */
    if (hub_active(current_state)) {
		ts_turnOnOneLED(blueLed, 1);			/* Connection LED is green, so this will yield turqoise color */
		ts_turnOnOneLED(redLed, 0);
    } else { /* (hub_not_active(current_state) */
		ts_turnOnOneLED(blueLed, 0);			/* Only connection LED is green, so this will yield green color*/
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
                client_connected = true;
                /* Reset time */
                _time_get_elapsed(&last_heartbeat);
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

/* 
 * Will try to connect for a set of time.
 */
uint32_t init_wmiconfig(uint32_t wifi_timeout/* seconds */) 
{
	uint32_t	ret;
	uint8_t		cnt;

        char* argv[] = {"wmiconfig", "--ap", "bconint", "100"};
        
        if(hubType_comPort == HUB_TYPE_DIRECT){
          /* "wmiconfig --mode ap" */
          ret = wmiconfig_set_mode_ap(); 
        
          /* "wmiconfig --p <password>" */
          ret = wmiconfig_set_passphrase(SSID_PASSWORD_STR);
        }
        else
          ret = wmiconfig_set_passphrase(SSID_PASSWORD_RT);

	if (ret != A_OK) {
		return ret;
	}

	/* Set WPA version and cipher
	 * "wmiconfig --wpa 2 CCMP CCMP" */
	ret = wmiconfig_set_wpa(2, "CCMP", "CCMP");
	if (ret != A_OK) {
		return ret;
	}

   if(hubType_comPort == HUB_TYPE_DIRECT){
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
        /* TURINGSENSE, set tx power to maximum allowable.  It might not go up to 63, but it should
         * set it high, around -30dBm. */
	ret = wmiconfig_set_tx_power(63);
	if (ret != A_OK) {
		return ret;
	}

      if(hubType_comPort == HUB_TYPE_DIRECT){
             
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
      } else {
        sprintf(&ssid_name[0], "%s",SSID_NAME_RT);
      }
        /* Check wifi connectivity.
	 * "wmiconfig --connect <SSID name>"
	 */
	ret = wmiconfig_connect_handler(&ssid_name);
	if (ret != A_OK) {
		return ret;
	}

	cnt = 0;
	while (((ret = wifi_connected()) != 1) && (cnt++ != wifi_timeout)) {

		/* Try to reconnect after each 10s */
		if (!(cnt % 10)) {
			ret = wmiconfig_connect_handler(&ssid_name);
			if (ret != A_OK) {
				return ret;
			}
		}
		app_time_delay(1000); /* Wait in millisecond */

	}
	if (ret != 1) {
		return A_ERROR;
	}

   if(hubType_comPort != HUB_TYPE_DIRECT){
	/* Requesting static IP for now */
        uint32_t IP0,IP1,IP2,IP3; //192.168.1.123
        
        IP0=hubType_IPAdress%256;
        IP1=((hubType_IPAdress-IP0)/256)%256;
        IP2=((hubType_IPAdress-IP0-IP1*256)/(256*256))%256;
        IP3=((hubType_IPAdress-IP0-IP1*256-IP1*256*256)/(256*256*256))%256;
        
        char ipStr[15] = {0};
        
        if(hubType_IPAdress > 0 && hubType_IPAdress < 0xFFFFFFFF){
          sprintf(&ipStr[0], "%d.%d.%d.%d",IP3,IP2,IP1,IP0);
        }else{
          sprintf(&ipStr[0], "%s",MY_IP_RT);
        }
        
        //sprintf(&ipStr[0], "%d.%d.%d.%d",192,168,1,123);
          
        sprintf(&ssid_name[0], "%s",SSID_NAME_RT);
	ret = wmiconfig_ipconfig_static(ipStr, MY_NETMASK_RT, MY_GATEWAY_RT);
	if (ret != A_OK) {
		return A_ERROR;
	}
   }

	return A_OK;

}

static void ts_init_atheros_task() {
	A_INT32 return_code, ret32;
	A_INT32 timeout = UNITY_CONNECT_TOUT;
        
#if DEMOCFG_ENABLE_RTCS
    HVAC_initialize_networking();
#endif

    //Setting the default power mode to REC_POWER
    /* cwati I don't think we can (should) set power_mode to be REC_POWER in AP mode.
     * Instead we should set it to MAX_PERF_POWER. TODO check this.
     * just want it to work properly for now.
     */
    SET_POWER_MODE("0");	//MAX_PERF_POWER
//	power_mode = REC_POWER;

//    hvac_init = 1;

#if ENABLE_WIFI
    ret32 = init_wmiconfig(300);
    if (ret32 != A_OK) {
            printf("ERROR initializing connection to wifi!\n");
            ts_blinkLEDerror(redLed, 1);
    }
    /* Connecting to client.  WILL NOT GET OUT OF THIS LOOP UNTIL CONNECTED!!! */
    while ((return_code = init_wifi()) != E_OK) {
            printf("ERROR to connect to client, keep trying..\n");
            app_time_delay(1000); /* Wait in millisecond */
    }

    if (return_code != A_OK) {
            /* Failed to establish connection to Wifi! */
            printf("Failed to initialize socket!\n");
            ts_blinkLEDerror(redLed, 2);
    }

    /* Waiting for connection... WILL NOT GET OUT OF THIS LOOP UNTIL CONNECTED!!! */
    timeout = UNITY_CONNECT_TOUT;

    while ((return_code = waiting_for_connection(p_tCxt, timeout)) != A_OK) {
            /* Failed to establish connection to Wifi! */
            printf("No connection available after %d seconds.  Keep trying!\n", timeout);
    }
#endif

    /* Initialize time */
    _time_get_elapsed(&last_heartbeat);  
}

static void quit_atheros_loop() {
     /* First we send not active/not sending and empty our cbufs */
  set_hub_not_active(&current_state);
  
  /* Reset LED to off */
  ts_turnOnOneLED(blueLed, 0);			/* Only connection LED is green, so this will yield green color*/
  ts_turnOnOneLED(redLed, 0);  
  ts_turnOnOneLED(greenLed, 1);  
 
  /* Close previous connection.  For PIVOT_TENNIS we don't need to close local socket. */
  t_shutdown((void*) handle, p_tCxt->sock_peer);
  t_shutdown((void*) handle, p_tCxt->sock_local);

  client_connected = false;
  
#if DEMOCFG_ENABLE_RTCS
  HVAC_shutdown_networking();
#endif

}

static void TsDoWork(void) {
	A_INT32 return_code, ret;
	/* Waiting for connection... WILL NOT GET OUT OF THIS LOOP UNTIL CONNECTED!!! */
	A_INT32 timeout = UNITY_CONNECT_TOUT;
	uint32_t		max_cbuf_fill;
        TIME_STRUCT             now, diff;
        uint32_t                diff_ms;
        static bool             first_time = true;
        
        if (first_time) {
          ts_init_atheros_task();
          first_time = false;
        }
#if ENABLE_WIFI
        
	/* Check if there's any command from wifi */
	return_code = recv_from_wifi();


	if ((return_code == A_SOCK_INVALID) || client_major_error) {
            /* Peer has closed connection or other socket issues.
             * Need to re-establish connection!
             */
            printf("Peer socket has closed connection! \n"
                            "Trying to re-establish communication with peer...\n");

            reinit_wifi_connection();
            
            if (client_major_error) {
              client_major_error = 0;
            }
	}

#if ENABLE_WIFI_TX
        /* We only talk if:
         * 1) our state allows us to talk,
         * 2) we have an acknowledgement to send, and
         * 3) we are connected to client,
         */
	if ((hub_active(current_state) || (ack_to_client)) && (client_connected)) {
#if DEBUG
		printf("Current state is can send.  I am sending message to the server...\n");
#endif
                for (uint8_t qq = 0; qq < wifi_repeat ; qq++) {
                    ret = hub_send_to_wifi_batch();
                    //ret = hub_send_to_wifi_flush();
                    if (ret != E_OK) {
                      /* E_INCOMPLETE or E_NO_RESOURCE means we need to wait for Nordic,
                       * E_SOCK_DISCONNECTED means we fail to send to client, which likely means
                       * that client is out of range */
                      break; //wait for nordic
                    }
                }
	}
        
        if (ret == E_SOCK_DISCONNECTED) {
            /* Peer has closed connection or other socket issues.
             * Need to re-establish connection!
             */
            printf("Failed to send.  It seems like peer socket has closed connection!\n"
                    "Trying to re-establish communication with peer...\n");
            client_major_error = 1;
            return;
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
        
      _time_get_elapsed(&now);
      _time_diff(&last_heartbeat, &now, &diff);
      diff_ms = diff.MILLISECONDS + diff.SECONDS * 1000;
      
      if ((diff_ms > max_client_no_heartbeat) && (client_connected)) {
        reinit_wifi_connection();
        _time_get_elapsed(&last_heartbeat);
      }

}


/*TASK*-------------------------------------------------------------------
 *
 * Task Name : hub_main_loop
 * Comments  : Called when user types in "starttx"
 *
 *END*----------------------------------------------------------------------*/
void atheros_loop(uint32_t dummy) {
   	uint_32 flags = 0, i=0, ret32;
   	A_INT32 return_code;
   	A_INT32 timeout = UNITY_CONNECT_TOUT;

   	_mutex_init(&cbuf_mutex, NULL);

	atheros_driver_setup();
        _int_install_unexpected_isr();

	for(;;){
		_lwevent_create(&atheros_task_event, 1/* autoclear */);

		/* block for events from other tasks */
		for(;;){
			switch(_lwevent_wait_ticks(&atheros_task_event, 0x01, TRUE, MSEC_HEARTBEAT))
			{
			case MQX_OK:
                                TsDoWork();
				_lwevent_clear(&atheros_task_event, 0x01);
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
        _mutex_destroy(&ack_to_client_mutex);

	_lwevent_destroy(&atheros_task_event);
	quit_atheros_loop();
}

