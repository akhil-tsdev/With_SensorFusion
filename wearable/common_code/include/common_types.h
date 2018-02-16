/*
 * Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * chrish
 *
 */

#ifndef COMMON_TYPES_H_
#define COMMON_TYPES_H_

#include <stdbool.h>
#include "stdint.h"
#include "common_err.h"
#include "frame_types.h"

#define MAJOR_VERSION           0
#define MINOR_VERSION           18

#define REVIS_VERSION_SAT22F    20
#define REVIS_VERSION_SATNRD    12
#define REVIS_VERSION_HUB22F    14
#define REVIS_VERSION_HUBNRD    12

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define REVIS_VERSION           REVIS_VERSION_SAT22F+REVIS_VERSION_SATNRD+REVIS_VERSION_HUB22F+REVIS_VERSION_HUBNRD

#define RELEASE_DATE            "August 11th, 2016"

#define MAX_SENSORS_IN_CS		7 /* We use external sensors only. */

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifdef PACK_PRAGMA
#define OPTIONALLY_PACKED_TYPEDEF_STRUCT(definition,name) typedef PACK_PRAGMA struct definition packed_ ## name; typedef struct definition name;
#else
#define OPTIONALLY_PACKED_TYPEDEF_STRUCT(definition,name) typedef struct definition name;
#endif

/* INITIAL LCP CHANNEL DEFINITION
//
//LCP   TX     RX      Frequency
//0     0      0       2.400
//1     3      3       2.403
//2     6
//3     9
//4     12
//...
//7     LCP*3  LCP*3
//...
//28    84
*/
#define INITIAL_LCP			28
#define MAX_LCP			    28

#define DEFAULT_HUB_ID 0xabcd1
#ifdef NO_NORDIC_BOOTLOADER
#define NORDIC_BL_WAIT_MS               0
#define NORDIC_BL_WAIT2_MS              0
#else
#define NORDIC_BL_WAIT_MS               100 /* wait this many ms at the bootloader */
#define NORDIC_BL_WAIT2_MS              900 /* wait this many extra ms after soft-resetting nordic */
#endif /* !NORDIC_BOOTLOADER */

#define BATTERY_READ_MS                 5000 /* How many ms before each battery read */

#if ENABLE_MAG
/* With MAG enabled, packet size is 31 bytes, with MAG disabled, pkt size 
 * (sat_sensor_record_t) is 31 bytes.  This is using "packed" structs.
 * Nordic has limitation on its SPI for uint8_t bytes == 256 bytes.
 */
#define MAX_SENSOR_RECORDS_PER_PACKET 	8
#else
#define MAX_SENSOR_RECORDS_PER_PACKET 	9
#endif

#if PIVOT_3_0
#define MAX_SENSORS           16
#define MAX_SATELLITES	      8
#else
#define MAX_SENSORS           8
#define MAX_SATELLITES	      8
#endif
#define MAX_SAT_COMMSTRUCT    6 //5 values: SAT_ID - SAT_COMMAND - SAT_VALUE 1 - SAT_VALUE 2 - SAT_VALUE 3 - SAT_VALUE 4
#define SAT_COMMSTRUCT_SATID  0
#define SAT_COMMSTRUCT_COMMA  1
#define SAT_COMMSTRUCT_VALX1  2
#define SAT_COMMSTRUCT_VALY2  3
#define SAT_COMMSTRUCT_VALZ3  4
#define SAT_COMMSTRUCT_VALT4  5
//SAT_COMMAND LIST
#define SAT_COMM_MIN         0  /* Used by hub to note that it's satellite's command */
#define SAT_COMM_IDLE_CYCLE  SAT_COMM_MIN
#define SAT_COMM_TESTVALUE0  8
#define SAT_COMM_TESTVALUE1  1
#define SAT_COMM_SETBTCOMCH  2 //set BT change comunication channel (read ltc value from SAT_COMMSTRUCT_VALX1)
#define SAT_COMM_TESTCOMSTC  3 //enable packages counter to test the communication protocols
#define SAT_COMM_SETWFCOMCH  4 //set WIFI change comunication channel (read WIFI value from SAT_COMMSTRUCT_VALX1)
#define SAT_COMM_SETBTCOMAU  5 //set BT change comunication channel automatically - generated from SAT NORDIC when non receive packets from HUB
#define SAT_COMM_START_DUMP  6 //start dumping
#define SAT_COMM_STOP_DUMPI  7 //stop dumping
#define SAT_COMM_MBIASSENDG  10 //command 10 - MAG BIAS SENDING MODE
#define SAT_COMM_NORMLSENDG  11 //SAT_COMM_NORMLSENDG
#define SAT_COMM_SETMAGBIAS  12 //command 12 - NORMAL SENDING MODE
#define SAT_COMM_MSTBIASENDG 13 //command 13 - MAG BIAS SETTED SENDING MODE
#define SAT_COMM_B_GYRO_EST  14 //command 14 - RUN GYRO BIAS ESTIMATION
#define SAT_COMM_B_ACCE_EST  15 //command 14 - RUN GYRO BIAS ESTIMATION
#define SAT_COMM_BACCGYRSND  16 //command 15 - SEND CURRENT GYRO BIAS
#define SAT_COMM_GETRELEASE  18 //command 18 - VRITE RELEASE FIRMWARE VERSION
#define SAT_COMM_SETACCSENS  19 //command 19 - SET ACC SENSITIVITY
#define SAT_COMM_SETACCBIAS  20 //command 20 - SET ACC BIAS
#define SAT_COMM_SETGYRBIAS  21 //command 21 - SET GYR BIAS
#define SAT_COMM_TSSELFTEST  22 //command 22 - RUN TS SELF TEST
#define SAT_COMM_FLASHWRITE  23 //command 23 - WRITE ID AND/OR CALIB PARAMETERS IN THE FLASH MEMORY
#define SAT_COMM_FLASH_READ  24 //command 24 - READ ID AND/OR CALIB PARAMETERS FROM THE FLASH MEMORY
#define SAT_COMM_FLASHERASE  25 //command 25 - ERASE FLASH MEMORY
#define SAT_COMM_GET_ALLIDS  26 //command 26 - GET SAT ID, MPU SAT ID, HUB ID, MPU HUB ID, LCP
#define SAT_COMM_BATTERY_LV  27 //command 27 - battery level
#define SAT_COMM_RESTOREMAG  28 //command 28 - RESTORE FACTORY MAG BIAS
#define SAT_COMM_FRQDIVIDER  29 //command 29 - SET FREQUENCY DIVIDER
#define SAT_COMM_WRHUBFLASH  30 //COMMAND 30 - WRITE THE HUB FLASH
#define SAT_COMM_RDHUBFLASH  31 //COMMAND 31 - READ THE HUB FLASH
#define SAT_COMM_ERHUBFLASH  32 //COMMAND 32 - ERASE THE HUB FLASH
#define SAT_COMM_STOPCALACC  33 //COMMAND 33 - STOP THE ACC CALIB ROUTINES
#define SAT_COMM_SETQTMAGMD  34 //COMMAND 34 - DETERMINATE IF SAT WORK IN ALTERNATE QUATERNION MODE (PAR1=0), only QUAT MODE (PAR=1), only MAG MODE (PAR=2)
#define SAT_COMM_SETQTTIMES  35 //COMMAND 35 - TIMESTAMP SEND
#define SAT_COMM_SETNODTRPR  36 //COMMAND 36 - SET NORDIC TRASMISSION PARAMETERS
#define SAT_COMM_NEWKFENABL	 37 //COMMAND 37 - SET NEW KF RUNNING MODE
#define SAT_COMM_SENDACKFRM  38 //COMMAND 38 - SEND ACK FRAME MODE ENABLED
#define SAT_COMM_HUBCONFIG1  39 //COMMAND 39 - SET HUB PAR1=multiplier for tismestamp_zone PAR2=Nordic max number of retransmission
#define SAT_COMM_MAX         39 /* Used by hub to know that it's a valid command for sat */

//cf command defined for HUB command removal

/* Hub-specific command
 *
 * If you add command that you don't need to forward to satellite,
 * please modify function 'command_is_for_sat_also'.
 */
#define SAT_COMM_CWIFI_WAIT  1000 //COMMAND 1000 - WIFI_WAIT
#define SAT_COMM_CWIFI_DIAG  1001 //COMMAND 1001 - WIFI_DIAG
#define SAT_COMM_WIF_SETSAT  1002 //COMMAND 1002 - WIFI_SET_SATELLITES
#define SAT_COMM_WIF_SETRTC  1003 //COMMAND 1003 - WIFI_SET_RTC
#define SAT_COMM_WIF_SETLCP  1004 //COMMAND 1004 - WIFI_SET_LCP
#define SAT_COMM_WIF_SETCHA  1005 //COMMAND 1004 - WIFI_SET_CHANNEL
#define SAT_COMM_WIFI_PING   1006 //COMMAND 1006 - WIFI_PING
#define SAT_COMM_WIF_SETSA2  1007 //COMMAND 1007 - WIFI_SET_SATELLITES ONLI FOR SAT
#define SAT_COMM_SET_HUB_ID  1000001 // INTERNAL COMMAND 1000001 - SET HUB ID
#define SAT_COMM_SET_HUBLCP  1000002 // INTERNAL COMMAND 1000002 - SET HUB LCP

/* Command is a valid command for satellite */
static inline bool command_is_for_sat(uint32_t cmd) {
	if (cmd <= SAT_COMM_MAX) {
		return true;
	}
	return false;
}

/* Don't forward to sat 22f, else it will blink red and people
 * are wondering if something goes wrong. */
static inline bool command_is_for_sat_nordic_only(uint32_t cmd) {
	if (cmd == SAT_COMM_WIF_SETSA2) {
		return true;
	}
	return false;
}

/* Don't need to forward to hub Nordic */
static inline bool command_is_for_hub22f_only (uint32_t cmd) {
  if ((cmd == SAT_COMM_WIFI_PING) || \
      (cmd == SAT_COMM_WIF_SETCHA)|| \
      (cmd == SAT_COMM_SENDACKFRM))
  {
    return true;
  }
  return false;  
}

/* Don't need to forward to sat */
static inline bool command_is_for_hub_only(uint32_t cmd) {
	if ((cmd == SAT_COMM_SET_HUBLCP) || (cmd == SAT_COMM_SET_HUB_ID)) {
		return true;
	}
	return false;
}

#define SAT_COMM_SETMAGBIAS_MULT  1000
// OUTPUT MODES
#define	OUTPUT_MODE_QUAT 0
#define OUTPUT_MODE_BACC 1
#define OUTPUT_MODE_BGYR 2
#define OUTPUT_MODE_BMAG 3
#define OUTPUT_MODE_COUN 4
#define OUTPUT_MODE_VERS 5
#define OUTPUT_MODE_HIDS 6
#define OUTPUT_MODE_BATL 7
#define OUTPUT_MODE_QUMA 8
#define OUTPUT_MODE_MAGO 9
#define OUTPUT_MODE_TIME 10

//CF 01/06/2016 - COMMUNICATION PROTOCOL IMPROVEMENT
#define CPI_QUATERNION16_MULTIPLIER 6500.0f //this value guarantees for the w component values ??from 100 to 110 for the management of the BIAS
#define CPI_QUATERNION16_BIAS_MULTIPLIER 66.0f //this value guarantees for the w component values ??from 100 to 110 for the management of the BIAS
#define CPI_QUATERNION16_BIASSEN_COMMAND 100.0f //DO NOT MODIFY
#define CPI_QUATERNION16_AGBISEN_COMMAND 101.0f //DO NOT MODIFY
#define CPI_QUATERNION16_COUNTER_COMMAND 111.0f //DO NOT MODIFY
#define CPI_QUATERNION16_VERSION_COMMAND 112.0f //DO NOT MODIFY
#define CPI_QUATERNION16_GET_IDS_COMMAND 113.0f //DO NOT MODIFY
#define CPI_QUATERNION16_BATTERY_COMMAND 114.0f //DO NOT MODIFY
#define CPI_QUATERNION16_MAGONQU_COMMAND 115.0f
#define CPI_QUATERNION16_TIMESTA_COMMAND 116.0f //DO NOT MODIFY
#define CPI_QUATERNION16_NOCOMND_COMMAND 117.0f //DO NOT MODIFY - do not use like frameType mode
#define CPI_QUATERNION16_SYNCHRO_COMMAND 118.0f //DO NOT MODIFY - used for synchronizzation
#define CPI_QUATERNION16_COUNTER_MAXVALU 32768 //DO NOT MODIFY

/* Acknowledgment fields 
 * 32-bit
 * 0xFF_FF_FF_FF
 *   |  |  |  |
 *  N/A |  |  |
 *     N/A |  |
 *        LCP |
 *           ACK
 */
#define HUB_LCP_ACK_MASK                    0xFF
#define HUB_LCP_ACK_OFFSET                  8
#define SET_HUB_LCP(field, lcp)             (field |= ((lcp & HUB_LCP_ACK_MASK) << HUB_LCP_ACK_OFFSET))
#define GET_HUB_LCP(num)                    ((num & (HUB_LCP_ACK_MASK << HUB_LCP_ACK_OFFSET)) >> HUB_LCP_ACK_OFFSET)
#define GET_HUB_ACK(num)                    (num & HUB_LCP_ACK_MASK)

#define HUB_RTC_ACK                         (1 << 0)

/* Macros for checking nordic state */
#define NORDIC_START_IS_SET(c) (c & NORDIC_START)
#define NORDIC_CMD_SET_START(c) (c |= NORDIC_START)
#define NORDIC_CMD_CLR_START(c) (c &= ~NORDIC_START)

#define NORDIC_WAIT_IS_SET(c) (c & NORDIC_WAIT)
#define NORDIC_CMD_SET_WAIT(c) (c |= NORDIC_WAIT)
#define NORDIC_CMD_CLR_WAIT(c) (c &= ~NORDIC_WAIT)

#define NORDIC_START_IS_SET(c) (c & NORDIC_START)
#define NORDIC_CMD_SET_START(c) (c |= NORDIC_START)
#define NORDIC_CMD_CLR_START(c) (c &= ~NORDIC_START)

#define NORDIC_RTC_IS_SET(c) (c & NORDIC_SET_RTC)
#define NORDIC_CMD_SET_RTC(c) (c |= NORDIC_SET_RTC)
#define NORDIC_CMD_SET_SAT(c) (c |= NORDIC_SET_SATELLITES)

/* Data structure definition */

/* Hub/22F to sensor */
typedef enum {
	SENSOR_INVALID, SENSOR_SLEEP, SENSOR_ACTIVE, SENSOR_CALIBRATE
} command_to_sensor_t;

/* From Hub's Nordic to Satellite's Nordic
 * ESB = Enhanced Shock Burst
 */
typedef enum {
	ESB_INVALID,
	ESB_NOT_YOU,
	ESB_START,
	ESB_STOP,
	ESB_WAIT,
	ESB_CALIBRATE,
	ESB_SET_RTC
} esb_command_t;

/* From Satellite's Nordic to Hub's Nordic */
typedef enum {
	ESB_CONNECT, ESB_DATA
} esb_request_type_t;

// Minimum requirement to send to Cloud (per Pietro) is raw accel and quaternion
// Keeping all 3 raw sensor data (accel, gyro, mag) in case we do sensor fusion in the Cloud
#if _IAR
typedef __packed struct sensor_record_ {
#else
typedef struct __attribute__((packed)) sensor_record_ {
#endif
	int16_t accel_x;	// Accelerometer's x-axis value
	int16_t accel_y;	// Accelerometer's y-axis value
	int16_t accel_z; 	// Accelerometer's z-axis value
	int16_t gyro_x;		// Gyroscope's x-axis value
	int16_t gyro_y;		// Gyroscope's y-axis value
	int16_t gyro_z;		// Gyroscope's z-axis value
#if ENABLE_MAG
	int16_t mag_x;		// Magnetometer's x-axis value
	int16_t mag_y;// Magnetometer's y-axis value
	int16_t mag_z;// Magnetometer's z-axis value
#endif
	int16_t quat_w;	// Quarternion's scalar component, rotation degree around the vector
	int16_t quat_x;		// Quarternion's x vector component
	int16_t quat_y;		// Quarternion's y vector component
	int16_t quat_z;		// Quarternion's z vector component
} sensor_record_t;

typedef struct sensor_record_wifi_ {
	int16_t accel_x;	// Accelerometer's x-axis value
	int16_t accel_y;	// Accelerometer's y-axis value
	int16_t accel_z; 	// Accelerometer's z-axis value
	int16_t gyro_x;		// Gyroscope's x-axis value
	int16_t gyro_y;		// Gyroscope's y-axis value
	int16_t gyro_z;		// Gyroscope's z-axis value
#if ENABLE_MAG
	int16_t mag_x;		// Magnetometer's x-axis value
	int16_t mag_y;// Magnetometer's y-axis value
	int16_t mag_z;// Magnetometer's z-axis value
#endif
	int16_t quat_w;	// Quarternion's scalar component, rotation degree around the vector
	int16_t quat_x;		// Quarternion's x vector component
	int16_t quat_y;		// Quarternion's y vector component
	int16_t quat_z;		// Quarternion's z vector component
} sensor_record_wifi_t;

typedef struct sensor_record_mag_ {
	int16_t accel_x;	// Accelerometer's x-axis value
	int16_t accel_y;	// Accelerometer's y-axis value
	int16_t accel_z; 	// Accelerometer's z-axis value
	int16_t gyro_x;		// Gyroscope's x-axis value
	int16_t gyro_y;		// Gyroscope's y-axis value
	int16_t gyro_z;		// Gyroscope's z-axis value
	int16_t mag_x;		// Magnetometer's x-axis value
	int16_t mag_y;		// Magnetometer's y-axis value
	int16_t mag_z;		// Magnetometer's z-axis value
	float quat_w;// Quarternion's scalar component, rotation degree around the vector
	float quat_x;		// Quarternion's x vector component
	float quat_y;		// Quarternion's y vector component
	float quat_z;		// Quarternion's z vector component
	//float lnaccel_x;	// Linear acceleration's x vector component
	//float lnaccel_y;	// Linear acceleration's y vector component
	//float lnaccel_z;	// Linear acceleration's z vector component
} sensor_record_mag_t;

#define INVALID_TIMESTAMP UINT32_MAX
#define INVALID_SAT_ID		UINT32_MAX	/* Sat ID here is the HW ID */
#define INVALID_SAT_IDX		UINT8_MAX		/* Sat idx is the index of the array where a particular
															 * sat ID is located. */

#define INVALID_SENSOR_IDX      UINT8_MAX		

#if (SAT_22F || SAT_NORDIC)
#define MAX(a,b) ((a>b)?(a):(b))
#define SPI_COMM_LENGTH MAX(sizeof(sat_mosi_packet_t),sizeof(sat_miso_packet_t))
#endif /* SAT */

/* Hub's Nordic to Hub */
#if _IAR
typedef __packed struct sat_sensor_record_ {
#else
typedef struct
	__attribute__((packed)) {
#endif
		uint8_t sensor_idx;	// Satellite index given by Hub --> Nordic will track this
		uint32_t timestamp;	// RTC value of Satellite's 22F, init/synched by Hub
		sensor_record_t data;	// Sensor's raw output + sensor fusion output
	} sat_sensor_record_t;

#if _IAR
	typedef __packed struct {
#else
	typedef struct
		__attribute__((packed)) {
#endif
			uint32_t ack; /* Acknowledgement to hub */
			uint16_t crc;
			sat_sensor_record_t record[MAX_SENSOR_RECORDS_PER_PACKET];
		} hub_spi_data_packet_t;

		/* There's an issue in SPI transaction from Nordic to hub22F, where the
		 * 1st byte always has bit 7 set.  So if we send 0x00, the hub will receive
		 * 0x80.
		 */
#if _IAR
		typedef __packed struct padded_nord {
#else
		typedef struct __attribute__((packed)) padded_nord {
#endif
			uint16_t crc16;
			hub_spi_data_packet_t packet;
		} padded_nordic_to_hub_t;

		/* From cloud to hub's wifi,
		 * same as from wifi to Hub,
		 * same as from Hub to Nordic
		 *
		 * Command				      |	cmd_field1		    |	cmd_field2
		 *----------------------|-------------------|--------------------
		 * WIFI_CALIBRATE	      | 	N/A (unused)	  | N/A (unused)
		 * WIFI_SET_RTC			    |	rtc value		      |
		 * WIFI_SET_SATELLITES	|	N/A (unused)	    | num_of_sat
		 * WIFI_SET_LCP			    |	lcp channel		    | N/A (unused)
		 * WIFI_SET_CHANNEL		  | 	wifi channel	  | N/A (unused)
		 *
		 */
		typedef struct {
			uint32_t cmd_field1;
			uint32_t cmd_field2;
			uint32_t satellite_ids[MAX_SAT_COMMSTRUCT]; /* currently valid satellite IDs */
		} hub_command_payload_t;

///* Hub to wifi */
//typedef struct {
//	uint32_t timestamp;		/* This timestamp should match each sat_sensor_record's data.timestamp. */
//	sat_sensor_record_t record[MAX_SENSOR_RECORDS_PER_PACKET];
//} hub_data_packet_t;

		/*****************************************************
		 * Data type for:
		 *  Communication from satellite's 22F -----> wifi
		 *  22F -> nordic -> nordic -> Hub -> wifi
		 *****************************************************/
		typedef struct {
			uint8_t record_count;
			sensor_record_t records[MAX_SENSOR_RECORDS_PER_PACKET];
		} data_packet_t;

//		/* The packets sent from 22F to Nordic contains satellite ID,
//		 * but we won't push that into this Nordic's cbuf */
//		typedef struct {
//			uint32_t timestamp;		// RTC value of Satellite's 22F, init/synched by Hub
//			uint8_t  sensor_idx;		// In sat 22f this is local sensor index, in
//			sensor_record_t data;	// Sensor's raw output + sensor fusion output
//		} sat_cbuf_packet_t;

		/* from 22F to satellite's nordic
		 */
		typedef struct {
			uint32_t sat_id;
			uint32_t hub_id;
			sat_sensor_record_t data;// Sensor's raw output + sensor fusion output
//	sensor_record_t data;	// Sensor's raw output + sensor fusion output
			uint16_t crc16;
			uint16_t nrdChannel;
		} sat_miso_packet_t;

		/* satellite's nordic to hub's nordic */
		typedef struct {
			uint32_t satellite_id;
			esb_request_type_t type;
			data_packet_t sensor_data;
		} esb_request_packet_t;
//
///* hub's nordic to Hub */
//typedef struct {
//	uint32_t satellite_id;
//	data_packet_t sensor_data;
//} sat_data_packet_t;

		/* Hub to wifi */
		typedef struct {
			uint32_t satellite_id;
			sensor_record_wifi_t record;
		} payload_to_wifi_t;

		typedef struct {
			uint32_t satellite_id;
			sensor_record_mag_t record;
		} payload_to_wifi_mag_t;

		typedef struct {
			uint32_t hubId;
			uint32_t ack; /* Acknowledgement from hub */
			uint32_t timestamp; /* This timestamp should match each sat_sensor_record's data.timestamp. */
			uint32_t bitmap; /* Set bit means valid data */
			payload_to_wifi_t data[MAX_SENSORS];
		} data_to_wifi_t;

		typedef struct {
			uint32_t hubId;
			uint32_t ack; /* Acknowledgement from hub */
			uint32_t timestamp; /* This timestamp should match each sat_sensor_record's data.timestamp. */
			uint32_t bitmap; /* Set bit means valid data */
			payload_to_wifi_mag_t data[MAX_SENSORS];
		} data_to_wifi_mag_t;

		/*****************************************************
		 * Data type for:
		 *  Communication from Wifi -----> satellite's 22F
		 *  22F <- nordic <- nordic <- Hub <- wifi
		 *****************************************************/
		/* From satellite's nordic to 22F */
		typedef struct {
			uint32_t sat_command[MAX_SAT_COMMSTRUCT]; //CF: [0] SAT ID to send command - 0 mens ALL SAT; [1] command; [2] value 1; [3] value 2; [4] value 3;
			uint16_t crc16;
		} sat_mosi_packet_t;

		/* From cloud to hub's wifi,
		 * same as from wifi to Hub,
		 * similar to from Hub to Nordic, but might have different content,
		 * for example if wifi is not busy, but Hub busy, then
		 * wifi doesn't set "WAIT" while Hub might set "WAIT" to Nordic.
		 * User request--> wifi-->Hub-->Nordic */
		typedef struct {
			uint16_t crc;
			hub_command_payload_t payload;
		} wifi_request_packet_t;

		/* Typedef to make it easier to see the flow... */
		typedef wifi_request_packet_t cloud_to_hub_t;
		typedef wifi_request_packet_t hub_to_nordic_t;

		typedef hub_spi_data_packet_t nordic_to_hub_t;
		typedef data_to_wifi_t hub_to_cloud_t;
		typedef data_to_wifi_mag_t hub_to_cloud_mag_t;
		typedef sat_miso_packet_t sat_to_nordic_t;
		typedef sat_mosi_packet_t nordic_to_sat_t;
#endif /* COMMON_TYPES_H_ */
