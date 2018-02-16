/*
 * Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * chrish
 *
 */

#ifndef COMMON_TYPES_H_
#define COMMON_TYPES_H_

#include "stdint.h"
#include "common_err.h"
#include "frame_types.h"

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

#if ENABLE_MAG
/* With MAG enabled, packet size is 42 bytes.
 * Nordic has limitation on its SPI for uint8_t bytes == 256 bytes.
 */
#define MAX_SENSOR_RECORDS_PER_PACKET 	5
#else
#define MAX_SENSOR_RECORDS_PER_PACKET 	6
#endif

#define MAX_SENSORS						11				// Including sensor residing in Hub
#define MAX_SAT_SENSORS				(MAX_SENSORS - 1)	// Satellite sensors only, no Hub
#define MAX_SAT_COMMSTRUCT    5 //5 values: SAT_ID - SAT_COMMAND - SAT_VALUE 1 - SAT_VALUE 2 - SAT_VALUE 3
#define SAT_COMMSTRUCT_SATID  0 
#define SAT_COMMSTRUCT_COMMA  1 
#define SAT_COMMSTRUCT_VALX1  2 
#define SAT_COMMSTRUCT_VALY2  3 
#define SAT_COMMSTRUCT_VALZ3  4 
//SAT_COMMAND LIST
#define SAT_COMM_TESTVALUE0  0
#define SAT_COMM_TESTVALUE1  1
#define SAT_COMM_MBIASSENDG  10 //command 10 - MAG BIAS SENDING MODE
#define SAT_COMM_NORMLSENDG  11 //SAT_COMM_NORMLSENDG
#define SAT_COMM_SETMAGBIAS  12 //command 12 - NORMAL SENDING MODE
#define SAT_COMM_MSTBIASENDG 13 //command 13 - MAG BIAS SETTED SENDING MODE
#define SAT_COMM_SETMAGBIAS_MULT  1000

/* From Cloud to Hub's Wifi */
#define WIFI_INVALID				(1 << 0)
#define WIFI_START					(1 << 1)
#define WIFI_DIAG						(1 << 2)
#define WIFI_WAIT			        (1 << 3)
#define WIFI_CALIBRATE				(1 << 4)
#define WIFI_SET_RTC				(1 << 5)
#define WIFI_SET_SATELLITES			(1 << 6)
#define WIFI_SET_LCP				(1 << 7)	/* Change Nordic channel */
#define WIFI_SET_CHANNEL			(1 << 8)	/* Change wifi channel.
												 * Will require client to restart connection to hub */
#define WIFI_VALID_DATA				(1 << 30)

/* From Hub to Nordic */
#define NORDIC_INVALID				WIFI_INVALID
#define NORDIC_START				WIFI_START
/* Should just use NORDIC_START */
/* #define NORDIC_STOP					WIFI_STOP */
#define NORDIC_WAIT				WIFI_WAIT
#define NORDIC_CALIBRATE			WIFI_CALIBRATE
/* We don't use calibration command, so use the following command
 * for the change_frequency broadcast */
#define NORDIC_CHANGE_FREQ		WIFI_CALIBRATE
#define NORDIC_SET_RTC				WIFI_SET_RTC
#define NORDIC_SET_SATELLITES	WIFI_SET_SATELLITES
#define NORDIC_SET_LCP				WIFI_SET_LCP

/* From Satellite's Nordic to 22F */
#define	SAT_SPI_INVALID				NORDIC_INVALID
#define SAT_SPI_START					NORDIC_START
#define SAT_SPI_WAIT					NORDIC_WAIT
#define SAT_SPI_SET_RTC				NORDIC_SET_RTC
#define SAT_SPI_SET_COM				NORDIC_CALIBRATE
#define SAT_SPI_SET_LCP				NORDIC_SET_LCP

#define HUB_RTC_ACK                         (1 << 0)

/* This stop and start should just be using 1 bit instead of 2
 * I'm just going to use START for now.
 * If START is not set then it means to STOP.
 */
#define WIFI_WAIT_IS_SET(c) (c & WIFI_WAIT)
#define WIFI_CMD_SET_WAIT(c) (c |= WIFI_WAIT)
#define WIFI_CMD_CLR_WAIT(c) (c &= ~WIFI_WAIT)

#define WIFI_START_IS_SET(c) (c & WIFI_START)
#define WIFI_CMD_SET_START(c) (c |= WIFI_START)
#define WIFI_CMD_CLR_START(c) (c &= ~WIFI_START)

#define WIFI_RTC_IS_SET(c) (c & WIFI_SET_RTC)
#define WIFI_CMD_SET_RTC(c) (c |= WIFI_SET_RTC)
#define WIFI_CMD_SET_SAT(c) (c |= WIFI_SET_SATELLITES)

#define WIFI_LCP_IS_SET(c) (c & WIFI_SET_LCP)
#define WIFI_CMD_SET_LCP(c) (c |= WIFI_SET_LCP)

#define WIFI_CMD_SET_CALIB(c) (c |= WIFI_CALIBRATE)
#define WIFI_CMD_CLR_CALIB(c) (c &= ~WIFI_CALIBRATE)

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

#define NORDIC_LCP_IS_SET(c) (c & NORDIC_SET_LCP)
#define NORDIC_CMD_SET_LCP(c) (c |= NORDIC_SET_LCP)

/* Data structure definition */

/* Hub/22F to sensor */
typedef enum {
	SENSOR_INVALID,
	SENSOR_SLEEP,
	SENSOR_ACTIVE,
	SENSOR_CALIBRATE
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
	ESB_CONNECT,
	ESB_DATA
} esb_request_type_t;

// Minimum requirement to send to Cloud (per Pietro) is raw accel and quaternion
// Keeping all 3 raw sensor data (accel, gyro, mag) in case we do sensor fusion in the Cloud
typedef struct sensor_record_ {
	int16_t accel_x;	// Accelerometer's x-axis value
	int16_t accel_y;	// Accelerometer's y-axis value
	int16_t accel_z; 	// Accelerometer's z-axis value
	int16_t gyro_x;		// Gyroscope's x-axis value
	int16_t gyro_y;		// Gyroscope's y-axis value
	int16_t gyro_z;		// Gyroscope's z-axis value
#if ENABLE_MAG
	int16_t mag_x;		// Magnetometer's x-axis value
	int16_t mag_y;		// Magnetometer's y-axis value
	int16_t mag_z;		// Magnetometer's z-axis value
#endif
	float quat_w;		// Quarternion's scalar component, rotation degree around the vector
	float quat_x;		// Quarternion's x vector component
	float quat_y;		// Quarternion's y vector component
	float quat_z;		// Quarternion's z vector component
	//float lnaccel_x;	// Linear acceleration's x vector component
	//float lnaccel_y;	// Linear acceleration's y vector component
	//float lnaccel_z;	// Linear acceleration's z vector component
} sensor_record_t;

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
    float quat_w;		// Quarternion's scalar component, rotation degree around the vector
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

/* Hub's Nordic to Hub */
typedef struct {
	uint8_t sat_idx;	// Satellite index given by Hub --> Nordic will track this
	uint32_t timestamp;		// RTC value of Satellite's 22F, init/synched by Hub
	sensor_record_t data;	// Sensor's raw output + sensor fusion output
} sat_sensor_record_t;

typedef struct {
	uint32_t	ack; /* Acknowledgement to hub */
	sat_sensor_record_t record[MAX_SENSOR_RECORDS_PER_PACKET];
} hub_spi_data_packet_t;

/* There's an issue in SPI transaction from Nordic to hub22F, where the
 * 1st byte always has bit 7 set.  So if we send 0x00, the hub will receive
 * 0x80. 
 */
typedef struct padded_nord {
	uint16_t 							crc16;
	hub_spi_data_packet_t	packet;
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
	uint32_t satellite_ids[MAX_SAT_SENSORS];   /* currently valid satellite IDs */
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

/* from 22F to satellite's nordic
 */
typedef struct {
	uint32_t timestamp;		// RTC value of Satellite's 22F, init/synched by Hub
	uint32_t sat_id;
	sensor_record_t data;	// Sensor's raw output + sensor fusion output
	uint16_t crc16;
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
	sensor_record_t record;
} payload_to_wifi_t;

typedef struct {
    uint32_t satellite_id;
    sensor_record_mag_t record;
} payload_to_wifi_mag_t;

typedef struct {
	uint32_t hubId;
	uint32_t ack;           /* Acknowledgement from hub */
	uint32_t timestamp;	/* This timestamp should match each sat_sensor_record's data.timestamp. */
	uint32_t bitmap;	/* Set bit means valid data */
	payload_to_wifi_t data[MAX_SENSORS];
} data_to_wifi_t;

typedef struct {
    uint32_t hubId;
    uint32_t ack;           /* Acknowledgement from hub */
    uint32_t timestamp;	/* This timestamp should match each sat_sensor_record's data.timestamp. */
    uint32_t bitmap;	/* Set bit means valid data */
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
	uint32_t			  command;
	hub_command_payload_t payload;
} wifi_request_packet_t;

/* Typedef to make it easier to see the flow... */
typedef wifi_request_packet_t 	cloud_to_hub_t;
typedef wifi_request_packet_t 	hub_to_nordic_t;

typedef hub_spi_data_packet_t 	nordic_to_hub_t;
typedef data_to_wifi_t 			hub_to_cloud_t;
typedef data_to_wifi_mag_t 		hub_to_cloud_mag_t;
typedef sat_miso_packet_t		satellite_to_nordic_t;
typedef sat_mosi_packet_t		nordic_to_satellite_t;
#endif /* COMMON_TYPES_H_ */
