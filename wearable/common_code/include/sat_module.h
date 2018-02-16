/* Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * sat_module.h
 *
 *  Created on: Mar 7, 2015
 *      Author: cwati
 */

#ifndef SAT_MODULE_H_
#define SAT_MODULE_H_
#include "common_types.h"

//define this in project
//#define PROTO_BOARD		0

// Satellite definitions and data structures
//#define SAT_SENSOR_ID	1	// Satellite SensorID is given by the Hub at init time, HOW??
//#define CB_SIZE 		209	// Queue size for sensor records to send to Nordic
//#define CB_TYPE 		sat_sensor_record_t  // Queue entry

#define MAXWELL_BAUD_RATE			230400
#define REV_F_UART_INSTANCE			0

typedef enum {
	SAT_INVALID,
	SAT_ACTIVE,
	SAT_SLEEP,
} sat_status_t;

typedef enum {
	SAT_SENDING_INVALID,
	SAT_SENDING_ACTIVE,
	SAT_SENDING_STOP,
} sat_sending_status_t;

typedef enum {
	SAT_INVALID_CALIBRATE,
	SAT_DO_CALIBRATE,
	SAT_DONT_CALIBRATE,
} sat_calibrate_status_t;

// SPI configurations at TuringSense Sensor Module */
enum _spi_instances
{
#if PROTO_BOARD
    knRF51822SpiInstance = 0,  /* SPI0 is connected to Nordic nRF51822 */
    kSensorSpiInstance = 1,   /* SPI1 is connected to Invensense MPU9250 */
#else
    /* PRODUCTION1 or eval board */
    knRF51822SpiInstance = 1,  /* SPI1 is connected to Nordic nRF51822 */
    kSensorSpiInstance = 0,   /* SPI0 is connected to Invensense MPU9250 */
#endif
};

/* The packets sent from 22F to Nordic contains satellite ID,
  * but we won't push that into this Nordic's cbuf */
 typedef struct {
 	uint32_t timestamp;		// RTC value of Satellite's 22F, init/synched by Hub
 #if PIVOT_3_0
 	sensor_record_t data[MAX_SENSORS_IN_CS];	// Sensor's raw output + sensor fusion output
 #else
 	sensor_record_t data;	// Sensor's raw output + sensor fusion output
 #endif
 } sat_cbuf_packet_t;

 typedef struct sensor_info_ {
 	uint8_t uart_num;
 	uint8_t cs_type;
 } sensor_info_t;

#endif /* SAT_MODULE_H_ */
