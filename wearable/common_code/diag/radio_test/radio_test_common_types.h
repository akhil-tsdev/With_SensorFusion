/*
 * Copyright (c) 2016 TuringSense Inc. All rights reserved.
 *
 * File: radio_test_common_types.h
 *
 */

#ifndef __RADIO_TEST_COMMON_TYPES_H__
#define __RADIO_TEST_COMMON_TYPES_H__

#include "stdint.h"

//#define RADIO_TEST_VERSION_RELEASE	"0.1"
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define RADIOTEST_VERSION_MAJOR 0
#define RADIOTEST_VERSION_MINOR 2

#define RADIOTEST_VERSION_RELEASE_22F 5
#define RADIOTEST_VERSION_RELEASE_NRD 0



#define VERSION_RELEASE	TOSTRING(RADIOTEST_VERSION_MAJOR)"."TOSTRING(RADIOTEST_VERSION_MINOR)"."TOSTRING(RADIOTEST_VERSION_RELEASE_22F)"."TOSTRING(RADIOTEST_VERSION_RELEASE_NRD)



#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef MAX
#define MAX(a,b) ((a>b)?(a):(b))
#endif
#define SPI_COMM_LENGTH MAX(sizeof(nordic_to_mk22f_radio_test_t),sizeof(mk22f_to_nordic_radio_test_t))
	
#ifdef PACK_PRAGMA
#define OPTIONALLY_PACKED_TYPEDEF_STRUCT(definition,name) typedef PACK_PRAGMA struct definition packed_ ## name; typedef struct definition name;
#else
#define OPTIONALLY_PACKED_TYPEDEF_STRUCT(definition,name) typedef struct definition name;
#endif

typedef struct mk22f_to_nordic_radio_test_ {
	char 		command;
	uint8_t		mode;
	uint8_t		start_channel;
	uint8_t		end_channel;
	uint8_t		delayms;
	uint8_t		power;
	uint16_t 	crc16;
} mk22f_to_nordic_radio_test_t;


typedef struct nordic_to_mk22f_radio_test_ {
	uint8_t 	padding; /* First byte always has the first bit set... */
	uint16_t 	last_crc16_recvd;
	uint16_t 	crc16;
	uint8_t		mode;
	uint8_t		tx_power;
	uint8_t		channel_start;
	uint8_t		channel_end;
	uint8_t		delayms;
} nordic_to_mk22f_radio_test_t;

typedef struct  {
	uint32_t spi_attemptLimit;
	uint32_t spi_numberOfTests;
	uint32_t spi_max_crc_fail;
	uint32_t spi_max_attempt_for_packet;
	uint32_t spiSensors_numberOfTests;
	uint32_t spiSensor_max_WR_fail;
	uint8_t log_enableDetail;
} parameters_t;

enum _spi_instances
{
    /* PRODUCTION1 or eval board */
    knRF51822SpiInstance = 1,  /* SPI1 is connected to Nordic nRF51822 */
    kMpu9250SpiInstance = 0,   /* SPI0 is connected to Invensense MPU9250 */
};

#endif /* __RADIO_TEST_COMMON_TYPES_H__ */

