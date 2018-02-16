
/*
 * Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * chrish
 *
 */

#ifndef DTM_COMMON_TYPES_H_
#define DTM_COMMON_TYPES_H_
#include "stdint.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define DTM_VERSION_MAJOR 2
#define DTM_VERSION_MINOR 0

#define SATDTM_VERSION_RELEASE_22F 3
#define SATDTM_VERSION_RELEASE_NRD 0
#define HUBDTM_VERSION_RELEASE_22F 0
#define HUBDTM_VERSION_RELEASE_NRD 0

#define VERSION_RELEASE	TOSTRING(DTM_VERSION_MAJOR)"."TOSTRING(DTM_VERSION_MINOR)"."TOSTRING(SATDTM_VERSION_RELEASE_22F)"."TOSTRING(SATDTM_VERSION_RELEASE_NRD)"."TOSTRING(HUBDTM_VERSION_RELEASE_22F)"."TOSTRING(HUBDTM_VERSION_RELEASE_NRD)


#define DTM_PKT_TYPE_PRBS9				0
#define DTM_PKT_TYPE_PRBS15				1
#define DTM_PKT_TYPE_11110000			2
#define DTM_PKT_TYPE_01010101			3
#define DTM_PKT_UNMODULATED				4
#define DTM_PKT_TYPE_10101010			5

#define DTM_PKT_TYPE_MIN					DTM_PKT_TYPE_PRBS9
#define DTM_PKT_TYPE_MAX					DTM_PKT_TYPE_10101010

#define DTM_FREQ_MIN							0
#define DTM_FREQ_MAX							83	/* USA */
#define DTM_FREQ_DEFAULT                        0

#define DTM_PKT_LEN_MAX						252 /* UESB_CORE_MAX_PAYLOAD_LENGTH */
#define DTM_PKT_LEN_MIN						1

#define DTM_ENUM_POWER_MAX				7
#define DTM_ENUM_POWER_DEFAULT                                  0

#define GET_DTM_TX_INTERVAL_US(c) (c * 1000)

#if (HUB_22F || HUB_NORDIC)
typedef struct hub_to_nordic_dtm_ {
#else
typedef struct sat_to_nordic_dtm_ {
#endif
	/* 06/18/16, default to Tx */
	uint16_t cmd_type;

	/* Interval (ms) between each transmit */
	uint16_t tx_interval;

	/* 0-83 only, i.e., 2,400?2,483.5 MHz */
	uint16_t dtm_tx_freq;

	/* Maximum is 252 (UESB_CORE_MAX_PAYLOAD_LENGTH) */
	uint16_t dtm_tx_pkt_len;

	/* DTM Power
	* 0: +4dBm   DEFAULT
	* 1: 0dBm
	* 2: -4dBm
	* 3: -8dBm
	* 4: -12dBm
	* 5: -16dBm
	* 6: -20dBm
	* 7: -30dBm
	*/
	uint16_t dtm_power;

	/* Packet type
	* 0: prbs9
	* 1: prbs15
	* 2: 11110000
	* 3: 01010101
        * 4: unmodulated
	*/
	uint16_t dtm_pkt_type;

	/* To validate the SPI transcation */
	uint16_t crc16;
	uint32_t randomFrom22F;
#if (HUB_22F || HUB_NORDIC)
  } hub_to_nordic_dtm_t;
#else
  } sat_to_nordic_dtm_t;
#endif

typedef struct nordic_to_hub_dtm_ {
	uint16_t padding; /* First byte always has the first bit set... */
	uint16_t last_crc16_recvd;
	uint16_t crc16;
	uint32_t randomFrom22F;
} nordic_to_hub_dtm_t;

typedef struct nordic_to_sat_dtm_ {
	uint16_t last_crc16_recvd;
	uint16_t crc16;
	uint32_t randomFrom22F;
} nordic_to_sat_dtm_t;


typedef struct  {
	uint32_t spi_attemptLimit;
	uint32_t spi_numberOfTests;
	uint32_t spi_max_crc_fail;
	uint32_t spi_max_attempt_for_packet;
	uint32_t spiSensors_numberOfTests;
	uint32_t spiSensor_max_WR_fail;
	uint8_t log_enableDetail;
} parameters_t;


extern parameters_t parameters;

#if (HUB_22F || HUB_NORDIC)
#define SPI_DTM_TRANSFER_SIZE	(sizeof(hub_to_nordic_dtm_t))
#else
#define SPI_DTM_TRANSFER_SIZE	(sizeof(sat_to_nordic_dtm_t))
#endif
#endif /* DTM_COMMON_TYPES_H_ */
