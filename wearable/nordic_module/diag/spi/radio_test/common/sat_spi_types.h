#include <stdint.h>
#include <stdio.h>
#include "common_types.h"

#ifndef SAT_SPI_TYPES_H_
#define SAT_SPI_TYPES_H_

#ifdef EVAL_NRF51DK
// Nordic Eval nRF51-DK SPI pins
#define TS_SPI_CLK_PIN      29   /**< SPI clock GPIO pin number. */
#define TS_SPI_MOSI_PIN     25   /**< SPI Master Out Slave In GPIO pin number. */
#define TS_SPI_MISO_PIN     28   /**< SPI Master In Slave Out GPIO pin number. */
#define TS_SPI_SS_PIN       24   /**< SPI Slave Select GPIO pin number. */
#else
// TuringSense Sensor Module SPI pins
#define TS_SPI_CLK_PIN      10   /**< SPI clock GPIO pin number. */
#define TS_SPI_MOSI_PIN      8   /**< SPI Master Out Slave In GPIO pin number. */
#define TS_SPI_MISO_PIN      9   /**< SPI Master In Slave Out GPIO pin number. */
#define TS_SPI_SS_PIN       11   /**< SPI Slave Select GPIO pin number. */
#endif

typedef __packed struct {
	uint8_t record_count;
} sat_data_header_t;

typedef satellite_to_nordic_t sat_spi_data_packet_t;


typedef sat_mosi_packet_t sat_control_packet_t;
#endif
