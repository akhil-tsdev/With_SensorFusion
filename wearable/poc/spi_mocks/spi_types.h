#include <stdint.h>
#include <stdio.h>
#include "common_types.h"


#define TS_SPI_CLK_PIN      29    /**< SPI clock GPIO pin number. */
#define TS_SPI_MOSI_PIN     25    /**< SPI Master Out Slave In GPIO pin number. */
#define TS_SPI_MISO_PIN     28    /**< SPI Master In Slave Out GPIO pin number. */
#define TS_SPI_SS_PIN       24    /**< SPI Slave Select GPIO pin number. */


typedef __packed struct {
	uint8_t record_count;
} sat_data_header_t;

typedef sat_sensor_record_t sat_spi_data_packet_t;


typedef sat_mosi_packet_t sat_control_packet_t;
