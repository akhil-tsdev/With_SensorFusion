#include <stdbool.h>
#include "sat_spi_types.h"
#include "dtm_common_types.h"

typedef void (*spi_callback)(sat_spi_data_packet_t);

typedef __packed struct {
	uint32_t requests_seen;
	uint32_t my_requests_seen;
	uint32_t rows_sent;
	uint16_t bad_requests_seen;
	uint8_t buffer_size;
} trace_data_t;
/// Trace data to be included in the SPI packet.
extern trace_data_t trace_data;

/* DTM */
extern uint16_t last_received_crc16;

/// Initializes/reinitializes SPI library.
void init_spi(void);
/// Sends the control packet to the satellite Freescale and blocks
/// until communication is finished. On success, calls spi_callback()
/// with the response packet and returns true. Otherwise, returns false.
/// Global side effects. Call only on the main loop.
bool spi_send_command(sat_control_packet_t, spi_callback);

typedef struct spi_tx_ {
	uint32_t numtx[2];
	uint16_t crc16;
} spi_tx_t;

typedef struct spi_rx_ {
	uint32_t numrx[2];
	uint16_t crc16;
} spi_rx_t;

typedef void (*spi_callback_diag)(sat_to_nordic_dtm_t);

bool spi_send_command_diag(nordic_to_sat_dtm_t tx, spi_callback_diag callback);
