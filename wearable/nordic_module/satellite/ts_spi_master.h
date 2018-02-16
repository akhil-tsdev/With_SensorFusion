#include <stdbool.h>
#include "sat_spi_types.h"

typedef void (*spi_callback)(sat_to_nordic_t);

//typedef __packed struct {
//	uint32_t requests_seen;
//	uint32_t my_requests_seen;
//	uint32_t rows_sent;
//	uint16_t bad_requests_seen;
//	uint8_t buffer_size;
//} trace_data_t;
///// Trace data to be included in the SPI packet.
//extern trace_data_t trace_data;

/// Initializes/reinitializes SPI library.
void init_spi(void);
/// Sends the control packet to the satellite Freescale and blocks
/// until communication is finished. On success, calls spi_callback()
/// with the response packet and returns true. Otherwise, returns false.
/// Global side effects. Call only on the main loop.
bool spi_send_command(nordic_to_sat_t, spi_callback);
