/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

// TODO: expunge Freescale code so that we can remove that header? 8]

#ifndef SPI_SLAVE_EXAMPLE_H__
#define SPI_SLAVE_EXAMPLE_H__

#include <stdbool.h>
#include <stdint.h>

#include "cbuf.h"


/// Initializes the hub SPI. Returns NRF_ codes (so NRF_SUCCESS on success).
uint32_t spi_slave_example_init(void);

// Diagnostic statistics.
extern uint32_t rows_pushed;
extern uint32_t rows_discarded;
extern uint32_t rows_popped;

// Interface is entirely property-based.
/// The queue for sensor records to be sent to the Freescale. Push rows into it.
extern cbuf_t outbound_queue;
// Set by commands from the Freescale.
extern bool sensing;
extern bool sending;
extern bool should_set_timestamp;
extern uint32_t current_timestamp;

#endif // SPI_SLAVE_EXAMPLE_H__

/** @} */
