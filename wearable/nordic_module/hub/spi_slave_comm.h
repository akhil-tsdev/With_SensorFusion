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

// Set to 0 for normal operation.
// Set to 1 for output quaternion data for python cube display, don't send data to Hub's K22F
#define OUTPUT_3D_CUBE_ONLY  0

/// Initializes the hub SPI. Returns NRF_ codes (so NRF_SUCCESS on success).
uint32_t spi_slave_example_init(void);

// Diagnostic statistics.
extern uint32_t rows_pushed;
extern uint32_t rows_discarded;
//extern uint32_t rows_popped;

// Interface is entirely property-based.
/// The queue for sensor records to be sent to the Freescale. Push rows into it.
extern cbuf_t outbound_queue;
// Set by commands from the Freescale.
extern bool start_dumping;
extern bool should_set_timestamp;
extern uint32_t current_timestamp;
extern nordic_to_hub_t tx_data; 

extern uint32_t cur_sat_ids[MAX_SATELLITES];
uint8_t get_sat_idx(uint32_t sat_ID);

#endif // SPI_SLAVE_EXAMPLE_H__

/** @} */
