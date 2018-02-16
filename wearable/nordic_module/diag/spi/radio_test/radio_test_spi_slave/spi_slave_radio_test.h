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

#ifndef __SPI_SLAVE_RADIO_TEST_H__
#define __SPI_SLAVE_RADIO_TEST_H__

#include <stdint.h>

extern nordic_to_mk22f_radio_test_t	spi_tx;
extern mk22f_to_nordic_radio_test_t spi_rx;
/// Initializes the hub SPI. Returns NRF_ codes (so NRF_SUCCESS on success).
uint32_t spi_slave_example_init(void);

#endif /* __SPI_SLAVE_RADIO_TEST_H__ */
