/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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
 
#include "spi_slave.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "crc16.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../common/hub_spi_types.h"	/* Pin configuration is similar to hub, where for SPI we're the slave */

#include "spi_slave_radio_test.h"
#include "radio_test_common_types.h"
#include "main_radio_test_spi_slave.h"

#define DEF_CHARACTER 0xAAu             	/**< SPI default character. Character clocked out in case of an ignored transaction. */      
#define ORC_CHARACTER 0x55u             	/**< SPI over-read character. Character clocked out after an over-read of the transmit buffer. */      
#define	MAX_Q_SIZE		100									/**< TODO define max queue. */

//#define	DEBUG					1
#if DEBUG
#define DEBUG_PRINT(str) simple_uart_putstring((uint8_t*)(str))
#else
#define DEBUG_PRINT(str) do{}while(0)
#endif

nordic_to_mk22f_radio_test_t	spi_tx;
mk22f_to_nordic_radio_test_t 	spi_rx;
uint8_t 									spi_tx_arr[SPI_COMM_LENGTH] = {0};
uint8_t 									spi_rx_arr[SPI_COMM_LENGTH] = {0};
uint16_t 									last_crc16_recvd = 0xFF;
float cached_sin[256] = {0};

static bool packet_valid(mk22f_to_nordic_radio_test_t packet) {
	uint16_t crc16_recvd, crc16_gen;
	
	crc16_recvd = packet.crc16;
	packet.crc16 = 0;
	crc16_gen = crc16_compute((uint8_t*)&packet, sizeof(packet), 0);
	
	if (crc16_gen == crc16_recvd) {
		return true;
	}
	
	return false;
}

/**@brief Function for SPI slave event callback.
 *
 * Upon receiving an SPI transaction complete event, LED1 will blink and the buffers will be set.
 *
 * @param[in] event SPI slave driver event.
 */
static void spi_slave_event_handle(spi_slave_evt_t event)
{
    uint32_t err_code;

    if (event.evt_type == SPI_SLAVE_XFER_DONE)
    {
			
			/* Copy over the received data */
			memcpy(&spi_rx, &spi_rx_arr, sizeof(spi_rx));

			/* Check validity of packet */
			if (packet_valid(spi_rx)) {
				/* Update last crc16 received */
				last_crc16_recvd = spi_rx.crc16;
	
				process_command();
			}
			
			/* Prepare next spi tx */
			spi_tx.last_crc16_recvd = last_crc16_recvd = spi_rx.crc16;
			prep_tx_crc16();

			memcpy(&spi_tx_arr, &spi_tx, sizeof(spi_tx));
			
			// TODO: sizeof(spi_tx) is exactly 256 B, which looks like it would
			// wrap around to 0 B when passed into this interface. In other words,
			// looking at this code, I'd expect no data to actually be sent back to
			// the Freescale. But for some reason, it works instead!
			// We may want to revisit this and either rewrite it or figure out what
			// is going on...
			err_code = spi_slave_buffers_set(
			    (uint8_t*)&spi_tx_arr, (uint8_t*)&spi_rx_arr,
		    sizeof(spi_tx_arr), sizeof(spi_rx_arr));
			APP_ERROR_CHECK(err_code);

    }
}

/**@brief Function for initializing SPI slave.
 *
 *  Function configures a SPI slave and sets buffers.
 *
 * @retval NRF_SUCCESS  Initialization successful.
 */
uint32_t spi_slave_example_init(void)
{
    uint32_t           err_code;
    spi_slave_config_t spi_slave_config;
	
    err_code = spi_slave_evt_handler_register(spi_slave_event_handle);
    APP_ERROR_CHECK(err_code);    

    spi_slave_config.pin_miso         = TS_SPI_MISO_PIN;
    spi_slave_config.pin_mosi         = TS_SPI_MOSI_PIN;
    spi_slave_config.pin_sck          = TS_SPI_CLK_PIN;
    spi_slave_config.pin_csn          = TS_SPI_SS_PIN;
    spi_slave_config.mode             = SPI_MODE_0;
    spi_slave_config.bit_order        = SPIM_MSB_FIRST;
    spi_slave_config.def_tx_character = 0xff;
    spi_slave_config.orc_tx_character = 0xff;
    
    err_code = spi_slave_init(&spi_slave_config);
    APP_ERROR_CHECK(err_code);
	
//Set buffers.
	err_code = spi_slave_buffers_set(
			(uint8_t*)&spi_tx, (uint8_t*)&spi_rx,
			sizeof(spi_tx), sizeof(spi_rx));
    APP_ERROR_CHECK(err_code);   
#if EVAL_BOARD		
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, false);
#endif
    return NRF_SUCCESS;
}
