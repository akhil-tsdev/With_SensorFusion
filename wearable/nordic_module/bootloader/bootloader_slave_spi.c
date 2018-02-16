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
#include "simple_uart.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "bootloader_main.h"
#include "bootloader_slave_spi.h"
#include "hub_timer.h"										/* hub RTC */
#include "bootloader_spi_types.h"

//#define	DEBUG					1
#if DEBUG
#define DEBUG_PRINT(str) simple_uart_putstring((uint8_t*)(str))
#else
#define DEBUG_PRINT(str) do{}while(0)
#endif
	
static padded_nordic_to_sat_boot_t		padded_tx;
static uint8_t spi_tx_arr[SPI_BOOT_LENGTH] = {0};
static uint8_t spi_rx_arr[SPI_BOOT_LENGTH] = {0};

static void prep_tx_pkt(nordic_to_sat_boot_t * spi_boot_tx) {
		padded_tx.packet = *spi_boot_tx;

		padded_tx.crc16 = 0;
		padded_tx.crc16 = crc16_compute((uint8_t*)(&padded_tx),sizeof(padded_tx),0);
		/* cwati 12/17/15 there's an issue in SPI that the first byte received (LSB) at 22F
		 * always has bit 7 set -> 0x80 */
		padded_tx.crc16 |= 0x80; 
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

			/* Copy only what we need */
			memcpy(&spi_boot_rx, &spi_rx_arr, sizeof(spi_boot_rx));

			/* Process RX from 22f */
			process_rx();
			
			/* Prepare packet TX to 22f.  WARNING: This will update the global "padded_tx".
			 * So, you should use padded_tx after this operation, and not spi_boot_tx!!. */
			prep_tx_pkt(&spi_boot_tx);
			memcpy((uint8_t*)&spi_tx_arr, &padded_tx, sizeof(padded_tx));

			// TODO: sizeof(padded_tx) is exactly 256 B, which looks like it would
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

		/* Cwati todo, should I prepare spi_boot_tx here/ */

		/* Prepare buffer that will be sent to 22f */
		err_code = spi_slave_buffers_set(
				(uint8_t*)&spi_tx_arr, (uint8_t*)&spi_rx_arr,
				sizeof(spi_tx_arr), sizeof(spi_rx_arr));
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
}

void spi_slave_deinit(void)
{    
    // Clear any possible pending interrupt.
    NVIC_ClearPendingIRQ(SPI1_TWI1_IRQn);
    
    // Disable IRQ.    
    NRF_SPIS1->INTENSET &= ~((SPIS_INTENSET_ACQUIRED_Enabled << SPIS_INTENSET_ACQUIRED_Pos) |
                          (SPIS_INTENSET_END_Enabled << SPIS_INTENSET_END_Pos));
    NVIC_DisableIRQ(SPI1_TWI1_IRQn);
    
    // Disable SPI slave device.        
    NRF_SPIS1->ENABLE &= ~(SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos);        
    
    return;
}
