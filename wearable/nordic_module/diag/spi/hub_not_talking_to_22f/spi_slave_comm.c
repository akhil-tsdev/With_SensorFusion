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
#include <math.h>
#include "../common/hub_spi_types.h"

#include "hub_main.h"
#include "hub_timer.h"										/* hub RTC */
#include "net_common.h"										/* LCP */
#include "spi_slave_comm.h"

#define DEF_CHARACTER 0xAAu             	/**< SPI default character. Character clocked out in case of an ignored transaction. */      
#define ORC_CHARACTER 0x55u             	/**< SPI over-read character. Character clocked out after an over-read of the transmit buffer. */      
#define	MAX_Q_SIZE		100									/**< TODO define max queue. */

//#define	DEBUG					1
#if DEBUG
#define DEBUG_PRINT(str) simple_uart_putstring((uint8_t*)(str))
#else
#define DEBUG_PRINT(str) do{}while(0)
#endif

uint32_t current_timestamp = 0;

cbuf_t outbound_queue;

uint32_t rows_pushed=0;
uint32_t rows_discarded=0;
uint32_t rows_popped=0;

uint32_t current_command = 0xFFFFFFFF;//NORDIC_INVALID;
	
padded_nordic_to_hub_t				padded_tx;
static nordic_to_hub_t 				tx_data; 
static nordic_to_hub_t 				empty_tx_data; 

static hub_to_nordic_t 				rx_command;

float cached_sin[256] = {0};
static void prep_tx_pkt(nordic_to_hub_t* tx_data);

static void set_spi_buffer() {
	static uint32_t timestamp = 0;
	static int32_t adder = 0;
	static uint32_t sat_idx = 0;
	const uint32_t max_sat_idx = MAX_SENSORS - 1;
	
	/* Wow, this data to hub contains maximum 6 records for now -___-
	 * so for 14-satellites case, it will take like 3 spi transcation -___-
   * Suppose we have max sat of 14, so max_sat_idx will be 13.
	 * With MAX_SENSOR_RECORDS_PER_PACKET 6, this loop iteration will yield
	 * the following sat_idx:
	 *  0  1  2  3   4  5 
	 *  6  7  8  9  10 11
	 * 12 13  0  1   2  3
	 * 4 ... and so on and so forth...
	 */ 
	for(int i=0;i<MAX_SENSOR_RECORDS_PER_PACKET; ++i) {
		
		if (adder + i > max_sat_idx) {
			adder = (-i);
			timestamp++;
		}
		sat_idx = adder + i;
		
		tx_data.record[i].timestamp = timestamp;
		tx_data.record[i].sensor_idx = sat_idx;
		tx_data.record[i].data.quat_w = 0;
		tx_data.record[i].data.quat_x = cached_sin[timestamp%256];
		tx_data.record[i].data.quat_y = cached_sin[timestamp%256];
		tx_data.record[i].data.quat_z = cached_sin[(timestamp+64)%256];
	}
	
	//Set adder for next iteration
	adder = sat_idx + 1;
	if (adder > max_sat_idx) {
		adder = 0;
		timestamp++;
	}
	prep_tx_pkt(&tx_data);
}

static void prep_tx_pkt(nordic_to_hub_t* tx_data) {
		padded_tx.packet = *tx_data;
		padded_tx.crc16 = crc16_compute((uint8_t*)(&padded_tx.packet),sizeof(padded_tx.packet),0);
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
			set_spi_buffer();

			padded_nordic_to_hub_t tmp = padded_tx;
			// TODO: sizeof(padded_tx) is exactly 256 B, which looks like it would
			// wrap around to 0 B when passed into this interface. In other words,
			// looking at this code, I'd expect no data to actually be sent back to
			// the Freescale. But for some reason, it works instead!
			// We may want to revisit this and either rewrite it or figure out what
			// is going on...
			err_code = spi_slave_buffers_set(
			    (uint8_t*)&padded_tx, (uint8_t*)&rx_command,
		    sizeof(padded_tx), sizeof(rx_command));
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
        
		cbufInit(&outbound_queue);
		// Float operations and sin/cos in particular are mega slow.
		// Precalculate the values so that the spinner can use them.
		if(!cached_sin[64]) {
			for(int i=0;i<256;++i) {
				cached_sin[i]=sin(i*6.28/256);
			}
		}
	
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
	
	for(int i=0;i<MAX_SENSOR_RECORDS_PER_PACKET; ++i) {
		empty_tx_data.record[i].timestamp = INVALID_TIMESTAMP;
		empty_tx_data.record[i].sensor_idx = UINT8_MAX;
		empty_tx_data.record[i].data.quat_w = 0xaa;
		empty_tx_data.record[i].data.quat_x = 0xbb;
		empty_tx_data.record[i].data.quat_y = 0xcc;
		empty_tx_data.record[i].data.quat_z = 0xdd;
	}
	
	prep_tx_pkt(&empty_tx_data);
	
	//cw debug
	padded_nordic_to_hub_t tmpcwati = padded_tx;
	
//Set buffers.
	err_code = spi_slave_buffers_set(
			(uint8_t*)&padded_tx, (uint8_t*)&rx_command,
			sizeof(padded_tx), sizeof(rx_command));
    APP_ERROR_CHECK(err_code);   
#if EVAL_BOARD		
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, false);
#endif
    return NRF_SUCCESS;
}
