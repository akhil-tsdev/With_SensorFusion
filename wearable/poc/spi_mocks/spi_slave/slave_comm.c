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
 
#include "slave_comm.h"
#include "spi_slave.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "simple_uart.h"
#include <stdio.h>
#include "../spi_types.h"
#include "queue.h"
#include <math.h>

//#define TX_BUF_SIZE   16u               /**< SPI TX buffer size. */      
//#define RX_BUF_SIZE   TX_BUF_SIZE       /**< SPI RX buffer size. */      
#define DEF_CHARACTER 0xAAu             	/**< SPI default character. Character clocked out in case of an ignored transaction. */      
#define ORC_CHARACTER 0x55u             	/**< SPI over-read character. Character clocked out after an over-read of the transmit buffer. */      
#define	MAX_Q_SIZE		100									/**< TODO define max queue. */

#define	DEBUG					1
static uint32_t current_timestamp = 0;

static bool sensing = false;
static bool sending = false;

typedef struct padded_nord {
	uint32_t 							buf[1];
	nordic_to_hub_t  			packet;
} padded_nordic_to_hub_t;

padded_nordic_to_hub_t				padded_tx;
static nordic_to_hub_t 				tx_data; 

static hub_to_nordic_t 				rx_command;

char *command_name(sat_spi_command_t command) {
	switch(command) {
#define enum2str(q) case q:\
		return #q ;
		enum2str(SAT_SPI_START)
		enum2str(SAT_SPI_STOP)
		enum2str(SAT_SPI_WAIT)
		enum2str(SAT_SPI_CALIBRATE)
		enum2str(SAT_SPI_SET_RTC)
#undef enum2str
		case SAT_SPI_INVALID:
		default:
			return "[Invalid]";
	}
}

void tick(void) {
	static uint32_t theta = 0;
	theta++;
	CRITICAL_REGION_ENTER();
	uint32_t ts = current_timestamp++;
//	if(sensing) {
	if (sensing && (sz() < MAX_Q_SIZE)) {
		q_entry_t data;
		data.timestamp = current_timestamp;
		data.theta = theta;
		push(data);
	}
	CRITICAL_REGION_EXIT();
	simple_uart_putstring((uint8_t*)".");
}

void process_command(void) {
	char msg[200];
	static sat_spi_command_t last_traced_command = SAT_SPI_INVALID;
	uint32_t	command;
	
#ifdef DEBUG
	simple_uart_putstring((uint8_t*)("\r\nReceiving:"));
	snprintf(msg, 199, "\r\ncommand = 0x%x rtc_value = 0x%x", rx_command.command, rx_command.payload.rtc_value);
	simple_uart_putstring((uint8_t*)msg);
#endif
/*	
	snprintf(msg,199,"\r\nreceived %s (#%d) [%d|%d]",command_name(rx_command.command),rx_command.command,sz(),current_timestamp);
	if(current_timestamp%100==0 || (current_timestamp%10==0 && rx_command.command != last_traced_command)) {
		simple_uart_putstring((uint8_t*)msg);
		last_traced_command = rx_command.command;
	}
*/

	static bool redzone = false;
	if(sz() >= 100) {
		if(!redzone)
		{
			redzone = true;
			simple_uart_putstring((uint8_t*)"\r\n==== RED ZONE ====");
		}
	} else {
		redzone = false;
	}
	
	command = rx_command.command;
	
#if DEBUG
	simple_uart_putstring((uint8_t*)"\r\nCurrent Status ");
#endif
	if (command & NORDIC_START) {
		sensing = true;
#if DEBUG
		simple_uart_putstring((uint8_t*)" sensing YES");
#endif
	} else {
		sensing = false;
#if DEBUG
		simple_uart_putstring((uint8_t*)" sensing NO");
#endif
	}

	if (command & NORDIC_WAIT) {
		sending = false;
#if DEBUG
		simple_uart_putstring((uint8_t*)" sending NO");
#endif
	} else {
		sending = true;
#if DEBUG
		simple_uart_putstring((uint8_t*)" sending YES");
#endif
	}
	
	if (command & NORDIC_SET_RTC) {
		current_timestamp = rx_command.payload.rtc_value;
#if DEBUG
		simple_uart_putstring((uint8_t*)" set new RTC YES");
#endif
	}
#if DEBUG
		simple_uart_putstring((uint8_t*)"\n");
#endif
}
			

/**@brief Function for initializing buffers.
 *
 * @param[in] p_tx_buf  Pointer to a transmit buffer.
 * @param[in] p_rx_buf  Pointer to a receive  buffer.
 * @param[in] len       Buffers length.
 */
static __INLINE void spi_slave_buffers_init(uint8_t * const p_tx_buf, uint8_t * const p_rx_buf, const uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        p_tx_buf[i] = (uint8_t)('a' + i);
        p_rx_buf[i] = 0;
    }
}

/**@brief Function for checking if received data is valid.
 *
 * @param[in] p_rx_buf  Pointer to a receive  buffer.
 * @param[in] len       Buffers length.
 *
 * @retval true     Buffer contains expected data.
 * @retval false    Data in buffer are different than expected.
 */
static bool __INLINE spi_slave_buffer_check(uint8_t * const p_rx_buf, const uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        if (p_rx_buf[i] != (uint8_t)('A' + i))
        {
            return false;
        }
    }
    return true;
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
		char msg[200];
	
    if (event.evt_type == SPI_SLAVE_XFER_DONE)
    {
			process_command();
			if(sending){
				q_entry_t generated_data;
				if(pop(&generated_data)) {	
					uint32_t	theta = generated_data.theta;
					/* cwati satellite ID is using timestamp TODO */
					tx_data.timestamp = generated_data.timestamp;
					tx_data.satellite_id = 0xAA001122;
					tx_data.data.quat_w = 0;
					tx_data.data.quat_x = sin(theta/100.0);
					tx_data.data.quat_y = sin(theta/100.0);
					tx_data.data.quat_z = cos(theta/100.0);
					simple_uart_putstring((uint8_t*)("\r\nGenerating data from pop"));
				}else{
					tx_data.satellite_id = INVALID_TIMESTAMP;
					simple_uart_putstring((uint8_t*)("\r\nFailed pop"));
				}
			} else {
					tx_data.satellite_id = INVALID_TIMESTAMP;
					simple_uart_putstring((uint8_t*)("\r\nNOT sending"));
			}
			
			padded_tx.buf[0] = 0xaabbccdd;
			padded_tx.packet = tx_data;

#ifdef DEBUG
			simple_uart_putstring((uint8_t*)("\r\nSending:"));
			memset (msg, 199, '\0');
			snprintf(msg, 199,"\r\nTimestamp:   0x%x"
												"\r\nSat ID  :    0x%02x"
												"\r\nfloat[0]: 		%f"
												"\r\nfloat[1]: 		%f"			
												"\r\nfloat[2]: 		%f"
												"\r\nfloat[3]: 		%f",
				tx_data.timestamp,
				tx_data.satellite_id,
				tx_data.data.quat_w,
				tx_data.data.quat_x,
				tx_data.data.quat_y,
				tx_data.data.quat_z);
				simple_uart_putstring((uint8_t*)msg);
			simple_uart_putstring((uint8_t*)"\r\n< ");
			for(int i=0;i<sizeof(tx_data);++i) {
				snprintf(msg,199,"%02.02x ",((uint8_t*)(&tx_data))[i]);
			}
			simple_uart_putstring((uint8_t*)">");
#endif
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
        
    err_code = spi_slave_evt_handler_register(spi_slave_event_handle);
    APP_ERROR_CHECK(err_code);    

		tx_data.satellite_id=1;
	
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
    
    //Initialize buffers.
    //spi_slave_buffers_init(m_tx_buf, m_rx_buf, (uint16_t)TX_BUF_SIZE);
    
    //Set buffers.
			err_code = spi_slave_buffers_set(
			    (uint8_t*)&padded_tx, (uint8_t*)&rx_command,
			    sizeof(padded_tx), sizeof(rx_command));
    APP_ERROR_CHECK(err_code);            
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, false);

    return NRF_SUCCESS;
}
