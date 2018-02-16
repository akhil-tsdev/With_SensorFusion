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

uint32_t current_command = 0;

#if OUTPUT_3D_CUBE_ONLY
bool start_dumping = true;
#else
bool start_dumping = false;
#endif
bool should_set_timestamp = false;

padded_nordic_to_hub_t				padded_tx;
nordic_to_hub_t 							tx_data; 
static nordic_to_hub_t 				empty_tx_data; 

static hub_to_nordic_t 				rx_command;

float cached_sin[256] = {0};

static bool command_crc_fail(void) {
	uint16_t crc_received, crc_computed;

	crc_received = rx_command.crc;
	rx_command.crc = 0;
	crc_computed = crc16_compute((uint8_t*)&rx_command, sizeof(rx_command), 0);

	if (crc_computed != crc_received) {
		 return true;
	}
	return false;
}

void process_command(void) {
	uint32_t        command;

	if (command_crc_fail()) {
		return;
	}

	current_command = rx_command.payload.satellite_ids[SAT_COMMSTRUCT_COMMA];
	command = current_command;
	
	if (command == SAT_COMM_WIF_SETRTC) {
		should_set_timestamp = true;
		/* Set RTC from hub 22F means to reset it to 0 */
		current_timestamp = 0;
		/* Empty cbuf */
		cbufInit (&outbound_queue);
		DEBUG_PRINT(" set timestamp YES");
	}
	
	if (command == SAT_COMM_WIF_SETSAT) {
		if (rx_command.payload.satellite_ids[SAT_COMMSTRUCT_VALZ3] != 0) {
			set_sat_idx(rx_command.payload.satellite_ids);
			DEBUG_PRINT(" set satellite YES");
		}
	}
	
		//cf 20160518 - change communication channel
		if(command == SAT_COMM_SETBTCOMCH){
			rx_command.payload.satellite_ids[SAT_COMMSTRUCT_VALZ3] = lcp_valid(rx_command.payload.satellite_ids[SAT_COMMSTRUCT_VALX1]); //the result is sent to the SAT's 22F
			if(rx_command.payload.satellite_ids[SAT_COMMSTRUCT_VALZ3]){
				new_lcp = rx_command.payload.satellite_ids[SAT_COMMSTRUCT_VALX1];
			}
		}	//cf 20160714 - communication analysis 
		else if (command == SAT_COMM_TESTCOMSTC) {
			resetPacketCounter();
	  }

	//cf 20161014 - MANAGEMENT OF START DUMPING WITH SAT COMMAND
		if(command == SAT_COMM_START_DUMP){
				start_dumping = true;
				max_quiet_wait = RR_MAX_QUIET_WAIT_DUMPING;
		}
		
		//cf 20161014 - MANAGEMENT OF STOP DUMPING WITH SAT COMMAND
		if(command == SAT_COMM_STOP_DUMPI){
			start_dumping = false;
			max_quiet_wait = RR_MAX_QUIET_WAIT_NODUMPING;
			/* Empty cbuf FAST_DRAIN */
			cbufInit (&outbound_queue);
		}
	
		if(command == SAT_COMM_SET_HUB_ID){
        my_id_hub = rx_command.payload.satellite_ids[SAT_COMMSTRUCT_VALX1];
		}

		if(command == SAT_COMM_SET_HUBLCP){
       
			rx_command.payload.satellite_ids[SAT_COMMSTRUCT_VALZ3] = lcp_valid(rx_command.payload.satellite_ids[SAT_COMMSTRUCT_VALX1]);
			if(rx_command.payload.satellite_ids[SAT_COMMSTRUCT_VALZ3]){
				new_lcp = rx_command.payload.satellite_ids[SAT_COMMSTRUCT_VALX1];
				if (set_lcp(new_lcp)) {
					reinit_esb();
				}
			}
		}
	
		update_cmd_to_sat(&rx_command.payload.satellite_ids[0]);	

	if (command == SAT_COMM_WIF_SETLCP) {
		if (lcp_valid(rx_command.payload.cmd_field1)) {
			new_lcp = rx_command.payload.cmd_field1;
			DEBUG_PRINT(" set lcp YES");
		} else {
			DEBUG_PRINT(" set lcp FAIL, lcp unchanged");
		}
		//CF ONLY FOR TESTS COMMENTED 23/04/2016
		//set_sat_com(rx_command.payload.cmd_field2, rx_command.payload.satellite_ids);
	}
	
	
	
		
//	DEBUG_PRINT("\r\n");
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
		uint32_t tmp_ack;
    if (event.evt_type == SPI_SLAVE_XFER_DONE)
    {
			if(start_dumping){
//				static uint32_t theta = 0;

				for(int data_slot=0; data_slot < MAX_SENSOR_RECORDS_PER_PACKET; ++data_slot) {
					sat_sensor_record_t slot_data;
					err_t pop_result = cbufPop(&outbound_queue, &slot_data);
					if(pop_result == E_OK) {
						rows_popped++;
						tx_data.record[data_slot] = slot_data;
					} else {
						// No more to send. Spin the spinner to indicate that we have free space.
						// TODO: the spinner is harmless for now but should be removed in production
						tx_data.record[data_slot] = empty_tx_data.record[data_slot];
					}
				}
			} else {
				if (tx_data.ack) {
					/* We are not sending data, but we are sending ACKs. */
					tmp_ack = tx_data.ack;
					tx_data = empty_tx_data;
					tx_data.ack = tmp_ack;
				} else {
					// No sending at all. Invalid out all slots.
					tx_data = empty_tx_data;
				}
			}
			
			prep_tx_pkt(&tx_data);

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
			
			/* Only send acknowledgment once */
			if (tx_data.ack & HUB_RTC_ACK) {
				tx_data.ack &= ~HUB_RTC_ACK;
			}
			
			process_command();
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
	
	tx_data.ack = 0;
	prep_tx_pkt(&empty_tx_data);

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
