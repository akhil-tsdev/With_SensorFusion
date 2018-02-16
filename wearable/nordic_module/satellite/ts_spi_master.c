/*
 * Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * ts_spi_master.c
 *
 */
#include "ts_spi_master.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "app_error.h"
#include "app_util_platform.h"
#include "common_types.h"
#include "crc16.h"
#include "nrf_delay.h"
#include "bsp.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "spi_master.h"
#include "nordic_common.h"
#include "simple_uart.h"
#include "sat_spi_types.h"

#define STALL_REINIT_THRESHOLD 3
#define TRANSFER_TIMEOUT_US 5000
#define SPI_MASTER_HW SPI_MASTER_0

// Cleared when a transfer begins. Set when we notice it's ended.
static volatile bool m_transfer_completed = true;

// SPI responses are stored here until read.
static sat_to_nordic_t spi_rx;
//static nordic_to_sat_t spi_tx;

//trace_data_t trace_data;

/// Returns true IFF transfer completed.
bool is_transfer_complete(void) {
	spi_master_state_t spi_driver_state = spi_master_get_state(SPI_MASTER_HW);
	if (!m_transfer_completed && spi_driver_state == SPI_MASTER_STATE_IDLE) {
		m_transfer_completed = true;
		return true;
	}
	return false;
}

///// Adds synchronization and trace data to the command packet.
///// rq_bytes is the output. Must be an array sizeof(spi_rx_data).
//void build_raw_command_packet(nordic_to_sat_t command,
//	                            uint8_t *rq_bytes) {								
//	command.crc16 = 0;
//	command.crc16 = crc16_compute((uint8_t*)&command, sizeof(command), 0);
//																
//	memcpy(rq_bytes,&command,sizeof(command));
//	memcpy(&(rq_bytes[sizeof(command)]), &trace_data, sizeof(trace_data));
//	int sync_pattern_begin = sizeof(spi_rx_data) - 6;
//	for(int i=0;i<6;++i) {
//		rq_bytes[sync_pattern_begin+i] = i+0xfa;
//	}
//}


void init_spi(void) {
	spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;
	spi_config.SPI_Pin_SCK = TS_SPI_CLK_PIN;
	spi_config.SPI_Pin_SS = TS_SPI_SS_PIN;
	spi_config.SPI_Pin_MISO = TS_SPI_MISO_PIN;
	spi_config.SPI_Pin_MOSI = TS_SPI_MOSI_PIN;
	spi_config.SPI_CONFIG_ORDER = SPI_CONFIG_ORDER_MsbFirst;
#if !KSDK_1_0_0
	spi_config.SPI_Freq = SPI_FREQUENCY_FREQUENCY_K250;
#endif
	spi_master_open(SPI_MASTER_HW, &spi_config);
}

bool spi_send_command(nordic_to_sat_t command, spi_callback callback) {
	static int consecutive_busy = 0;

	if(!m_transfer_completed) return false;	

	/* Preparing crc16 */
	command.crc16 = 0;
	command.crc16 = crc16_compute((uint8_t*) &command, sizeof(command), 0);

	/* 06/17/16 cwati SPI transmit and Receive should be the same.
   * I saw some inconsistency in the SPI data if size is different.
   * This issue might just originate from 22F, but let's make it the
   * same for now.	*/
	static uint8_t spi_tx_arr[SPI_COMM_LENGTH];
	static uint8_t spi_rx_arr[SPI_COMM_LENGTH];

	memcpy(&spi_tx_arr, &command, sizeof(nordic_to_sat_t));
#if KSDK_1_0_0
	int sync_pattern_begin = sizeof(spi_tx_arr) - 6;
	for(int i=0;i<6;++i) {
		spi_tx_arr[sync_pattern_begin+i] = i+0xfa;
	}
#endif /* KSDK_1_0_0 */
//	memcpy(&spi_rx_arr, &spi_rx, sizeof(spi_rx));
	
	uint32_t error_code =	spi_master_send_recv(SPI_MASTER_HW,
			(uint8_t*)(&spi_tx_arr), sizeof(spi_tx_arr),
			(uint8_t*)&spi_rx_arr, sizeof(spi_rx_arr));

	// Check for communication errors.
	if(error_code != NRF_SUCCESS) {
		// Since this is only supposed to be run from the main loop,
		// a busy signal is a sign of a stalled SPI library.
		// Reinit if it persists.
		if(error_code == NRF_ERROR_BUSY) {
			consecutive_busy++;
			if(consecutive_busy >= STALL_REINIT_THRESHOLD) {
				init_spi();
			}
		}
		return false;
	}
			
	// Wait for communication to complete.
	m_transfer_completed = false;
	consecutive_busy = 0;
	int microseconds_since_transfer_began = 0;
	while(!is_transfer_complete()) {
		nrf_delay_us(5);
		microseconds_since_transfer_began += 5;
		if(microseconds_since_transfer_began > TRANSFER_TIMEOUT_US) {
			// SPI should never take this long. Assume it's stalled.
			init_spi();
			return false;
		}
	}

	// Communication completed correctly.
	memcpy(&spi_rx, &spi_rx_arr, sizeof(spi_rx));
	if(callback) {
		callback(spi_rx);
	}
	return true;
}


