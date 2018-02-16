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

#ifndef MAX
#define MAX(a,b) ((a>b)?(a):(b))
#endif
#define SPI_COMM_LENGTH MAX(sizeof(nordic_to_sat_dtm_t),sizeof(sat_to_nordic_dtm_t))
	
// Cleared when a transfer begins. Set when we notice it's ended.
static volatile bool m_transfer_completed = true;

// SPI responses are stored here until read.
static sat_spi_data_packet_t spi_rx_data_old;

static sat_to_nordic_dtm_t spi_rx;

trace_data_t trace_data;
//#define CONTROL_SHORTER_THAN_DATA ((sizeof(sat_control_packet_t) + \
//	                                  sizeof(trace_data_t) + 6) \
//																	 <= sizeof(spi_rx_data))
//void assert_valid_struct_sizes(void) {
//  static uint8_t make_your_trace_data_or_control_packet_shorter[CONTROL_SHORTER_THAN_DATA?1:-1];
//	make_your_trace_data_or_control_packet_shorter[0]++;
//}
//#undef CONTROL_SHORTER_THAN_DATA

/// Returns true IFF transfer completed.
bool is_transfer_complete(void) {
	spi_master_state_t spi_driver_state = spi_master_get_state(SPI_MASTER_HW);
	if (!m_transfer_completed && spi_driver_state == SPI_MASTER_STATE_IDLE) {
		m_transfer_completed = true;
		return true;
	}
	return false;
}

/// Adds synchronization and trace data to the command packet.
/// rq_bytes is the output. Must be an array sizeof(spi_rx_data).
void build_raw_command_packet(sat_control_packet_t command,
	                            uint8_t *rq_bytes) {
	memcpy(rq_bytes,&command,sizeof(command));
	memcpy(&(rq_bytes[sizeof(command)]), &trace_data, sizeof(trace_data));
	int sync_pattern_begin = sizeof(sat_to_nordic_dtm_t) - 6;
	for(int i=0;i<6;++i) {
		rq_bytes[sync_pattern_begin+i] = i+0xfa;
	}
}


void init_spi(void) {
	spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;
	spi_config.SPI_Pin_SCK = TS_SPI_CLK_PIN;
	spi_config.SPI_Pin_SS = TS_SPI_SS_PIN;
	spi_config.SPI_Pin_MISO = TS_SPI_MISO_PIN;
	spi_config.SPI_Pin_MOSI = TS_SPI_MOSI_PIN;
	spi_config.SPI_CONFIG_ORDER = SPI_CONFIG_ORDER_MsbFirst;
	
	spi_config.SPI_Freq = SPI_FREQUENCY_FREQUENCY_K250;
	spi_master_open(SPI_MASTER_HW, &spi_config);
}

bool spi_send_command(sat_control_packet_t command, spi_callback callback) {
	static int consecutive_busy = 0;

	if(!m_transfer_completed) return false;
	
//	// Begin transfer.
//	static uint8_t rq_bytes[sizeof(spi_rx_data)];
//	build_raw_command_packet(command,rq_bytes);
//	uint32_t error_code =	spi_master_send_recv(SPI_MASTER_HW,
//			(uint8_t*)(rq_bytes), sizeof(spi_rx_data),
//			(uint8_t*)&spi_rx_data, sizeof(spi_rx_data));
//	
			
	uint32_t error_code =	spi_master_send_recv(SPI_MASTER_HW,
			(uint8_t*)(&command), sizeof(command),
			(uint8_t*)&spi_rx_data_old, sizeof(spi_rx_data_old));
	
			
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
	if(callback) {
		callback(spi_rx_data_old);
	}
	return true;
}

bool spi_send_command_diag(nordic_to_sat_dtm_t spi_tx, spi_callback_diag callback) {
	static int consecutive_busy = 0;

	if(!m_transfer_completed) return false;	

	/* 06/17/16 cwati SPI transmit and Receive should be the same.
   * I saw some inconsistency in the SPI data if size is different.
   * This issue might just originate from 22F, but let's make it the
   * same for now.	*/
	static uint8_t spi_tx_arr[SPI_COMM_LENGTH];
	static uint8_t spi_rx_arr[SPI_COMM_LENGTH];

	memcpy(&spi_tx_arr, &spi_tx, sizeof(spi_tx));
	memcpy(&spi_rx_arr, &spi_rx, sizeof(spi_rx));
	
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
