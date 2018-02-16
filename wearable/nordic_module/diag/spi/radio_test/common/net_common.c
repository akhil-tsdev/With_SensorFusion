/*
 * Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * net_common.c
 *
 */
#include <stdio.h>
#include <string.h>

#include "net_common.h"

#include "nrf_delay.h"
#include "uesb/micro_esb.h"
#include "uesb/uesb_error_codes.h"
#include "dtm_common_types.h"

//Logical channel pair
//The nordic chip supports frequencies from 2.400 GHz to 2.526 GHz. 
//These are described as "channels" 0 - 126, with each channel having a 1MHz band.
//In the USA, only the frequency range of 2,400–2,483.5 MHz is permitted
//for unlicensed use. In other words, channels 0-83 only. 
//So, if we use StM channel “2”, we need to leave channel “2” and “3”, “4”, 
//and its corresponding MtS channel “5”, and “6” “7”, for a total of 6 channels.
//Which means 14 logical channel pairs, 0 to 13.

static uint32_t lcp = INITIAL_LCP;

// There used to be a logging system here, as payload data during early tests.
// But the old implementation no longer really makes sense.
// I've left the calls in to log events, just in case we want another log later, but
// for now those calls go to the bit bucket.
typedef enum {
	NO_MESSAGE = 0,	INIT, TO_RX, TO_TX,
	OFF, TX_ON, RX_ON, DATA_IN, DATA_OUT,
	OUT_OK, OUT_ERR, ERR_RESET, SATURATED
} log_event_t;
#define log_message(...) do{}while(0)

/// Callback provided at init time. Called when a packet comes in.
static net_rx_callback rx_callback = 0;

// TODO: unsure if this is useful. Since we don't use ESB's built-in
// ACK implementation, packets should never be nacked.
/// Callback provided at init time. Called when a packet is nacked.
static net_nack_callback nack_warning = 0;

// These are provided at init time. In our system, they'll be reversed
// on satellite vs hub.
static uint32_t net_tx_channel = 103;
static uint32_t net_rx_channel = 103;
static uesb_tx_power_t net_tx_power = UESB_TX_POWER_4DBM;
// Redeclared diagnostics.
uint32_t diag_tx_attempt_ct = 0;
uint32_t diag_tx_ack_ct = 0;
uint32_t diag_tx_nack_ct = 0;
uint32_t diag_rx_ct = 0;
uint32_t diag_rx_after_ack_ct = 0;
uint32_t diag_rx_after_nack_ct = 0;
// This one might actually be used for program logic somewhere. Oops!
tx_fate_t fate_of_last_tx = TX_FATE_UNKNOWN;

// The "pipe address" of ESB is a prefix shoved on the beginning
// of packets to give some clue what sort of packets they are.
// I haven't really messed with this feature; these pipe addresses
// are copypasted from somewhere.
#define UESB_PIPE0_ADDR {0x12, 0x34, 0x56, 0x78, 0x9A}
#define UESB_PIPE1_ADDR {0xBC, 0xDE, 0xF0, 0x12, 0x23}
#define UESB_PIPE2_ADDR 0x66

#define UESB_TX_PIPE 0

//size_t sth_payload_size(sth_payload_t *payload_ptr) {
//	return sizeof(payload_ptr->header) + sizeof(payload_ptr->diagnostic_pattern1) + (sizeof(esb_sensor_record_t)*payload_ptr->header.row_ct);
//}

/// Returns false on immediate failures, true otherwise.
bool switch_to_rx(void) {
	log_message(TO_RX);
	uesb_flush_tx();
	
	if(uesb_set_rf_channel(net_rx_channel) != UESB_SUCCESS) return false; 
	if(uesb_start_rx() == UESB_SUCCESS) {
		log_message(RX_ON);
		return true;
	}
	return false;
}

/// Interrupt handler.
void handle_uesb_events(void)
{
	// Be very careful not to let this function get too heavy. It seems
	// to block interrupts; in the past, making this take too long (e.g. by
	// zero-initializing some big structs) caused all kinds of things to
	// break in difficult to diagnose ways.
	static uint32_t rf_interrupts;
	static uint32_t tx_attempts;
    
  uesb_get_clear_interrupts(&rf_interrupts);
    
  if(rf_interrupts & UESB_INT_TX_SUCCESS_MSK) {   
		log_message(OUT_OK);
		fate_of_last_tx = TX_ACKED;
		diag_tx_ack_ct++;

		// Data successfully sent. Start RX again
		switch_to_rx();
  }
    
  if(rf_interrupts & UESB_INT_TX_FAILED_MSK) {
		log_message(OUT_ERR);
		// Data failed to be sent.
		// TODO: figure out whether this is still a possible code path.
		fate_of_last_tx = TX_NACKED;
		diag_tx_nack_ct++;
		if(nack_warning) {
			nack_warning();
		}
		switch_to_rx();
  }
    
	if(rf_interrupts & UESB_INT_RX_DR_MSK)
	{
		log_message(DATA_IN);
		uesb_payload_t payload;

		// Update statistics.
		diag_rx_ct++;
		switch(fate_of_last_tx) {
			case TX_ACKED:
				diag_rx_after_ack_ct++;
				break;
			case TX_NACKED:
				diag_rx_after_nack_ct++;
				break;
			default:
				break;
		}
		uesb_read_rx_payload(&payload);
		// Make sure the packet is big enough to have the sort of data it
		// claims to have.
		if(payload.length >= sizeof(packet_header_t)) {
#if HUB_NORDIC
			sth_packet_t packet = *(sth_packet_t*)(payload.data);
#else /* SAT_NORDIC */
			hts_packet_t packet = *(hts_packet_t*)(payload.data);
#endif
			if(payload.length >= sizeof(packet)) {
				// All looks good. Call back to the program.
				rx_callback(packet);
			}
		}
	}
	// Cargo cult programming. I don't know what this does but IIRC in the examples
	// they call this even without using the output for anything. So I *assume*
	// it's an accumulator that has to be cleared.
	uesb_get_tx_attempts(&tx_attempts);
}

static void init_uesb(void) {
	uint8_t rx_addr_p0[] = UESB_PIPE0_ADDR;
	uint8_t rx_addr_p1[] = UESB_PIPE1_ADDR;
	uint8_t rx_addr_p2   = UESB_PIPE2_ADDR;
	
	uesb_config_t uesb_config       = UESB_DEFAULT_CONFIG;
	uesb_config.rf_channel          = net_rx_channel;
	uesb_config.crc                 = UESB_CRC_OFF;
	// I *think* this translates to not asking for a retransmit at all.
	// But I'm not sure; if this really means "retransmit once", fixing it
	// will probably increase stability.
	uesb_config.retransmit_count    = 1;
	uesb_config.retransmit_delay    = 300;
	// As far as I can tell, the only way to not ask for ack pings is
	// to set dynamic_ack and then disable them for each packet.
	uesb_config.dynamic_ack_enabled = 1;
	uesb_config.protocol            = UESB_PROTOCOL_ESB_DPL;
	// As fast and loud as it'll go.
	uesb_config.bitrate             = UESB_BITRATE_2MBPS;
	uesb_config.tx_output_power     = net_tx_power;
	uesb_config.event_handler       = handle_uesb_events;

	// Reinitialize if UESB is already initialized or is stuck in some state.
	if(uesb_init(&uesb_config) != UESB_SUCCESS) {
		uesb_force_reinit(&uesb_config);
	}

	uesb_set_address(UESB_ADDRESS_PIPE0, rx_addr_p0);
	uesb_set_address(UESB_ADDRESS_PIPE1, rx_addr_p1);
	uesb_set_address(UESB_ADDRESS_PIPE2, &rx_addr_p2);
}

void net_init(net_rx_callback callback, net_nack_callback nack_callback, uint32_t tx_channel, uint32_t rx_channel,
		uesb_tx_power_t tx_power, uint8_t dtm_flags) {
	rx_callback = callback;
	nack_warning = nack_callback;
	net_rx_channel = rx_channel;
	net_tx_channel = tx_channel;
	net_tx_power = tx_power;
			
	fate_of_last_tx = TX_FATE_UNKNOWN;
	
	init_uesb();
	// Bidirectional communication over ESB is achieved by all parties
	// waiting in an RX state, and then switching to TX only when they
	// wish to speak.
	uesb_start_rx();
	log_message(INIT);
	
	if (dtm_flags & DTM_PLL_LOCK_ENABLED) {
		NRF_RADIO->TEST |= (RADIO_TEST_PLL_LOCK_Enabled << RADIO_TEST_PLL_LOCK_Pos);
	} else {
		NRF_RADIO->TEST &= ~(RADIO_TEST_PLL_LOCK_Enabled << RADIO_TEST_PLL_LOCK_Pos);
	}
	
	if (dtm_flags & DTM_CONST_CARRIER_ENABLED) {
		NRF_RADIO->TEST |= (RADIO_TEST_CONST_CARRIER_Enabled << RADIO_TEST_CONST_CARRIER_Pos);
	} else {
		NRF_RADIO->TEST &= ~(RADIO_TEST_CONST_CARRIER_Enabled << RADIO_TEST_CONST_CARRIER_Pos);
	}

}

bool send_packet_dtm(uint8_t* packet, uint8_t pkt_size) {
	uesb_payload_t payload;
	payload.pipe  = UESB_TX_PIPE;
	payload.length = pkt_size;
	payload.noack = true;
	memcpy(payload.data, packet, pkt_size);
	
	log_message(TO_TX);
	// Stop RX; fail if the ESB driver isn't in RX state or if something is wrong.
	if(uesb_stop_rx() != UESB_SUCCESS) return false;
	
	diag_tx_attempt_ct++;
	if(uesb_set_rf_channel(net_tx_channel) != UESB_SUCCESS) {
		// Try to recover.
		uesb_set_rf_channel(net_rx_channel);
		uesb_start_rx();
		return false;
	}
	log_message(TX_ON);

	bool success = (uesb_write_tx_payload_noack(&payload) == UESB_SUCCESS);
	fate_of_last_tx = TX_FATE_UNKNOWN;
	if(success) log_message(DATA_OUT);
	
	return success;
}

#if HUB_NORDIC
bool send_packet(hts_packet_t packet) {
#else /* SAT_NORDIC */
bool send_packet(sth_packet_t packet) {
#endif
	uesb_payload_t payload;
	payload.pipe  = UESB_TX_PIPE;
	payload.length = sizeof(packet);
	payload.noack = true;
	memcpy(payload.data, &packet, sizeof(packet));
	
	log_message(TO_TX);
	// Stop RX; fail if the ESB driver isn't in RX state or if something is wrong.
	if(uesb_stop_rx() != UESB_SUCCESS) return false;
	
	diag_tx_attempt_ct++;
	if(uesb_set_rf_channel(net_tx_channel) != UESB_SUCCESS) {
		// Try to recover.
		uesb_set_rf_channel(net_rx_channel);
		uesb_start_rx();
		return false;
	}
	log_message(TX_ON);

	bool success = (uesb_write_tx_payload_noack(&payload) == UESB_SUCCESS);
	fate_of_last_tx = TX_FATE_UNKNOWN;
	if(success) log_message(DATA_OUT);
	
	return success;
}

uint32_t get_txchan_from_lcp() {
#if HUB_NORDIC
	return (lcp * 6);
#elif SAT_NORDIC
	return ((lcp * 6) + 3);
#endif
}
uint32_t get_rxchan_from_lcp() {
#if HUB_NORDIC
	return ((lcp * 6) + 3);
#elif SAT_NORDIC
	return (lcp * 6);
#endif
}
uint32_t get_lcp() {
	return (lcp);
}

bool lcp_valid(uint16_t new_lcp) {
	return (new_lcp > MAX_LCP ? 0 : 1);
}

/* If set_lcp fails, it returns 0 */
bool set_lcp(uint16_t new_lcp) {
	//Fail to set LCP, return 1.
	if (new_lcp > MAX_LCP) {
		return false;
	}
	lcp = new_lcp;
	return true;
}

uint32_t get_next_lcp() {
	if (lcp >= 13) {
		return 0;
	} else {
		return (lcp + 1);
	}
}
