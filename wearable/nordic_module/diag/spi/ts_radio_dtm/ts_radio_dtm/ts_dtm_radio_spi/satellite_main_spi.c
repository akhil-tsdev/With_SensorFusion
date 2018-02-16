/*
 * Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * satellite_main_spi.c
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "app_util.h"
#include "nrf.h"
#include "uesb/micro_esb.h"
#include "uesb/uesb_error_codes.h"

#include "net_common.h"
#include "cbuf.h"

#include "crc32.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "simple_uart.h"
//#include "fifo.h"

#include "crc16.h"
#include "nrf_delay.h"

#include "led_trace.h"

#include "ts_spi_master.h"

#include "ts_radio_dtm_function.h"


uint8_t g_ts_radio_dtm_debug_level = 0;
uint8_t g_ts_radio_dtm_cw = 0;
uint8_t g_ts_radio_dtm_tx_package_type = 0;
uint16_t g_ts_radio_dtm_tx_package_length = 0;
uint8_t g_ts_radio_dtm_rx_package_type = 0;
uint16_t g_ts_radio_dtm_rx_package_length = 0;
uint8_t g_ts_radio_dtm_tx_channel_id = 0;
uint8_t g_ts_radio_dtm_rx_channel_id = 0;
uesb_tx_power_t g_ts_radio_dtm_power_level = UESB_TX_POWER_4DBM;
bool g_ts_radio_dtm_modulated = true;
uint8_t g_ts_radio_dtm_tx_packet[256];
uint8_t g_ts_radio_dtm_rx_packet[256];
uint16_t dtm_tx_interval_ms = 5;

uint8_t g_ts_radio_dtm_cur_num = 0;
static uint32_t sRandomSeed;

/// Starts at 1 for satellites. Must be different for EVERY satellite.
uint32_t sat_id = INVALID_SAT_ID;
uint32_t my_hub_id = 0xabcd1;
uint32_t sat_id_forCommand = 0;

// Statistics for diagnosing problems.
uint32_t requests_seen = 0;
uint32_t my_requests_seen = 0;
uint32_t laggard_requests_seen = 0;
uint32_t nonsense_requests_seen = 0;
uint32_t rows_sent = 0;
uint32_t packets_generated = 0;
uint32_t ct_excessive_diff = 0;
uint32_t sum_excessive_diff = 0;
uint32_t payload_crc_futile = 0;

// DTM
uint16_t last_received_crc16 = 0xFFFF;
static uint16_t dtm_cmd_type;
static uint16_t dtm_freq;
static uint16_t dtm_pkt_len;
static uint16_t dtm_power;
static uint16_t dtm_pkt_type;
static uint32_t randomFrom22F;
static bool dtm_new_cmd = false;

// When running on an eval stack, every press of Button 4
// will make this function return true once. Used to trigger
// UART logging.
bool control_button_pressed(void);


// Tracks synchronization with the hub. After being generated, a payload packet
// is kept and re-sent until the hub replies with a matching CRC (confirming that
// the packet has arrived safe and sound).
uint16_t expected_crc16 = 0;
uint16_t previous_crc16 = 0;
static sth_payload_t next_payload;
static bool first_payload_generated = false;


// System state, set according to commands from the hub.
bool sensing_enabled = false;
bool sending_enabled = false;
bool should_set_timestamp = false;
uint32_t timestamp_to_set = INVALID_TIMESTAMP;
bool must_calibrate = false;
uint16_t last_rtc_id = 0;
//SAT COMMAND MANAGEMENF CF
uint32_t command_to_sat[MAX_SAT_COMMSTRUCT];

/// Contains readings from the Freescale, enqueued to be sent to the hub.
cbuf_t data_queue;
// If the queue gets too close to full, it'll hit a "red zone" and go
// into a mode focused on draining values. Until it drains, SPI communication
// will be limited; we will only call the Freescale to "refill" rows that were.
// drained.
//static bool redzone_mode = false;
#define CBUF_REDZONE_SIZE ((CB_SIZE)*2/3)
#define CBUF_IS_REDZONE(cbuf) (CBUF_REDZONE_SIZE <= cbufNum(cbuf))
#define CBUF_GREENZONE_SIZE 25

void satellite_rx_handler(hts_packet_t packet);

void initRandom()
{
	sRandomSeed = 0x02;
}


uint32_t getRandom(uint8_t is_prbs9)
{
	uint8_t a = (is_prbs9 ? 6 : 14);
	uint32_t new_v = (((sRandomSeed >> a) ^ (sRandomSeed >> (a-1))) & 0x01);
	sRandomSeed = ((sRandomSeed << 1) | new_v) & 0x7F;
	return sRandomSeed;
}


static void reinit_esb() {
	net_init(satellite_rx_handler, 0, get_txchan_from_lcp(), get_rxchan_from_lcp(), \
		g_ts_radio_dtm_power_level, g_ts_radio_dtm_modulated);
}

/// Builds a satellite-to-hub packet payload using data popped from the queue.
/// You MUST save the result someplace safe, else the rows will be lost!
sth_payload_t new_payload(void) {
	// TODO: This function could probably stand to be optimized a bit.
	// Since it's called just before writing a response to the hub, it's on
	// the critical path.
	FLIP_TRACE(3);
	first_payload_generated = true;
	
	// Initial/default state: a packet with no rows.
	sth_payload_t payload = {0};
	sat_cbuf_packet_t data;
	packets_generated++;
	uint32_t packet_timestamp = INVALID_TIMESTAMP;
	payload.header.row_ct = 0;
	
	if(sending_enabled && cbufPop(&data_queue,&data) == E_OK) {
		// At least one row exists to be sent. Write whole-packet metadata.
		payload.sensor_records[0] = pack_sensor_record(data.data);
		packet_timestamp = data.timestamp;
		uint32_t last_timestamp = packet_timestamp;
		rows_sent++;
		payload.header.row_ct++;
		for(int i=1;i<SENSOR_RECORDS_PER_NET_PACKET;++i) {
			// For each additional row available to send...
			if(cbufPeek(&data_queue,&data) == E_OK) {
				// Rows after the first in a packet don't have a full timestamp.
				// Instead, we send only the diff between the previous timestamp
				// and this one. Normally packets should be something very close to
				// 10ms apart.
				int32_t ts_diff = data.timestamp - (last_timestamp);
				if(ts_diff < 0 || ts_diff > 255) {
					// Anomaly. This row is more than 1/4 second past the previous. Or
					// has jumped BACKWARD in time! Either way, there's not enough room
					// in a ts_diff to describe that. End the packet here, and send this
					// new and surprising row as the first timestamp of the next packet.
					int32_t excessive_diff = (data.timestamp - packet_timestamp);
					if(excessive_diff > 0 && excessive_diff < 10000)
					{
						ct_excessive_diff++;
						sum_excessive_diff += (data.timestamp - packet_timestamp);
					}
					
					// Make sure the packet is properly terminated.
					// TODO: might not be necessary? We do an = {0} up top.
					memset(&(payload.sensor_records[i]), 0, sizeof(esb_sensor_record_t));
					break;
				}
				// Row looks good to include in this packet.
				cbufPop(&data_queue,&data);
				payload.sensor_records[i] = pack_sensor_record(data.data);
				payload.sensor_records[i].ts_diff = ts_diff;
				last_timestamp = data.timestamp;
				rows_sent++;
				payload.header.row_ct++;
			}
			else {
				// TODO: might not be necessary? We do an = {0} up top.
				memset(&(payload.sensor_records[i]), 0, sizeof(esb_sensor_record_t));
			}
		}
	}
	payload.header.timestamp = packet_timestamp;
	payload.header.timestamp_crc = crc16_compute((uint8_t*)(&packet_timestamp),sizeof(packet_timestamp),0);
	payload.diagnostic_pattern1 = 0x50;
	payload.diagnostic_pattern2 = 0xFA;
	payload.header.rtc_id_ack = last_rtc_id;

	expected_crc16 = crc16_compute((uint8_t*)(&payload),sizeof(payload),0);
	return payload;
}

// TODO: this is an out-of-band return value. Find a better way to pass it
// back to the main loop.
uint8_t rows_replied = 0;
/// Checks whether the hub has received the current response payload correctly
/// If so, loads up a new response payload to be sent.
void prepare_response(hts_payload_t request) {
	rows_replied = 0;
	if (request.last_seen_crc16 == expected_crc16 || !first_payload_generated) {
		previous_crc16 = expected_crc16;
		uint32_t rows_sent_before = rows_sent;
		next_payload = new_payload();
		rows_replied = rows_sent - rows_sent_before;
	} else if(request.last_seen_crc16 == previous_crc16) {
		laggard_requests_seen++;
	} else {
		nonsense_requests_seen++;
	}
}

/// Replies to the hub with the currently prepared data payload. Blocks until sending is over.
void send_response(void) {
	sth_packet_t sth_packet;
	
	/* If we haven't got our valid Satellite ID, refrain from sending */
	if (sat_id != INVALID_SAT_ID) {
		sth_packet.header.sender = sat_id;
		sat_id_forCommand = sat_id;
	} else {
		return;
	}
	sth_packet.satellite_to_hub_payload = next_payload;
	
	send_packet(sth_packet);
	int timeout_10us = 0;
	while(fate_of_last_tx == TX_FATE_UNKNOWN) {
		nrf_delay_us(10);
		timeout_10us++;
		if(timeout_10us > 1000) {
			// It's been 10ms. Assume something went wrong. 
			
			// Reset the RF library since we can't know if state is still good.
			reinit_esb();
			uesb_start_rx();
			
			break;
		}
	}

}

// Packets received from the common RF code are stored here, and a flag
// is set for the main loop to pick it up. This is because the event handler
// has to be as short as possible - I've observed repeatedly that long event
// handlers cause ESB library to break in hard-to-debug ways. Still not sure
// why.
volatile bool packet_waiting = false;
volatile hts_packet_t packet_to_process;
void satellite_rx_handler(hts_packet_t packet) {
	if(!packet_waiting) {
		packet_to_process = packet;
		packet_waiting = true;
	}
}

/// Extra validation to be done on the claim that a packet is waiting to be
/// processed. Clears packet_waiting if false.
bool waiting_rx_packet_valid(void) {
	// Only interested in packets from the hub.
	if(packet_to_process.header.sender != my_hub_id) {
		packet_waiting = false;
		return false;
	}
	// Check that the control packet hasn't been corrupted.
	hts_payload_t payload = packet_to_process.hub_to_satellite_payload;
	uint32_t sent_crc32 = payload.command_crc32;
	payload.command_crc32 = 0;
	uint32_t calculated_crc32 = crc32(0xFFFFFFFF, &payload,
	                               sizeof(payload));
	packet_waiting = calculated_crc32 == sent_crc32;
	return packet_waiting;
}

void reset_spi_backoff(void);

void process_command(hts_payload_t payload) {
	bool enable_sensing = false;
	bool enable_command = (sat_id == payload.command_toSat[SAT_COMMSTRUCT_SATID] || 
												 sat_id_forCommand == payload.command_toSat[SAT_COMMSTRUCT_SATID] ||
												 payload.command_toSat[SAT_COMMSTRUCT_SATID] == 0); //cf
	if((sat_id != INVALID_SAT_ID) && !sat_id_forCommand) {
		sat_id_forCommand=sat_id;
	}
		
	if(payload.command & NORDIC_START) {
		enable_sensing = true;
	}
	if(sensing_enabled != enable_sensing) {
		reset_spi_backoff();
	}
	sensing_enabled = enable_sensing;

	sending_enabled = !(payload.command & NORDIC_WAIT);
	if(payload.command & NORDIC_SET_RTC) {
		should_set_timestamp = true;
		timestamp_to_set = payload.rtc_value;
		reset_spi_backoff();
		last_rtc_id = payload.rtc_id;
		//if (payload.rtc_value == 0) {
			cbufInit(&data_queue);
		//}
	}
	
	//sat command management cf
	if((payload.command & NORDIC_CALIBRATE) && enable_command) {
		must_calibrate = true;
		uint8_t cnt;
		for (cnt = 0; cnt < MAX_SAT_COMMSTRUCT; cnt++) {
				command_to_sat[cnt] = payload.command_toSat[cnt];
		}
		
		//KO
		//command_to_sat[2] = 1;
		//command_to_sat[3] = 2;
		//command_to_sat[4] = 3;
	}
	
	if(payload.command & NORDIC_SET_LCP) {
		if(set_lcp(payload.lcp_value)) {
			reinit_esb();
		}
	}
}


void process_rx_packet(hts_packet_t packet) {	
	requests_seen++;
	FLIP_TRACE(1);
	hts_payload_t payload = packet.hub_to_satellite_payload;
	// Obey commands.
	process_command(payload);
	// Only interested if WE were asked to speak.
	if (payload.recipient != sat_id || packet.header.sender != my_hub_id) {
		return;
	};
	
	my_requests_seen++;
	FLIP_TRACE(2);
	prepare_response(payload);
	send_response();
}

void process_packet(hts_packet_t hts_packet) {
	char report[110] = {0};	
	snprintf(report,109,"\r\nsat_id=0x%x",hts_packet.header.sender);
	simple_uart_putstring((uint8_t*)report);
	// debug_putstring((uint8_t*)report);

	if (g_ts_radio_dtm_rx_package_length <= 256) {
		if (g_ts_radio_dtm_cur_num >= g_ts_radio_dtm_rx_package_length) {
			uesb_stop_rx();
		}  else {
			g_ts_radio_dtm_cur_num ++;
		}
	}
}


static void ui_init(void)
{
	uint32_t buttons[] = BUTTONS_LIST;
	uint32_t leds[] = LEDS_LIST;
	uint32_t i;

	for (i = 0; i < BUTTONS_NUMBER; i++)
	{
			nrf_gpio_cfg_input(buttons[i],NRF_GPIO_PIN_PULLUP);
	}
	for (i = 0; i < LEDS_NUMBER; i++)
	{
			nrf_gpio_cfg_output(leds[i]);
	}
	//simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
}

bool control_button_pressed(void) {
	uint32_t buttons[] = BUTTONS_LIST;
	static bool control_button_was_down = false;
	bool control_button_is_down = !nrf_gpio_pin_read(buttons[3]);
	bool press = (!control_button_was_down && control_button_is_down);
	control_button_was_down = control_button_is_down;
	return press;	
}

// We only want to bother communicating with the Freescale if one of us has
// something to say to the other. We can't quite know when the Freescale
// will have something to say, so we use an exponential back-off to approximate.
#define ATOM_US 20
#define MS_TO_TIME_OUT_ESB 80
#define CYCLES_TO_TIME_OUT_ESB (MS_TO_TIME_OUT_ESB * 1000 / ATOM_US)
#define CYCLES_PER_SPI_ROUND 1 /*((DELAY_MS*1000)/ATOM_US)*/

// These were added to debug the time travel issue found on 7 July.
uint32_t last_timestamp_seen = 0;
uint8_t next_jump_idx = 0;
struct{uint32_t before;uint32_t after;} jumps_seen[16] = {{0,0}};

bool last_spi_had_row = false;
uint32_t backoff_atoms = 256;

void reset_spi_backoff(void) {
	backoff_atoms = 1;
}

void spi_received_diag (sat_to_nordic_dtm_t packet) {
	uint16_t recv_crc16, crc16;
	static uint32_t bad_spi_rx = 0, good_spi_rx = 0;
	
	/* Calculate CRC16 */
	recv_crc16 = packet.crc16;
	packet.crc16 = 0;
	crc16 = crc16_compute((uint8_t*)(&packet), sizeof(packet), 0);
	
	if (recv_crc16 != crc16) {
		bad_spi_rx++;
	} else {
		good_spi_rx++;
		
		/* Need to send back to 22F */
		last_received_crc16 = crc16;
		
		if ((packet.cmd_type != dtm_cmd_type) || (packet.dtm_tx_freq != dtm_freq) || 
				(packet.dtm_tx_pkt_len != dtm_pkt_len) || (packet.dtm_pkt_type !=  dtm_pkt_type) ||
				(packet.dtm_power != dtm_power) || (packet.tx_interval != dtm_tx_interval_ms)) {

			dtm_new_cmd = true;					
			dtm_cmd_type = packet.cmd_type;
			dtm_freq = packet.dtm_tx_freq;
			dtm_pkt_len = packet.dtm_tx_pkt_len;		/* Max is UESB_CORE_MAX_PAYLOAD_LENGTH */
			dtm_power = packet.dtm_power;
			dtm_pkt_type = packet.dtm_pkt_type;
			dtm_tx_interval_ms = packet.tx_interval; 
		}
		randomFrom22F = packet.randomFrom22F; 
	}
}
	/* DTM Power
	* 0: +4dBm   DEFAULT
	* 1: 0dBm
	* 2: -4dBm
	* 3: -8dBm
	* 4: -12dBm
	* 5: -16dBm
	* 6: -20dBm
	* 7: -30dBm
	*/
static uesb_tx_power_t convert_dtm_power(uint16_t dtm_power) {
	uesb_tx_power_t tx_power = (uesb_tx_power_t)dtm_power;
	switch (dtm_power) {
		case 0: 
			tx_power = UESB_TX_POWER_4DBM;
			break;
		case 1: 
			tx_power = UESB_TX_POWER_0DBM;
			break;
		case 2: 
			tx_power = UESB_TX_POWER_NEG4DBM;
			break;
		case 3: 
			tx_power = UESB_TX_POWER_NEG8DBM;
			break;
		case 4: 
			tx_power = UESB_TX_POWER_NEG12DBM;
			break;
		case 5: 
			tx_power = UESB_TX_POWER_NEG16DBM;
			break;
		case 6: 
			tx_power = UESB_TX_POWER_NEG20DBM;
			break;
		case 7: 
			tx_power = UESB_TX_POWER_NEG30DBM;
			break;
	}
	
	return tx_power;
}
//static uint32_t spi_tx_cnt = 0;		
int main(void)
{
		nordic_to_sat_dtm_t		spi_tx_data;
		uint8_t 							uesb_tx_data[UESB_CORE_MAX_PAYLOAD_LENGTH];	/* Max UESB packet size */
		bool									prbs_val = false;
		uint8_t								data_repeat;
	
    // Set 16 MHz crystal as our 16 MHz clock source (as opposed to internal RCOSC)
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Wait
    }
		
		//Sometimes after power up, 22F is not ready, let's wait for 1 second.
		nrf_delay_us(100000);

    ui_init();
    
		init_spi();
		
		ts_init_ts_radio_dtm_command();
		initRandom();
		g_ts_radio_dtm_tx_package_length = 10;
		g_ts_radio_dtm_tx_channel_id = 34;
		g_ts_radio_dtm_tx_package_type = 0;
		
		while(1) {
			/* Prepare packet */
			spi_tx_data.last_crc16_recvd = last_received_crc16;
			
			spi_tx_data.crc16 = 0;
			spi_tx_data.randomFrom22F = randomFrom22F;
			spi_tx_data.crc16 = crc16_compute ((uint8_t*) &spi_tx_data, sizeof(spi_tx_data), 0);
	
			spi_send_command_diag(spi_tx_data, spi_received_diag);

			/* 06/18/16 default to Transmit-only for now */
			if (dtm_new_cmd) {
				if (dtm_pkt_type <= DTM_PKT_TYPE_MAX) {
					if (dtm_pkt_type == DTM_PKT_UNMODULATED) {
						g_ts_radio_dtm_modulated = false;
					} else {
						g_ts_radio_dtm_modulated = true;
					}
					g_ts_radio_dtm_tx_package_type = dtm_pkt_type;
				}
				if (dtm_pkt_len <= UESB_CORE_MAX_PAYLOAD_LENGTH) {
					g_ts_radio_dtm_tx_package_length = dtm_pkt_len;
				}
				if (dtm_freq <= DTM_FREQ_MAX) {
					g_ts_radio_dtm_tx_channel_id = dtm_freq;
					g_ts_radio_dtm_rx_channel_id = g_ts_radio_dtm_tx_channel_id;
				}
				g_ts_radio_dtm_power_level = convert_dtm_power(dtm_power);
				
				net_init(process_packet, 0, g_ts_radio_dtm_tx_channel_id, g_ts_radio_dtm_rx_channel_id, g_ts_radio_dtm_power_level,
					g_ts_radio_dtm_modulated);
				
				dtm_new_cmd = false;
			}
			
			/* Fill in packet and send it.
			 * Only the first bytes reflect the packet type.  If the packet length is bigger than that,
			 * then we don't guarantee the content of the rest of the packet.
			 */
			prbs_val = false;
			if (g_ts_radio_dtm_tx_package_type == DTM_PKT_TYPE_PRBS9) {
				for (uint8_t qq = 0; qq < g_ts_radio_dtm_tx_package_length; qq++) {
					uesb_tx_data[qq] = getRandom(1);
				}
				prbs_val = true;
			} else if (g_ts_radio_dtm_tx_package_type == DTM_PKT_TYPE_PRBS15) {
				for (uint8_t qq = 0; qq < g_ts_radio_dtm_tx_package_length; qq++) {
					uesb_tx_data[qq] = getRandom(0);
				}
				prbs_val = true;
			} else if (g_ts_radio_dtm_tx_package_type == DTM_PKT_TYPE_11110000) {
				data_repeat = 0xF0;
			}else if (g_ts_radio_dtm_tx_package_type == DTM_PKT_TYPE_10101010) {
				data_repeat = 0xAA;
			} else { /* DTM_PKT_TYPE_01010101 */
				data_repeat = 0x55;
			}
			
			if (prbs_val == false) {
				for (uint8_t qq = 0; qq < g_ts_radio_dtm_tx_package_length; qq++) {
					uesb_tx_data[qq] = data_repeat;
				}
			}
				
			send_packet_dtm(&uesb_tx_data[0], g_ts_radio_dtm_tx_package_length);
		
			/* We can't put 0 for this function, so delay for 1 us if user enters 0ms */
			if (dtm_tx_interval_ms == 0) {
				nrf_delay_us(1);
			} else {
				nrf_delay_us(GET_DTM_TX_INTERVAL_US(dtm_tx_interval_ms));
			}
		}
		
}
