/*
 * Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * satellite_main.c
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
int32_t g_ts_radio_dtm_power_level = 0;
uint8_t g_ts_radio_dtm_tx_packet[256];
uint8_t g_ts_radio_dtm_rx_packet[256];

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
bool sensing_enabled = true;
bool sending_enabled = true;
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
	net_init(satellite_rx_handler, 0, get_txchan_from_lcp(), get_rxchan_from_lcp(), UESB_TX_POWER_4DBM);
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
		if (payload.rtc_value == 0) {
			cbufInit(&data_queue);
		}
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
	//cwati	return;
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
		
	// No significant work done here; just record the packet and signal
	// to the main loop that there's something to read. This method is
	// called from an interrupt handler, so having it do significant
	// work is a drain on the firmware's sanity.
	if(!packet_waiting) {
		// packet_to_process = sth_packet;
		packet_waiting = true;
	}
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
	simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
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
void spi_received(sat_spi_data_packet_t packet) {
	static bool first_time = true;
	sat_cbuf_packet_t cbuf_packet;
	uint16_t crc16, recv_crc16;
		
	/* Calculate CRC16 */
	recv_crc16 = packet.crc16;
	packet.crc16 = 0;
	crc16 = crc16_compute((uint8_t*)(&packet), sizeof(packet), 0);
		
	if (recv_crc16 != crc16) {
		last_spi_had_row = false;
		return;
	}
	
	/* Initializing satellite ID. */
	if (first_time) {
		sat_id = packet.sat_id;
		first_time = false;
	}
	
	if(packet.timestamp == INVALID_TIMESTAMP) {
		last_spi_had_row = false;
		return;
	}

	last_spi_had_row = true;

	/* Discard old data */
	if(cbufIsFull(&data_queue)) {
		cbufPopDiscard(&data_queue);
	}
	
	/* The SPI packet received contains satellite ID,
	 * but we won't store it into our cbuf */
	cbuf_packet.data = packet.data;
	cbuf_packet.timestamp = packet.timestamp;
	
	cbufPush(&data_queue,cbuf_packet);

	backoff_atoms = 1;
}

void reset_spi_backoff(void) {
	backoff_atoms = 1;
}

/// Decide what command to send the Freescale based on our own state.
sat_control_packet_t command_for_spi(void) {
	sat_control_packet_t command = {.sat_command[SAT_COMMSTRUCT_SATID] = 0, .sat_command[SAT_COMMSTRUCT_VALX1] = INVALID_TIMESTAMP};
//	bool sensor_should_be_quiet = CBUF_IS_REDZONE(&data_queue);
//	
//	if (sensor_should_be_quiet || (sending_enabled == false)) {
//		command.command |= SAT_SPI_WAIT;
//	} 

	if (sending_enabled == false) {
		command.sat_command[SAT_COMMSTRUCT_SATID] |= SAT_SPI_WAIT;
	} 
	if (should_set_timestamp) {
		command.sat_command[SAT_COMMSTRUCT_SATID] |= SAT_SPI_SET_RTC;
		command.sat_command[SAT_COMMSTRUCT_VALX1] = timestamp_to_set;
		should_set_timestamp = false;
	}
 
	//cf
	if (must_calibrate) {
		command.sat_command[SAT_COMMSTRUCT_SATID] |= (SAT_SPI_SET_COM);
		command.sat_command[SAT_COMMSTRUCT_COMMA] = command_to_sat[SAT_COMMSTRUCT_COMMA];
		command.sat_command[SAT_COMMSTRUCT_VALX1] = command_to_sat[SAT_COMMSTRUCT_VALX1];
		command.sat_command[SAT_COMMSTRUCT_VALY2] = command_to_sat[SAT_COMMSTRUCT_VALY2];
		command.sat_command[SAT_COMMSTRUCT_VALZ3] = command_to_sat[SAT_COMMSTRUCT_VALZ3];

		must_calibrate=false;
	}
	
	//cf 23/04/2016
	/*
	if (must_set_lcp) {
		command.sat_command[SAT_COMMSTRUCT_SATID] |= (SAT_SPI_SET_LCP);
		command.sat_command[SAT_COMMSTRUCT_COMMA] = command_to_sat[SAT_COMMSTRUCT_COMMA];
		command.sat_command[SAT_COMMSTRUCT_VALX1] = command_to_sat[SAT_COMMSTRUCT_VALX1];
		command.sat_command[SAT_COMMSTRUCT_VALY2] = command_to_sat[SAT_COMMSTRUCT_VALY2];
		command.sat_command[SAT_COMMSTRUCT_VALZ3] = command_to_sat[SAT_COMMSTRUCT_VALZ3];
		
		must_set_lcp=false;
	}*/
	
	
	if (sensing_enabled) {
		command.sat_command[SAT_COMMSTRUCT_SATID] |= SAT_SPI_START;
	} 
	
	return command;
}

/// Blocks until communication is done. Returns whether communication was successful.
/// Only one of the SPI master library and the ESB library can be active at once.
/// If you're using ESB, set rf_active, so that it can be turned off during the
/// communication. If rf_active=true and communication was successful, when this
/// returns RF will be off. Otherwise, it should be unchanged from its original state.
bool do_one_spi_communication(bool rf_active) {
	sat_control_packet_t command = command_for_spi();
//	if(CBUF_IS_REDZONE(&data_queue)) {
//		redzone_mode = true;
//	}

	if(rf_active) {
		if(uesb_stop_rx() != UESB_SUCCESS) {
			return false;
		}
		if(uesb_disable() != UESB_SUCCESS) {
			uesb_start_rx();
			return false;
		}
	}
	bool was_able_to_communicate = spi_send_command(command,spi_received);
	if(!was_able_to_communicate) {
		if(rf_active) {
			reinit_esb();
			uesb_start_rx();
		}
		return false;
	}

	return true;
}

/// Try to communicate enough to read row_ct rows. Returns the number
/// of rows received. This function assumes that RF is turned on, and
/// does the right thing to handle that: turns it off before communicating,
/// and turns it back on before it returns.
int communicate_over_spi(int row_ct) {
	int rows_received = 0;
	if(do_one_spi_communication(true)) {
		bool communicated = true;
		rows_received=last_spi_had_row?1:0;
		while(last_spi_had_row && rows_received < row_ct && communicated) {
			rows_received++;
			communicated = do_one_spi_communication(false);
		}
			reinit_esb();
	}
	return rows_received;
}

int main(void)
{
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
		cbufInit(&data_queue);
		
    uint32_t counter=0;
		uint32_t max_counter = (8000 / ATOM_US);  /* 10 ms */
			
		reinit_esb();
    uesb_start_rx();
		
		
		uint32_t cycles_since_rx = 0;
		
		int8_t inputFromCOMPort[GARY_MAX_COM_PORT_IO_ARRAY_SIZE];
		ts_init_ts_radio_dtm_command();
		initRandom();
		g_ts_radio_dtm_tx_package_length = 10;
		g_ts_radio_dtm_tx_channel_id = 34;
		g_ts_radio_dtm_tx_package_type = 0;
		
    while (true)
    {   
			uint8_t cur_rec_num = get_ts_radio_dtm_command(inputFromCOMPort);
			if (cur_rec_num == 0) {
				continue;
			}
			
			int cmdIndex = get_ts_radio_dtm_command_index(inputFromCOMPort);
			simple_uart_putstring("\r\n");
			print_ts_radio_dtm_cmd(cmdIndex);
			nrf_delay_ms(1000);
			
			if (cmdIndex < 0) {
				continue;
			}
			// int cmdIndex = 2;
			switch (cmdIndex) {
				case 1:
					net_init(process_packet, 0, 0, g_ts_radio_dtm_rx_channel_id, UESB_TX_POWER_4DBM);
					uesb_start_rx();
					g_ts_radio_dtm_cur_num = 0;
					break;
				case 2:
					
					net_init(process_packet, 0, g_ts_radio_dtm_tx_channel_id, 0, g_ts_radio_dtm_power_level);
										
					nrf_delay_us(ATOM_US);
					sth_packet_t sth_packet;
					
					g_ts_radio_dtm_cur_num = 0;
					//int data = 0;
					bool infinite_num = (g_ts_radio_dtm_tx_package_length > 256);
					while (g_ts_radio_dtm_cur_num < g_ts_radio_dtm_tx_package_length) {
						if (g_ts_radio_dtm_tx_package_type == 0) {
							sth_packet.header.sender = getRandom(1);
						} else if (g_ts_radio_dtm_tx_package_type == 1) {
							sth_packet.header.sender = getRandom(0);
						} else if (g_ts_radio_dtm_tx_package_type == 2) {
							sth_packet.header.sender = 0xF0;
						} else {
							sth_packet.header.sender = 0x55;
						}
						send_packet(sth_packet);
						nrf_delay_ms(1000);
						if (!infinite_num) {
							g_ts_radio_dtm_cur_num ++;
						}
					}
					break;
				case 3:
					break;
				default:
					break;
			}
    }
}
