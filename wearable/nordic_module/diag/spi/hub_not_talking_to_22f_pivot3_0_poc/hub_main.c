/*
 * Copyright © 2015 Turingsense, Inc.
 *
 * hub_main.c
 *
 * Main Nordic code running on the hub.
 * Standalone Nordic.  Will send generated data.
 * Will not talk to satellites.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "app_util.h"
#include "nrf.h"
#include "boards.h"
#include "uesb/micro_esb.h"
#include "uesb/uesb_error_codes.h"
#include "nrf_gpio.h"
#include "simple_uart.h"

#include "hub_main.h"
#include "crc16.h"
#include "crc32.h"
#include "net_common.h"
#include "nrf_delay.h"

#include "spi_slave_comm.h"
#include "hub_timer.h"			/* RTC */

/* Pivot 3.0 POC */
#include "pivot3_0_poc.h"

#if EVAL_BOARD
// Set to 0 for normal operation.
// Set to 1 for output quaternion data for python cube display, don't send data to Hub's K22F
#define OUTPUT_3D_CUBE_ONLY  1

// If this is turned on, the hub will only speak once every 300ms. Makes it easy to
// see packet printouts one by one, but it's obviously too slow for real life.
#define SLOW_DEBUG 0

#define PRINT_DEBUG 1

// Whether the app on the other side of the UART is an eMPL-client.py-like renderer.
#define UART_RENDER	1
#endif /* EVAL_BOARD */

#if NO22F
bool should_set_timestamp = false;

uint8_t overwrite_cur_sensors_per_sat[MAX_SATELLITES] = {1, 1, 1, 1, 1, 1};

#if OUTPUT_3D_CUBE_ONLY
bool start_dumping = true;
#else
bool start_dumping = false;
#endif
#endif /* NO22F */

// Metrics
uint32_t cur_num_of_sat = 2;
uint32_t cur_sat_ids[MAX_SATELLITES] = {313,219};
uint32_t new_lcp = 26;
uint32_t my_id_hub = 32;


//USED FOR COMMAND MANAGEMENT
uint32_t command_to_sat[MAX_SAT_COMMSTRUCT]; //[0] SAT ID to send command - 0 mens ALL SAT; [1] command; [2] value 1; [3] value 2; [4] value 3; 
bool command_should_send_to_sat = false;
static uint8_t command_sendings_left = 0;

/************************** Globals *******************************/
static uint32_t ct_rows_received[MAX_SATELLITES] = {0};
static uint32_t ct_packets_received_at[MAX_SATELLITES] = {0};
uint32_t total_rows_received = 0;
static uint32_t rs_callback_ct = 0;
static uint32_t valid_rs_ct = 0;
static uint32_t last_timestamp_seen_at[MAX_SENSORS] = {0};
//static uint8_t last_row_count_seen_at[MAX_SENSORS] = {0};
static uint32_t ct_out_of_turn = 0;
static uint32_t requests_timed_out[MAX_SATELLITES] = {0};
static uint32_t rowct_from_sensors[MAX_SENSORS] = {0};

static sth_payload_t last_payload[MAX_SATELLITES];

// Current satellite idx to talk to.
// This is the index to cur_sat_ids.
static uint8_t target_sat_idx = 0;

static uint16_t last_received_crc16[MAX_SATELLITES] = {0};

// Variables used for RTC calibration.
static uint32_t timestamp_to_send = INVALID_TIMESTAMP;
static uint32_t local_clock_at_last_sending = 0;
static uint16_t last_rtc_id = 0;				/* The last RTC ID we sent (i.e., now waiting for the ack) */
static bool waiting_rtc_complete = false;
static uint16_t ack_from_sat = 0;

static uint8_t ct_repeats_of_last_satellite = 0;
static uint8_t rows_in_last_packet = 0;
static uint32_t communicationTestCounter[MAX_SENSORS]; //CF: communication analysis 
static uint32_t communicationTestCounter_lastValue[MAX_SENSORS]={0}; //CF: communication analysis

// How often should we check for an ESB response from the satellites?
// And how long should we wait?
#define TIME_PER_CYCLE 400
#define CYCLES_TO_TIMEOUT 20
/// Keeps track of how long it took to receive responses.
/// For instance, if it took three cycles' worth of time, increment [3].
//uint32_t time_to_fill_histogram[CYCLES_TO_TIMEOUT+1] = {0};
/// Number of cycles that have elapsed since execution began.
static int32_t main_loop_cycle_counter = 0,
	max_cycle_to_timeout = CYCLES_TO_TIMEOUT,
	max_time_per_cycle = TIME_PER_CYCLE,
  max_repeats_of_last_satellite = 3;

#if UART_RENDER
/// Sends |data| across UART in a format that eMPL-client.py instances can read.
void debug_putstring(uint8_t* data) {
	// How this works: packets for arbitrary string logging have an 18-byte buffer.
	//
	// Bytes to be sent are copied into |current_buffer|. First from |scraps|,
	// then from |data|.
	// Whenever |current_buffer| fills up, send those 18 bytes and empty it again.
	// When you run out of bytes to process, anything still in |current_buffer|
	// becomes the new |scraps|, to be sent (we hope) next time.
	
	// Leftover data from previous calls to this function.
	static uint8_t scraps[17] = {0};
	static uint8_t len_scraps = 0;
	uint8_t scraps_pos = 0;
	uint8_t data_pos = 0;
	uint8_t current_buffer[18];
	int buffer_pos=0;
	// Get all leftovers in
	while(scraps_pos < len_scraps) {
		current_buffer[buffer_pos] = scraps[scraps_pos];
		scraps_pos++; buffer_pos++;
	}
	len_scraps = 0;
	while(data[data_pos]) {
		current_buffer[buffer_pos] = data[data_pos];
		data_pos++; buffer_pos++;
		if(buffer_pos == 18) {
			simple_uart_put('$');
			simple_uart_put(0x01);
			simple_uart_put(0x00);
			for(buffer_pos=0; buffer_pos < 18; ++buffer_pos)
			{
				simple_uart_put(current_buffer[buffer_pos]);
			}
			simple_uart_put('\r');
			simple_uart_put('\n');
			buffer_pos = 0;
		}
	}
	for(len_scraps = 0; len_scraps < buffer_pos; ++len_scraps) {
		scraps[len_scraps] = current_buffer[len_scraps];
	}
	
}
#else
// No weird rendering protocol. Just echo out the bytes directly.
#define debug_putstring simple_uart_putstring
#endif

void process_packet(sth_packet_t packet);
void nack_warning(void) {
}
void reinit_esb() {
	net_init(process_packet, nack_warning, get_txchan_from_lcp(), get_rxchan_from_lcp());
}

/* 
 * Sat ID is the 32-bit Hardware ID assigned for each satellite hardware.
 * It's sort of like a MAC address, stays with the hardware throughout its
 * life.
 *
 * On the other hand, sat_idx is the index of this satellite that this
 * hub is concerned with.
 *
 * For example:
 * A user might at one recording uses:
*   sat_idx 0: sat ID 0x0
*   sat_idx 1: sat ID 0xbbccbbcc
*   sat_idx 2: sat ID 0x8
*  
*  on another recording, the user switches the satellite so that:
*
*   sat_idx 0: sat ID 0xbbccbbcc
*   sat_idx 1: sat ID 0x0
*   sat_idx 2: sat ID 0x8
*
* Return:		sat_idx - index of the satellite ID
*						0xFFFFFFFF (a.k.a INVALID_SAT_ID) - if not found
*/ 
static uint8_t get_sat_idx(uint32_t sat_ID) {
	uint8_t sat_idx = INVALID_SAT_IDX;
	
	for (uint8_t idx = 0; idx < cur_num_of_sat; idx++) {
		if (cur_sat_ids[idx] == sat_ID) {
			sat_idx = idx;
			break;
		}
	}
	
	return sat_idx;
}

void set_sat_idx(uint32_t satellite_ids[MAX_SAT_COMMSTRUCT]) {
	
	// TODO: the client should not send always six, but the right values  (max 5)
	cur_num_of_sat = satellite_ids[SAT_COMMSTRUCT_VALZ3];
	
		if (satellite_ids[SAT_COMMSTRUCT_VALX1] < cur_num_of_sat) {
			cur_sat_ids[satellite_ids[SAT_COMMSTRUCT_VALX1]] = satellite_ids[SAT_COMMSTRUCT_VALY2];
			cur_sensors_per_sat[satellite_ids[SAT_COMMSTRUCT_VALX1]] = satellite_ids[SAT_COMMSTRUCT_VALT4];
		} else {
			cur_sat_ids[satellite_ids[SAT_COMMSTRUCT_VALX1]] = INVALID_SAT_ID;
		}
		
		for (int cc = cur_num_of_sat; cc < MAX_SATELLITES;cc++)
		{
			cur_sat_ids[cc] = 0;
			cur_sensors_per_sat[cc] = 0;
		}
}

//cf
void resetPacketCounter(){
	uint8_t cnt;
	for (cnt = 0; cnt < MAX_SENSORS; cnt++) {
	 communicationTestCounter[cnt] = 0;
	 communicationTestCounter_lastValue[cnt] = 0;
	}
}

void set_sat_com(uint32_t num_of_param, uint32_t satellite_ids[MAX_SAT_COMMSTRUCT]) {
	uint8_t cnt;
	
	for (cnt = 0; cnt < MAX_SAT_COMMSTRUCT; cnt++) {
		//if (cnt < cur_num_of_par) {
			command_to_sat[cnt] = satellite_ids[cnt];
		//} else {
		//	command_to_sat[cnt] = 0;
		//}
	}
	command_should_send_to_sat=true;
}

/// Outputs a float value in the right byte order for eMPL-client.py.
static void output_pyserial_float(float value) {
	union {
		float f;
		uint8_t b[sizeof(float)];
	} float_to_bytes;
	
	float_to_bytes.f = value;
	simple_uart_put(float_to_bytes.b[0]);
	simple_uart_put(float_to_bytes.b[1]);
	simple_uart_put(float_to_bytes.b[2]);
	simple_uart_put(float_to_bytes.b[3]);
}

/// Send a quaternion packet across UART to eMPL-client.py.
static void output_pyserial_record(float qx, float qy, float qz, float qw, uint8_t sat_id) {
	simple_uart_put('$');
	simple_uart_put(0x02);
	simple_uart_put(sat_id);
	output_pyserial_float(qw);
	output_pyserial_float(qx);
	output_pyserial_float(qy);
	output_pyserial_float(qz);
	simple_uart_put(0x00);
	simple_uart_put(0x00);
	simple_uart_put('\r');
	simple_uart_put('\n');
}

uint8_t next_sat_idx(void) {
	// Summary: if the current satellite is the farthest behind of
	// all satellites, read from it up to 3 times in a row.
	// Other than that, round-robin to the next satellite.
	
	// TODO: a totally missing satellite is considered "farthest
	// behind", and so will take a lot of time away from the ones
	// that actually exist. Maybe add logic to not retry a satellite
	// unless you've heard from it at least once in the past ### ms?
	uint8_t most_out_of_date_sat_idx = 0;
	uint32_t most_out_of_date_timestamp = INVALID_TIMESTAMP;
	for(int i = 0; i < cur_num_of_sensors; ++i)
	{
		if(last_timestamp_seen_at[i] < most_out_of_date_timestamp) {
			most_out_of_date_sat_idx = get_sat_idx_of_sensor_idx(i);
			most_out_of_date_timestamp = last_timestamp_seen_at[i];
		}
	}
	if(most_out_of_date_sat_idx == target_sat_idx && \
		ct_repeats_of_last_satellite < max_repeats_of_last_satellite && rows_in_last_packet > 0) {
		ct_repeats_of_last_satellite++;
		return target_sat_idx;
	}
	target_sat_idx += 1;
	if(target_sat_idx >= cur_num_of_sat) {
		target_sat_idx = 0;
	}
	ct_repeats_of_last_satellite = 0;
	return target_sat_idx;
}

/// Advance to the next satellite, build an up to date control packet, and
/// send it off.
static uint32_t cwati_sat_to_talk[MAX_SATELLITES] = {0};
static void cue_next_satellite(void) {
	uint32_t local_clock;
	uint8_t	 sat_idx;
	
	uint32_t cwati_target_sat_idx;
	cwati_target_sat_idx = next_sat_idx();
	cwati_sat_to_talk[cwati_target_sat_idx]++;
	
	hts_packet_t hts_packet = {0};
	hts_packet.header.sender = my_id_hub;
	
	static uint32_t satid = 0;
	static uint32_t command = 0;
	static uint32_t valx1 = 0;
	static uint32_t valy2 = 0;
	static uint32_t valz3 = 0;
	static uint32_t valt4 = 0;

	if((command_to_sat[SAT_COMMSTRUCT_COMMA] !=0) && (command_sendings_left == 0))
	{
			satid = command_to_sat[SAT_COMMSTRUCT_SATID];
			command = command_to_sat[SAT_COMMSTRUCT_COMMA];
			valx1 = command_to_sat[SAT_COMMSTRUCT_VALX1];
			valy2 = command_to_sat[SAT_COMMSTRUCT_VALY2];
			valz3 = command_to_sat[SAT_COMMSTRUCT_VALZ3];
			valt4 = command_to_sat[SAT_COMMSTRUCT_VALT4];

			command_sendings_left = COM_SENDING_CNT;
#if NO22F
		  command_to_sat[SAT_COMMSTRUCT_COMMA] = 0;
#endif
	}

	if(command_sendings_left) {
		command_sendings_left--;

		if(command == SAT_COMM_WIF_SETRTC) {			
			local_clock = hub_timer_get_elapsed_ms();
			timestamp_to_send += (local_clock - local_clock_at_last_sending);
			valx1 = timestamp_to_send;
			local_clock_at_last_sending = local_clock;
			valy2 = last_rtc_id;
		}else if(((command == SAT_COMM_WIF_SETLCP)||(command == SAT_COMM_SETBTCOMCH)) && (command_sendings_left==0))
		{
			//Finished notifying all satellites.  It's time to change channel.
				if (set_lcp(new_lcp)) {
					reinit_esb();
				}
		}else if(command == SAT_COMM_SETNODTRPR)
		{
			
			if(valx1 > 0) max_repeats_of_last_satellite = valx1;
			if(valy2 > 0) max_cycle_to_timeout = valy2;
			if(valz3 > 0) max_time_per_cycle = valz3;
		}
	}else{
	
			satid = 0;
			command = 0;
			valx1 = 0;
			valy2 = 0;
			valz3 = 0;	
			valt4 = 0;	
	}
	
	hts_packet.hub_to_satellite_payload.command_toSat[SAT_COMMSTRUCT_SATID] = satid;
	/* Only forward command to sat if we need to */
	if (!command_is_for_hub_only(command)) {
		hts_packet.hub_to_satellite_payload.command_toSat[SAT_COMMSTRUCT_COMMA] = command;
	}
	hts_packet.hub_to_satellite_payload.command_toSat[SAT_COMMSTRUCT_VALX1] = valx1;
	hts_packet.hub_to_satellite_payload.command_toSat[SAT_COMMSTRUCT_VALY2] = valy2;
	hts_packet.hub_to_satellite_payload.command_toSat[SAT_COMMSTRUCT_VALZ3] = valz3;
	hts_packet.hub_to_satellite_payload.command_toSat[SAT_COMMSTRUCT_VALT4] = valt4;
	hts_packet.hub_to_satellite_payload.recipient = cur_sat_ids[target_sat_idx];
	for (sat_idx = 0; sat_idx < MAX_SATELLITES; sat_idx++) {
		hts_packet.hub_to_satellite_payload.last_seen_crc16[sat_idx] = last_received_crc16[sat_idx];
	}
	hts_packet.hub_to_satellite_payload.command_crc32 = 0;
	uint32_t payload_crc32 = crc32(0xFFFFFFFF, &hts_packet.hub_to_satellite_payload,
															 sizeof(hts_packet.hub_to_satellite_payload));
	hts_packet.hub_to_satellite_payload.command_crc32 = payload_crc32;

#if PRINT_DEBUG
#if SLOW_DEBUG
	char msg[200] = {0};
	snprintf(msg,199,"\r\nSending %x to #%d (sending CRC %04.04x) (acking CRC %04.04x) ",
		hts_packet.hub_to_satellite_payload.command,
		hts_packet.hub_to_satellite_payload.recipient,
		hts_packet.hub_to_satellite_payload.command_crc32,
		hts_packet.hub_to_satellite_payload.last_seen_crc16
	);
	debug_putstring(msg);
#endif
#endif
	
	if(!send_packet(hts_packet)) {
		// Something went wrong in the network library. Recover as best you can.
		reinit_esb();
	}
}

#if EVAL_BOARD
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

/// The first time this is called during Button 4 being pressed down, it'll return true.
bool control_button_pressed(uint8_t num) {
	uint8_t button_index = num - 1; //for example button 4 would yield index 3
	uint32_t buttons[] = BUTTONS_LIST;
	static bool control_button_was_down[4] = {false, false, false, false};
	bool control_button_is_down = !nrf_gpio_pin_read(buttons[button_index]);
	bool press = (!control_button_was_down[button_index] && control_button_is_down);
	control_button_was_down[button_index] = control_button_is_down;
	return press;	
}
#endif /* EVAL_BOARD */

// Most recent control packet from the hub.
volatile bool packet_waiting = false;
sth_packet_t packet_to_process;

// Our incoming packet handler.
void process_packet(sth_packet_t sth_packet) {
	// No significant work done here; just record the packet and signal
	// to the main loop that there's something to read. This method is
	// called from an interrupt handler, so having it do significant
	// work is a drain on the firmware's sanity.
	if(!packet_waiting) {
		packet_to_process = sth_packet;
		packet_waiting = true;
	}
}

// Quaternion printouts are delayed for two reasons (I forget which
// one is more important):
// 1) The hub is very busy conducting the satellites.
// 2) There might be an interrupt conflict between the UART and other stuff.
static bool frame_to_print = false;
static esb_sensor_record_t frame_record = {0};

void print_frame_if_ready(uint8_t frame_sender) {
	if(!frame_to_print) return;
	frame_to_print = false;

	output_pyserial_record(frame_record.quat_x,
												 frame_record.quat_y,
												 frame_record.quat_z,
												 frame_record.quat_w,
												 frame_sender);

}

/// How many bits of |n| are different from the bits in |m|?
int hamming_distance_16(uint16_t n, uint16_t m) {
	uint16_t difference = n^m;
	int distance = 0;
	while(difference) {
		distance += difference&1;
		difference >>= 1;
	}
	return distance;
}

/// Assume |payload| is from a known good packet which came from |satellite_id|.
/// Deserializes the payload data and pushes it to the queue.
void process_good_packet(uint32_t satellite_id, sth_payload_t *payload) {
	uint8_t sat_idx;
	uint8_t sensor_idx;
#if NO22F
	char report[100];
#endif
	sat_idx = get_sat_idx(satellite_id);
	if(sat_idx == INVALID_SAT_IDX) {
		return;
	}

	if (waiting_rtc_complete) {
		if (payload->header.rtc_id_ack == last_rtc_id) { 
			ack_from_sat |= (1 << sat_idx);

			if (ack_from_sat == ((1 << cur_num_of_sat) - 1)) {
				waiting_rtc_complete = false;
#if NO22F
				snprintf(report,99,"\r\nRTC ACK Complete!\n");
				debug_putstring((uint8_t*)report);
#endif
			}
		}
	}	

	// There are four (four!) ways to tell that a packet has ended.
	// 1) The header timestamp is INVALID. This packet has no rows at all!
	// 2) You encounter a row with all zeroes for its quaternion data. That's
	//    not a possible orientation (it's a vector with no direction) so it's
	//    used as a terminator.
	// 3) Youve read header.row_ct rows. Anything beyond that is past the end of
	//    the physical packet.
	// 4) You've read SENSOR_RECORDS_PER_NET_PACKET rows. Anything beyond that is
	//    past the end of the struct (and the maximum value of header.row_ct).
	if(payload->header.timestamp != INVALID_TIMESTAMP)
	{
		uint32_t running_timestamp = payload->header.timestamp;
		rows_in_last_packet = payload->header.row_ct;

		for(int i=0;i<get_sensor_num_of_sat_idx(sat_idx);++i) {
			sensor_idx = get_starting_sensor_idx_of_sat_idx(sat_idx) + i;
			if(i >= payload->header.row_ct) {
				break;
			}
			rowct_from_sensors[sensor_idx]++;
			
			/// Translate the compact RF packet back into the standard SPI format.
			sat_sensor_record_t outbound_datum;
			outbound_datum.timestamp = running_timestamp;
			outbound_datum.sensor_idx = sensor_idx;
			last_timestamp_seen_at[sensor_idx] = running_timestamp;
			outbound_datum.data.accel_x = payload->sensor_records[i].accel_x;
			outbound_datum.data.accel_y = payload->sensor_records[i].accel_y;
			outbound_datum.data.accel_z = payload->sensor_records[i].accel_z;
			outbound_datum.data.gyro_x = payload->sensor_records[i].gyro_x;
			outbound_datum.data.gyro_y = payload->sensor_records[i].gyro_y;
			outbound_datum.data.gyro_z = payload->sensor_records[i].gyro_z;
#if ENABLE_MAG
			outbound_datum.data.mag_x = payload->sensor_records[i].mag_x;
			outbound_datum.data.mag_y = payload->sensor_records[i].mag_y;
			outbound_datum.data.mag_z = payload->sensor_records[i].mag_z;
#endif
			outbound_datum.data.quat_w = payload->sensor_records[i].quat_w;
			outbound_datum.data.quat_x = payload->sensor_records[i].quat_x;
			outbound_datum.data.quat_y = payload->sensor_records[i].quat_y;
			outbound_datum.data.quat_z = payload->sensor_records[i].quat_z;
#if OUTPUT_3D_CUBE_ONLY
			// Just display the quaternion output, don't push to cbuf as 
			// there is no SPI read from Hub's K22F to pop cbuf
#else
			/* If cbuf is full, discard old data */
			if(cbufIsFull(&outbound_queue)) {
				cbufPopDiscard(&outbound_queue);
			}
			
			if(cbufPush(&outbound_queue,outbound_datum) == E_OK) {
				rows_pushed++;
			}
#endif
#if UART_RENDER
#if OUTPUT_3D_CUBE_ONLY
			if((rowct_from_sensors[sensor_idx] % 1) == 0)	{
#else
			if((rowct_from_sensors[sensor_idx] % 100) == 0)	{
#endif
				frame_to_print = true;
				frame_record = payload->sensor_records[i];
			}
			print_frame_if_ready(sensor_idx);
#endif
			
			/* This part to increase running_timestamp is actually only needed for satellite
			 * who has only 1 sensor */
			// Increment the timestamp so that the next row's unpacked timestamp will be correct.
			if((i+1 < SENSOR_RECORDS_PER_NET_PACKET) && (i+1 < payload->header.row_ct)) {
				// TODO: find a better place for this increment to go in the loop so you don't need
				// the messy IF statement above.
				running_timestamp += payload->sensor_records[i+1].ts_diff;
			}
		}
	}

}

/// "Real" is relative. This ensures that the packet is a satellite packet, from a
/// satellite that's ours,  and that its timestamp is trustworthy. Other data 
/// integrity not guaranteed.
bool waiting_rx_packet_valid(void) {
	rs_callback_ct++;
	
	// Only pay attention to satellites we know about.
	// TODO: this should instead use the list of registered satellite IDs from
	// the Freescale.
	if(get_sat_idx(packet_to_process.header.sender) == INVALID_SAT_IDX) {
		return false;
	}
	
	// We always expect to get a reply back from the same satellite that we
	// contacted. If not, it's a sign that something is very delayed on the
	// satellite - it's taking more than 5ms to respond to our request! Make
	// note of that for diagnostic purposes.
	if(packet_to_process.header.sender != cur_sat_ids[target_sat_idx]) {
		ct_out_of_turn ++;
	}
	
	sth_payload_t payload = packet_to_process.satellite_to_hub_payload;
	uint32_t payload_timestamp = payload.header.timestamp;
	uint16_t timestamp_crc = crc16_compute((uint8_t*)(&payload_timestamp),sizeof(payload_timestamp),0);
	
	return payload.header.timestamp_crc == timestamp_crc;
}

/// Checks whether timestamp and CRC have moved on between two StH payloads. Assumes
/// that the timestamps are not bogus values.
bool satellite_has_advanced_between_packets(sth_payload_t *old_payload, sth_payload_t *new_payload) {	
	uint32_t payload_timestamp = new_payload->header.timestamp;
	
	uint16_t new_crc16 = crc16_compute((uint8_t*)(new_payload),sizeof(sth_payload_t),0);
	uint16_t old_crc16 = crc16_compute((uint8_t*)(old_payload),sizeof(sth_payload_t),0);
	bool crc_changed = new_crc16 != old_crc16;
	
	uint32_t old_timestamp = old_payload->header.timestamp;
	bool timestamp_changed = old_timestamp != payload_timestamp;
	bool packet_empty = payload_timestamp == INVALID_TIMESTAMP;
	
	return (crc_changed && timestamp_changed) || packet_empty;
}

static uint32_t cwati_sat_hasnt_advanced[MAX_SATELLITES] = {0};
	
/// Checks |packet| against previous packets to see if it's a retransmit or not. Parses and
/// queues any waiting rows that are now known to be trustworthy. |packet| must be valid (good
/// sender, trustworthy timestamp), but is allowed to be corrupted other than that.
void process_response() {
	uint8_t sat_idx;

	valid_rs_ct++;
	sth_payload_t payload = packet_to_process.satellite_to_hub_payload;
	
	sat_idx = get_sat_idx(packet_to_process.header.sender);
	if (sat_idx == INVALID_SAT_IDX) {
		return;
	}
	
	if(satellite_has_advanced_between_packets(
			&last_payload[sat_idx],
			&payload)) {
		ct_rows_received[sat_idx]++;
		total_rows_received++;

		//CF 01/06/2016 - COMMUNICATION PROTOCOL IMPROVEMENT
	  for(int i=0;i<SENSOR_RECORDS_PER_NET_PACKET;++i) {
			int16_t tmp=(last_payload[sat_idx].sensor_records[i].quat_w)/CPI_QUATERNION16_BIAS_MULTIPLIER;
			if(tmp==(int16_t)CPI_QUATERNION16_COUNTER_COMMAND){
				if(communicationTestCounter_lastValue[sat_idx]<last_payload[sat_idx].sensor_records[i].accel_x) {
					communicationTestCounter[sat_idx]++;
					communicationTestCounter_lastValue[sat_idx]=last_payload[sat_idx].sensor_records[i].accel_x;
				}else if(communicationTestCounter_lastValue[sat_idx]>last_payload[sat_idx].sensor_records[i].accel_x){
          communicationTestCounter_lastValue[sat_idx] = last_payload[sat_idx].sensor_records[i].accel_x;
        }else if(last_payload[sat_idx].sensor_records[i].accel_x <= 0){
          communicationTestCounter[sat_idx] = 0;
          communicationTestCounter_lastValue[sat_idx]=0;
        }
					last_payload[sat_idx].sensor_records[i].gyro_y = (int16_t)(communicationTestCounter[sat_idx]%CPI_QUATERNION16_COUNTER_MAXVALU);
					last_payload[sat_idx].sensor_records[i].gyro_z = 0;
				
			}else if(tmp==(int16_t)CPI_QUATERNION16_VERSION_COMMAND){
					last_payload[sat_idx].sensor_records[i].gyro_z = (int16_t)(REVIS_VERSION_HUBNRD);
			}
		}
				
		process_good_packet(packet_to_process.header.sender, &last_payload[sat_idx]);
	} else {
		cwati_sat_hasnt_advanced[sat_idx]++;
	}
	last_payload[sat_idx] = payload;
	last_received_crc16[sat_idx] = crc16_compute((uint8_t*)(&payload),sizeof(sth_payload_t),0);
}

void hub_timer_self_test(void) {	
	char report[110] = {0};	
	hub_timer_start();
	uint32_t before = hub_timer_get_elapsed_ms();
	nrf_delay_us(100000);
	uint32_t after = hub_timer_get_elapsed_ms();
	snprintf(report,109,"\r\nelapsed timer=0x%x",before);
	debug_putstring((uint8_t*)report);
	snprintf(report,109,"\r\nelapsed timer=0x%x",after);
	debug_putstring((uint8_t*)report);
	hub_timer_stop();
}

void talk_to_satellites(void) {
	int32_t timeout_position = (main_loop_cycle_counter + max_cycle_to_timeout);
	uint8_t	sat_idx;
	
	// TODO: the timeout loop here is probably way more complex than it needs to be.
	// May even consider making it a tight loop and using the lowfreq RTC to specify
	// a timeout, instead of using a counter; cwajh only used a counter because he
	// didn't know how to access the RTC.

	// Communicate with a satellite.
	cue_next_satellite();
	
	uint32_t time_to_fill = 0;
	bool received_real_packet;

	
	for (sat_idx = 0; sat_idx < cur_num_of_sat; sat_idx++) {
		received_real_packet = false;
		
		while(!received_real_packet && main_loop_cycle_counter != timeout_position) {
			main_loop_cycle_counter++;
			time_to_fill++;
			nrf_delay_us(max_time_per_cycle);
			if(packet_waiting) {
				if(waiting_rx_packet_valid()) {
					// The wait is over.
					received_real_packet = true;
				} else {
					// If the net layer gave us a packet but it's not a real packet, keep listening.
					packet_waiting = false;
				}
			}
			
		}
		
		if(received_real_packet) {
			ct_packets_received_at[target_sat_idx]++;
			process_response();
			packet_waiting = false;
		} else {
			requests_timed_out[target_sat_idx]++;
		}
	}
}

int main(void)
{
#if EVAL_BOARD
		static bool send_sat_id = true;
		static uint8_t send_sat_id_cur_idx = 0;
#endif
#if PRINT_DEBUG
		char report[110] = {0};
#endif 
		
		if ((sizeof(padded_nordic_to_hub_t) > (0xFF + 1)) || (sizeof(hub_to_nordic_t) > (0xFF + 1))) {
			printf("ERROR: spi size too big!\n");
			while(1) {
				;// Stop here.  Must fix this.
			}
		}

    // Initialize the 16MHz clock used by SPI and ESB.
		// Supposedly uses a crystal instead of the internal RCOSC
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
       // Wait
    }
		
		spi_slave_example_init();
		reinit_esb();
#if EVAL_BOARD
		ui_init();
#endif /* EVAL_BOARD */
		
		// Initialize the slow clock used for recording timestamps.
		hub_timer_start_clock();
		int ret = hub_timer_init();
#if PRINT_DEBUG
		snprintf(report,109, "\r\nhub_timer_init ret %d\n", ret);
		debug_putstring((uint8_t*)report);

		//test rtc timer
		hub_timer_self_test();
#endif	
    for(int i = 0; i < cur_num_of_sat; ++i) {
			last_payload[i].header.timestamp = INVALID_TIMESTAMP;
			last_payload[i].sensor_records[0].quat_w = 0;
			last_payload[i].sensor_records[0].quat_x = 0;
			last_payload[i].sensor_records[0].quat_y = 0;
			last_payload[i].sensor_records[0].quat_z = 0;
		}
    // Start listening for incoming transmissions
    uesb_start_rx();

#if NO22F
		set_lcp(new_lcp);
		cur_num_of_sensors = 0;
		
		for (uint8_t qq = 0; qq < cur_num_of_sat; qq++) {
			cur_sensors_per_sat[qq] = overwrite_cur_sensors_per_sat[qq];
			cur_num_of_sensors += get_sensor_num_of_sat_idx(qq);
		}
		
#endif
		
    while (true)
    {   
			// When a SET_RTC is sent out, every satellite hopefully receives it.
			// But some may not. As insurance, we send out the RTC multiple times,
			// advancing the value each time to match the previous ones.
			if(should_set_timestamp) {
#if PRINT_DEBUG
				char rtc_string[20] = {0};
				debug_putstring((uint8_t*)"\r\nSetting RTC to ");
				snprintf(rtc_string,20,"%u",current_timestamp);
				debug_putstring((uint8_t*)rtc_string);
#endif //PRINT_DEBUG

				hub_timer_start();
				timestamp_to_send = current_timestamp;
				local_clock_at_last_sending = hub_timer_get_elapsed_ms();
				//ct_rtc_sendings_left = RTC_SENDING_CNT;
				should_set_timestamp = false;
				last_rtc_id++;
				waiting_rtc_complete = true;
				ack_from_sat = 0;
#if NO22F
				command_to_sat[SAT_COMMSTRUCT_COMMA] = SAT_COMM_WIF_SETRTC;
#endif
			}
			talk_to_satellites();

#if SLOW_DEBUG
			// Rate control so that in slow debug mode, the hub is only reading a few packets per second.
			nrf_delay_ms(300);
#endif
			
#if EVAL_BOARD
			if (control_button_pressed(3)) {
				command_to_sat[SAT_COMMSTRUCT_COMMA] = SAT_COMM_STOP_DUMPI;
				start_dumping = false;

				/* Empty cbuf FAST_DRAIN */
				cbufInit (&outbound_queue);
			}
			if (control_button_pressed(2)) {
				command_to_sat[SAT_COMMSTRUCT_COMMA] = SAT_COMM_START_DUMP;
				start_dumping = true;
			}
			if (control_button_pressed(1)) {
//				send_sat_id = true;
//				send_sat_id_cur_idx = 0;
				should_set_timestamp = true;
				/* Set RTC from hub 22F means to reset it to 0 */
				current_timestamp = 0;
				/* Empty cbuf */
				cbufInit (&outbound_queue);
				
				for (uint8_t qq = 0; qq < MAX_SENSORS; qq++) {
					last_timestamp_seen_at[qq] = 0;
				}
#if PRINT_DEBUG
				debug_putstring("Set RTC to 0");
#endif /* PRINT_DEBUG */
			}
			
			if (send_sat_id == true) {
				command_to_sat[SAT_COMMSTRUCT_COMMA] = SAT_COMM_WIF_SETSAT;
				command_to_sat[SAT_COMMSTRUCT_VALX1] = send_sat_id_cur_idx;
				command_to_sat[SAT_COMMSTRUCT_VALY2] = cur_sat_ids[send_sat_id_cur_idx];
				command_to_sat[SAT_COMMSTRUCT_VALZ3] = cur_num_of_sat;
				command_to_sat[SAT_COMMSTRUCT_VALT4] = cur_sensors_per_sat[send_sat_id_cur_idx];

				send_sat_id_cur_idx++;
				if (send_sat_id_cur_idx == cur_num_of_sat) {
					send_sat_id = false;
				}
			}
#endif /* EVAL_BOARD */

#if PRINT_DEBUG
			static uint32_t time_start = 0, time_diff, time_now;
			static uint32_t pkt_start[MAX_SENSORS] = {0}, pkt_diff[MAX_SENSORS], pkt_now[MAX_SENSORS];
			static uint32_t last_valid_rs_ct = 0, last_diag_rx_ct = 0;
			
			static bool first_time = true;
			bool print_now = false, control_button_4_pressed = false;
			uint32_t time_print_now;
			static uint32_t last_time_print;

			print_now = false;
			
			control_button_4_pressed = control_button_pressed(4);
			
			if (first_time) {
				last_time_print = hub_timer_get_elapsed_ms();
				first_time = false;
			} else {
				time_print_now = hub_timer_get_elapsed_ms();
				
				if (time_print_now == 0) {
					/* Timer has expired */
					hub_timer_stop();
					hub_timer_start();
					last_time_print = 0;
				} else {
					if (control_button_4_pressed) {
						print_now = true;
						last_time_print = hub_timer_get_elapsed_ms();
					} else{
//						time_print_now = hub_timer_get_elapsed_ms();
						if (time_print_now - last_time_print > 3000) {
							/* Print every 3s */
							print_now = true;
							last_time_print = time_print_now;
						}
					}
				}
			}
			
			if(print_now) {
					
				time_now = hub_timer_get_elapsed_ms();
				time_diff = time_now - time_start;
				
				if (start_dumping) {
					debug_putstring("\r\nStart Dumping");
				} else {
					debug_putstring("\r\nStop Dumping");
				}
				for (uint8_t qq = 0; qq < cur_num_of_sensors; qq++) {
					pkt_now[qq] = rowct_from_sensors[qq];
					pkt_diff[qq] = pkt_now[qq] - pkt_start[qq];

					snprintf(report,109,"\r\nPacket rate[%d]: %u pkts/ %u ms = %f pkts/s Last ts[%i]", qq, pkt_diff[qq], time_diff,
							(float) ((float)pkt_diff[qq] / ((float)(time_diff) / 1000.0)), last_timestamp_seen_at[qq]);
					debug_putstring((uint8_t*)report);
					pkt_start[qq] = pkt_now[qq];
				}
				time_start = time_now;


				snprintf(report,109,"\r\nTX attempt:	%d", diag_tx_attempt_ct);
				debug_putstring((uint8_t*)report);
				
				snprintf(report,109,"\r\nRX interrupt:	%d...RX valid: %d", diag_rx_ct, valid_rs_ct);
				debug_putstring((uint8_t*)report);
				snprintf(report,109,"\r\nRX after TX acked:%d...RX after TX not acked: %d", 
					diag_rx_after_ack_ct, diag_rx_after_nack_ct);
				debug_putstring((uint8_t*)report);	

				
				snprintf(report,109,"\r\nRX valid this period (num of valid %d / num of RX interrupt %d): %f",
						valid_rs_ct - last_valid_rs_ct, diag_rx_ct - last_diag_rx_ct,
						(diag_rx_ct - last_diag_rx_ct) ? 
						((float)(((float)(valid_rs_ct - last_valid_rs_ct) * 100) / (float)(diag_rx_ct - last_diag_rx_ct))) : 
						0);
				last_valid_rs_ct = valid_rs_ct;
				last_diag_rx_ct = diag_rx_ct;
				debug_putstring((uint8_t*)report);	

				snprintf(report,109,"\r\nLCP: %u TX chan: %u RX chan: %u", new_lcp, get_txchan_from_lcp(), get_rxchan_from_lcp());
				debug_putstring((uint8_t*)report);
				
				snprintf(report,109,"\r\nSat ID: [0]=0x%x [1]=0x%x [2]=0x%x [3]=0x%x [4]=0x%x CUR NUM OF SAT: %d",
					cur_sat_ids[0], cur_sat_ids[1], cur_sat_ids[2], cur_sat_ids[3], cur_sat_ids[4], cur_num_of_sat);
				debug_putstring((uint8_t*)report);
						
				debug_putstring("\n");
				
				// Try to prevent this message being printed many times in a row, even if the counter
				// doesn't advance.
				main_loop_cycle_counter++;				
			}
#endif
    }
}





