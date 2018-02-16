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
#include "rfcbuf.h"
#include "spi_slave_comm.h"
#include "hub_timer.h"			/* RTC */

/************************** Inits *******************************/

// Metrics
uint32_t cur_num_of_sat = 8;
uint32_t cur_sat_ids[MAX_SENSORS] = {313,330,327,333,336,0xc14,214,219};
static uint32_t cwati_cbuf_full = 0;
uint32_t new_lcp;

// TODO: in a world where multiple hubs might exist, an additional
// "hub id" or "hub serial number" might be needed. But it should be a
// separate variable.
/// Whenever the system uses a device ID, satellites start at 1.
/// The hub is 0 by convention.
#define DEVICE_ID 				0xabcd1

#if EVAL_BOARD
// Set to 0 for normal operation.
// Set to 1 for output quaternion data for python cube display, don't send data to Hub's K22F
#define OUTPUT_3D_CUBE_ONLY  0

// If this is turned on, the hub will only speak once every 300ms. Makes it easy to
// see packet printouts one by one, but it's obviously too slow for real life.
#define SLOW_DEBUG 0

#define PRINT_DEBUG 1

// Whether the app on the other side of the UART is an eMPL-client.py-like renderer.
#define UART_RENDER	0
#endif /* EVAL_BOARD */

#if NO22F
bool should_set_timestamp = false;

uint8_t overwrite_cur_sensors_per_sat[MAX_SATELLITES] = {1, 1, 4, 5, 1, 1};

#if OUTPUT_3D_CUBE_ONLY
bool sensing = true;
#else
bool sensing = false;
#endif
#endif /* NO22F */

// Metrics
uint32_t cur_num_of_sat = 2;
uint32_t cur_sat_ids[MAX_SATELLITES] = {7,8};
uint32_t new_lcp = 10;
uint32_t my_id_hub = 32;


//USED FOR COMMAND MANAGEMENT
static uint8_t command_sendings_left = 0;
static uint32_t command_to_sat[MAX_SAT_COMMSTRUCT] = {0};

/************************** Globals *******************************/
static uint32_t ct_rows_received[MAX_SATELLITES] = {0};
//static uint32_t ct_packets_received_at[MAX_SATELLITES] = {0};
uint32_t total_rows_received = 0;
static uint32_t valid_rs_ct = 0;
static uint32_t last_timestamp_seen_at[MAX_SENSORS] = {0};
//static uint8_t last_row_count_seen_at[MAX_SENSORS] = {0};
static uint32_t ct_out_of_turn = 0;
//static uint32_t requests_timed_out[MAX_SATELLITES] = {0};
static uint32_t rowct_from_sat[MAX_SATELLITES] = {0};

#define MS_PER_SAMPLE 10
//int32_t satellite_timestamp_adj[MAX_SENSORS] = {0};
//uint16_t ct_gap_rows[MAX_SENSORS] = {0};


static sth_payload_t last_payload[MAX_SENSORS];

// Current satellite idx to talk to.
// This is the index to cur_sat_ids.
static uint8_t target_sat_idx = 0;

static uint16_t last_received_crc16[MAX_SENSORS] = {0};
void reinit_esb(void);

// Variables used for RTC calibration.
static uint32_t timestamp_to_send = INVALID_TIMESTAMP;
static uint32_t local_clock_at_last_sending = 0;
static uint16_t last_rtc_id = 0;				/* The last RTC ID we sent (i.e., now waiting for the ack) */
static bool waiting_rtc_complete = false;
static uint16_t ack_from_sat = 0;

//static uint8_t ct_repeats_of_last_satellite = 0;
static uint8_t rows_in_last_packet = 0;
static uint32_t communicationTestCounter[MAX_SENSORS]; //CF: communication analysis 
static uint32_t communicationTestCounter_lastValue[MAX_SENSORS]={0}; //CF: communication analysis

// How often should we check for an ESB response from the satellites?
// And how long should we wait?
/// Keeps track of how long it took to receive responses.
/// For instance, if it took three cycles' worth of time, increment [3].
//uint32_t time_to_fill_histogram[CYCLES_TO_TIMEOUT+1] = {0};
/// Number of cycles that have elapsed since execution began.
static int32_t main_loop_cycle_counter = 0,
	max_cycle_to_timeout = 20,
	max_time_per_cycle = 400,
  max_repeats_of_last_satellite = 3;

uint32_t 		time_last_hearing_from_sat = 0;
uint32_t		time_last_talked_to_sat = 0;

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

void set_sat_idx(uint32_t num_of_sat, uint32_t satellite_ids[MAX_SENSORS]) {
	uint8_t cnt;
	
	cur_num_of_sat = num_of_sat;
	
	for (cnt = 0; cnt < MAX_SENSORS; cnt++) {
		if (cnt < cur_num_of_sat) {
			cur_sat_ids[cnt] = satellite_ids[cnt];
		} else {
			cur_sat_ids[cnt] = INVALID_SAT_ID;
		}
		
		for (int cc = cur_num_of_sat; cc < MAX_SATELLITES;cc++)
			cur_sat_ids[cc] = 0;
}

//cf
void resetPacketCounter(){
	uint8_t cnt;
	for (cnt = 0; cnt < MAX_SENSORS; cnt++) {
	 communicationTestCounter[cnt] = 0;
	 communicationTestCounter_lastValue[cnt] = 0;
	}
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

//uint8_t next_sat_idx(void) {

//	// Summary: if the current satellite is the farthest behind of
//	// all satellites, read from it up to 3 times in a row.
//	// Other than that, round-robin to the next satellite.
//	
//	// TODO: a totally missing satellite is considered "farthest
//	// behind", and so will take a lot of time away from the ones
//	// that actually exist. Maybe add logic to not retry a satellite
//	// unless you've heard from it at least once in the past ### ms?
//	uint8_t most_out_of_date_device_idx = 0;
//	uint32_t most_out_of_date_timestamp = INVALID_TIMESTAMP;
//	for(int i = 0; i < cur_num_of_sat; ++i)
//	{
//		if(last_timestamp_seen_at[i] < most_out_of_date_timestamp) {
//			most_out_of_date_device_idx = i;
//			most_out_of_date_timestamp = last_timestamp_seen_at[i];
//		}
//	}
//	if(most_out_of_date_device_idx == target_sat_idx && \
//		ct_repeats_of_last_satellite < max_repeats_of_last_satellite && rows_in_last_packet > 0) {
//		ct_repeats_of_last_satellite++;
//		return target_sat_idx;
//	}
//	target_sat_idx += 1;
//	if(target_sat_idx >= cur_num_of_sat) {
//		target_sat_idx = 0;
//	}
//	ct_repeats_of_last_satellite = 0;
//	return target_sat_idx;
//}

void update_cmd_to_sat(uint32_t command[MAX_SAT_COMMSTRUCT]) {	

	if((command[SAT_COMMSTRUCT_COMMA] !=0) && (command_sendings_left == 0))
	{
			command_to_sat[SAT_COMMSTRUCT_SATID] = command[SAT_COMMSTRUCT_SATID];
			command_to_sat[SAT_COMMSTRUCT_COMMA] = command[SAT_COMMSTRUCT_COMMA];
			command_to_sat[SAT_COMMSTRUCT_VALX1] = command[SAT_COMMSTRUCT_VALX1];
			command_to_sat[SAT_COMMSTRUCT_VALY2] = command[SAT_COMMSTRUCT_VALY2];
			command_to_sat[SAT_COMMSTRUCT_VALZ3] = command[SAT_COMMSTRUCT_VALZ3];

			command_sendings_left = COM_SENDING_CNT;
#if NO22F
		  command_to_sat[SAT_COMMSTRUCT_COMMA] = 0;
#endif
	}
}

/// Advance to the next satellite, build an up to date control packet, and
/// send it off.
static void talk_to_satellites(void) {
	uint8_t	 sat_idx;
	uint32_t local_clock;

//cwati	next_sat_idx();

	if(command_sendings_left) {
		command_sendings_left--;

		if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_WIF_SETRTC) {			
			local_clock = hub_timer_get_elapsed_ms();
			timestamp_to_send += (local_clock - local_clock_at_last_sending);
			command_to_sat[SAT_COMMSTRUCT_VALX1] = timestamp_to_send;
			local_clock_at_last_sending = local_clock;
			command_to_sat[SAT_COMMSTRUCT_VALY2] = last_rtc_id;
		}else if(((command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_WIF_SETLCP) ||\
			(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_SETBTCOMCH)) &&\
			(command_sendings_left==0))
		{
			//Finished notifying all satellites.  It's time to change channel.
				if (set_lcp(new_lcp)) {
					reinit_esb();
				}
		}else if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_SETNODTRPR)
		{
			
			if(command_to_sat[SAT_COMMSTRUCT_VALX1] > 0) max_repeats_of_last_satellite = command_to_sat[SAT_COMMSTRUCT_VALX1];
			if(command_to_sat[SAT_COMMSTRUCT_VALY2] > 0) max_cycle_to_timeout = command_to_sat[SAT_COMMSTRUCT_VALY2];
			if(command_to_sat[SAT_COMMSTRUCT_VALZ3] > 0) max_time_per_cycle = command_to_sat[SAT_COMMSTRUCT_VALZ3];
		}
	}else{
			command_to_sat[SAT_COMMSTRUCT_SATID] = 0;
			command_to_sat[SAT_COMMSTRUCT_COMMA] = 0;
			command_to_sat[SAT_COMMSTRUCT_VALX1] = 0;
			command_to_sat[SAT_COMMSTRUCT_VALY2] = 0;
			command_to_sat[SAT_COMMSTRUCT_VALZ3] = 0;	
	}
		
	//command |= NORDIC_CALIBRATE;
	
	hts_packet_t hts_packet = {0};
	hts_packet.header.sender = my_id_hub;

	hts_packet.hub_to_satellite_payload.command_toSat[SAT_COMMSTRUCT_SATID] = command_to_sat[SAT_COMMSTRUCT_SATID];
	/* Only forward command to sat if we need to */
	if (!command_is_for_hub_only(command_to_sat[SAT_COMMSTRUCT_COMMA])) {
		hts_packet.hub_to_satellite_payload.command_toSat[SAT_COMMSTRUCT_COMMA] = command_to_sat[SAT_COMMSTRUCT_COMMA];
	}
	hts_packet.hub_to_satellite_payload.command_toSat[SAT_COMMSTRUCT_VALX1] = command_to_sat[SAT_COMMSTRUCT_VALX1];
	hts_packet.hub_to_satellite_payload.command_toSat[SAT_COMMSTRUCT_VALY2] = command_to_sat[SAT_COMMSTRUCT_VALY2];
	hts_packet.hub_to_satellite_payload.command_toSat[SAT_COMMSTRUCT_VALZ3] = command_to_sat[SAT_COMMSTRUCT_VALZ3];
	hts_packet.hub_to_satellite_payload.recipient = cur_sat_ids[target_sat_idx];
	for (sat_idx = 0; sat_idx < cur_num_of_sat; sat_idx++) {
		hts_packet.hub_to_satellite_payload.last_seen_crc16[sat_idx] = last_received_crc16[sat_idx];
	}
	hts_packet.hub_to_satellite_payload.command_crc32 = 0;
	uint32_t payload_crc32 = crc32(0xFFFFFFFF, &hts_packet.hub_to_satellite_payload,
	                               sizeof(hts_packet.hub_to_satellite_payload));
	hts_packet.hub_to_satellite_payload.command_crc32 = payload_crc32;

	
	//cw
//	hts_packet.header.sender = 0xaabbccdd;
//		hts_packet.hub_to_satellite_payload.command = 0xababcdcd;
//		hts_packet.hub_to_satellite_payload.recipient =0xdeadc0d3;
//		hts_packet.hub_to_satellite_payload.command_crc32 = 0x11223344;
//		hts_packet.hub_to_satellite_payload.last_seen_crc16 = 0xeeffeeff;
//	
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
//volatile bool packet_waiting = false;
rfcbuf_t rfcbuf;
//sth_packet_t packet_to_process;

uint32_t cwati_discard_rfcbuf = 0;
uint32_t cwati_recvd_packet = 0;

static uint32_t cwati_complete_all_sat = 0;
static uint32_t cwati_incomplete = 0;
static uint32_t cwati_pkt_valid_cnt = 0;

// Our incoming packet handler.
void process_packet(sth_packet_t sth_packet_new) {
	cwati_recvd_packet++;
	
	if (rfcbufIsFull(&rfcbuf)) {
		rfcbufPopDiscard(&rfcbuf);
		cwati_discard_rfcbuf++;
	} else {
		rfcbufPush(&rfcbuf, sth_packet_new);
	}

//	// No significant work done here; just record the packet and signal
//	// to the main loop that there's something to read. This method is
//	// called from an interrupt handler, so having it do significant
//	// work is a drain on the firmware's sanity.
//	cwati_recvd_packet++;
//	if(!packet_waiting) {
//		cwati_recvd_and_processed++;
//		packet_to_process = sth_packet;
//		packet_waiting = true;
//	}
}


void reinit_esb() {
	net_init(process_packet, nack_warning, get_txchan_from_lcp(), get_rxchan_from_lcp());
}

// TODO: remove this when nack warnings are removed from net_common.
volatile bool packet_was_nacked = false;
void nack_warning(void) {
}

// Quaternion printouts are delayed for two reasons (I forget which
// one is more important):
// 1) The hub is very busy conducting the satellites.
// 2) There might be an interrupt conflict between the UART and other stuff.
static bool frame_to_print = false;
typedef struct frame_3d_ {
	float quat_w;
	float quat_x;
	float quat_y;
	float quat_z;
	} frame_3d_t;

static frame_3d_t frame_record = {0};

void print_frame_if_ready(uint8_t frame_sender) {
	if(!frame_to_print) return;
	frame_to_print = false;

	output_pyserial_record(frame_record.quat_x,
												 frame_record.quat_y,
												 frame_record.quat_z,
												 frame_record.quat_w,
												 frame_sender);

}

// Wasn't sure if the compiler was doing this the
// smart way (i.e. the way you do it without an FPU).
typedef union{float f;uint32_t i;} f2icast_t;
static inline bool fast_is_float_zero(float f) {
	f2icast_t f2icast;
	f2icast.f = f;
	// Zero floats will only ever havew their sign bit set.
	return (f2icast.i & 0x7fffffff) == 0;
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


// These were added to debug the time travel issue found on 7 July.
uint8_t next_jump_idx = 0;
struct{uint8_t sat_id;uint32_t before;uint32_t after;} jumps_seen[16] = {{0,0}};


uint32_t normalized_timestamp_for_satellite(uint8_t sat_idx, uint32_t this_timestamp) {
	// Upstream is not generating a predictable number of sensor records.
	// So this algorithm can't be trusted.
	// Just use the timestamps as sent instead.
	if(true) {
		do{
			// Timestamps from upstream should always be about 10ms apart. If they're
			// drastically wrong, make a note of it for debugging purposes.
			int32_t ts_diff = (this_timestamp - last_timestamp_seen_at[sat_idx]);
			if(ts_diff < 5 || ts_diff > 15) {
				jumps_seen[next_jump_idx].before = last_timestamp_seen_at[sat_idx];
				jumps_seen[next_jump_idx].after = this_timestamp;
				next_jump_idx = (next_jump_idx+1)%16;
			}
		}while(0);
		last_timestamp_seen_at[sat_idx] = this_timestamp;
		return this_timestamp;
	}
//	// If we ever reenable it, the original algorithm appears below.
//	if(this_timestamp == INVALID_TIMESTAMP) return this_timestamp;
//	this_timestamp = this_timestamp + satellite_timestamp_adj[sat_idx];
//	uint32_t last_timestamp = last_timestamp_seen_at[sat_idx];
//	uint32_t since_last_timestamp = this_timestamp - last_timestamp;
//	if(since_last_timestamp <= (MS_PER_SAMPLE*3)/2) {
//		// Gap is small enough that it's probably drift. Eliminate the bias.
//		int32_t d_bias = ((int32_t)since_last_timestamp) - MS_PER_SAMPLE;
//		int32_t d_adj = -d_bias;
//		satellite_timestamp_adj[sat_idx] += d_adj;
//		this_timestamp += d_adj;
//	} else {
//		ct_gap_rows[sat_idx]++;
//	}
//	// Otherwise, it's large enough that it's probably actually a missing row.
//	// Not enough info to tell which or how many are missing. Just leave it.
//	last_timestamp_seen_at[sat_idx] = this_timestamp;
//	return this_timestamp;
}

/// Assume |payload| is from a known good packet which came from |satellite_id|.
/// Deserializes the payload data and pushes it to the queue.
void process_good_packet(uint32_t satellite_id, sth_payload_t *payload) {
	uint8_t sat_idx;
	
	sat_idx = get_sat_idx(satellite_id);
	if(sat_idx == INVALID_SAT_IDX) {
		return;
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
		for(int i=0;i<SENSOR_RECORDS_PER_NET_PACKET;++i) {
			if(i >= payload->header.row_ct) {
				break;
			}
			if(
				fast_is_float_zero(payload->sensor_records[i].quat_w) &&
				fast_is_float_zero(payload->sensor_records[i].quat_x) &&
				fast_is_float_zero(payload->sensor_records[i].quat_y) &&
				fast_is_float_zero(payload->sensor_records[i].quat_z)
			) {
				// End of Packet
				rows_in_last_packet = i-1;
				break;
			}
			rowct_from_satellite[sat_idx]++;
			total_rowct_from_sats++;
			
			/// Translate the compact RF packet back into the standard SPI format.
			sat_sensor_record_t outbound_datum;
			outbound_datum.timestamp = 
		    normalized_timestamp_for_satellite(sat_idx,
		                                       running_timestamp);
			
			outbound_datum.sensor_idx = sat_idx;
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
			int percent_full = cbufNum(&outbound_queue) * 100 / CB_SIZE;
#else
			// TODO(cwajh): propogate WAIT state.
			int percent_full = cbufNum(&outbound_queue) * 100 / CB_SIZE;
			if(percent_full < 95) {
				if(cbufPush(&outbound_queue,outbound_datum) == E_OK) rows_pushed++;
			} else {
				cwati_cbuf_full++;
			}
#endif
#if UART_RENDER
#if OUTPUT_3D_CUBE_ONLY
			if((rowct_from_satellite[sat_idx] % 1) == 0)	{
#else
			if((rowct_from_satellite[sat_idx] % 100) == 0)	{
#endif
				frame_to_print = true;

				//CF 01/06/2016 - COMMUNICATION PROTOCOL IMPROVEMENT
				float tmp=(((float)payload->sensor_records[i].quat_w)/(CPI_QUATERNION16_BIAS_MULTIPLIER));
				if(tmp>=99.0f){
					frame_record.quat_w = ((float)payload->sensor_records[i].quat_w)/CPI_QUATERNION16_BIAS_MULTIPLIER;
					frame_record.quat_x = ((float)payload->sensor_records[i].quat_x)/CPI_QUATERNION16_BIAS_MULTIPLIER;
					frame_record.quat_y = ((float)payload->sensor_records[i].quat_y)/CPI_QUATERNION16_BIAS_MULTIPLIER;
					frame_record.quat_z = ((float)payload->sensor_records[i].quat_z)/CPI_QUATERNION16_BIAS_MULTIPLIER;
				}else{//in this case is sent the MAG BIAS values 
					frame_record.quat_w = ((float)payload->sensor_records[i].quat_w)/CPI_QUATERNION16_MULTIPLIER;
					frame_record.quat_x = ((float)payload->sensor_records[i].quat_x)/CPI_QUATERNION16_MULTIPLIER;
					frame_record.quat_y = ((float)payload->sensor_records[i].quat_y)/CPI_QUATERNION16_MULTIPLIER;
					frame_record.quat_z = ((float)payload->sensor_records[i].quat_z)/CPI_QUATERNION16_MULTIPLIER;
				}
			}
			print_frame_if_ready(sat_idx);
#endif
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
bool waiting_rx_packet_valid(sth_packet_t packet_to_process) {
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

/// Checks |packet| against previous packets to see if it's a retransmit or not. Parses and
/// queues any waiting rows that are now known to be trustworthy. |packet| must be valid (good
/// sender, trustworthy timestamp), but is allowed to be corrupted other than that.
void process_response(sth_packet_t packet_to_process) {
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
		process_good_packet(packet_to_process.header.sender, &last_payload[sat_idx]);
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

void check_rf_recv_buffer(void) {
	sth_packet_t	new_packet;
	err_t					ret;
	
	if (rfcbufIsEmpty(&rfcbuf)) {
				 return;
	}

	/* Disable interrupt while fetching rx.  This operation should be very quick,
	 * otherwise you risk losing incoming RF packet.
	 */
	CRITICAL_REGION_ENTER();
	ret = rfcbufPop(&rfcbuf, &new_packet);
	CRITICAL_REGION_EXIT();

	if (ret == E_OK) {
		if(waiting_rx_packet_valid(new_packet)) {
			time_last_hearing_from_sat = hub_timer_get_elapsed_ms();
			process_response(new_packet);
			cwati_pkt_valid_cnt++;
		} 
	}	
}

//void talk_to_satellites(void) {
//	// Communicate with a satellite.
//	cue_next_satellite();
//}

int main(void)
{
		bool				receive_complete = false;
	  uint32_t		time_now;
		static uint32_t max_quiet_wait = 50; /* ms.  Wait until everything is quiet before talking again */
		bool				havent_heard_too_long = false;
		bool				havent_talked_too_long = false;
#if EVAL_BOARD
		static bool send_sat_id = true;
		static uint8_t send_sat_id_cur_idx = 0;
#endif
#if PRINT_DEBUG
		char report[110] = {0};
#endif 

    // Initialize the 16MHz clock used by SPI and ESB.
		// Supposedly uses a crystal instead of the internal RCOSC
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
       // Wait
    }
		
		rfcbufInit(&rfcbuf);
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
		
		hub_timer_start();
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
		hub_timer_start();

#if NO22F
		set_lcp(new_lcp);
#if PIVOT_3_0
		cur_num_of_sensors = 0;
		
		for (uint8_t qq = 0; qq < cur_num_of_sat; qq++) {
			cur_sensors_per_sat[qq] = overwrite_cur_sensors_per_sat[qq];
			cur_num_of_sensors += get_sensor_num_of_sat_idx(qq);
		}
#endif /* PIVOT_3_0 */
#endif

    while (true)
    {   
			// When a SET_RTC is sent out, every satellite hopefully receives it.
			// But some may not. As insurance, we send out the RTC multiple times,
			// advancing the value each time to match the previous ones.
			if(should_set_timestamp) {
				for(int i = 0; i < cur_num_of_sat; ++i) {					// This bit is part of the (disabled + probably wrong) timestamp realignment algo.
//					ct_gap_rows[i] = 0;
//					satellite_timestamp_adj[i] = 0;
				}
#if PRINT_DEBUG

				char rtc_string[20] = {0};
				debug_putstring((uint8_t*)"\r\nSetting RTC to ");
				snprintf(rtc_string,20,"%u",current_timestamp);
				debug_putstring((uint8_t*)rtc_string);
#endif //PRINT_DEBUG
				cwati_cbuf_full = 0;

				timestamp_to_send = current_timestamp;
				local_clock_at_last_sending = hub_timer_get_elapsed_ms();
				ct_rtc_sendings_left = RTC_SENDING_CNT;
				should_set_timestamp = false;
			}
			
			time_now = hub_timer_get_elapsed_ms();

			havent_heard_too_long = (((time_now > time_last_hearing_from_sat) && \
				   (time_now - time_last_hearing_from_sat > max_quiet_wait)) || \
					/* timer has reset so that previous time is bigger */
			    (time_last_hearing_from_sat > time_now));
			
			havent_talked_too_long = (((time_now > time_last_talked_to_sat) && \
				   (time_now - time_last_talked_to_sat > max_quiet_wait)) || \
					/* timer has reset so that previous time is bigger */
			    (time_last_talked_to_sat > time_now));
			
			receive_complete = cwati_pkt_valid_cnt == cur_num_of_sat;

			static uint32_t cwati_talking = 0, cwati_not_talking = 0;
			if (receive_complete || (havent_heard_too_long && havent_talked_too_long)) {
						
					if (receive_complete) {
						cwati_complete_all_sat++;
					} else {
						cwati_incomplete++;
					}
					talk_to_satellites();
					time_last_talked_to_sat = hub_timer_get_elapsed_ms();
					cwati_pkt_valid_cnt = 0;
					cwati_talking++;
			} else {
				cwati_not_talking++;
			}
			
			nrf_delay_ms(100);//cwati ridiculous delay cwati todo
			check_rf_recv_buffer();

			/* 
			 *
			 *
			 *
			 *
			 *
			 *
			 *
			 * End of real code.  Below is just printf */
			
#if SLOW_DEBUG
			// Rate control so that in slow debug mode, the hub is only reading a few packets per second.
			nrf_delay_ms(300);
#endif
			
#if EVAL_BOARD
			if (control_button_pressed(3)) {
				sensing = false;
#if PRINT_DEBUG
				debug_putstring("Set sensing to false");
#endif /* PRINT_DEBUG */	
			}
			if (control_button_pressed(2)) {
				sensing = true;
#if PRINT_DEBUG
				debug_putstring("Set sensing to true");
#endif /* PRINT_DEBUG */				
			}
			if (control_button_pressed(1)) {
//				should_set_lcp = true;
//				new_lcp = get_next_lcp();
//				debug_putstring("Hop Up LCP!");

				sensing = false;
				should_set_timestamp = true;
				current_timestamp = 0;
#if PRINT_DEBUG
				debug_putstring("Set RTC to 0, not sending!");
#endif /* PRINT_DEBUG */
			}
#endif /* EVAL_BOARD */

#if PRINT_DEBUG
			static uint32_t time_start = 0, time_diff;
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
						time_print_now = hub_timer_get_elapsed_ms();
						if (time_print_now - last_time_print > 3000) {
							/* Print every 3s */
							print_now = true;
							last_time_print = time_print_now;
						}
					}
				}
			}
			
			if(print_now) {
					
				last_total_rowct_from_sats = total_rowct_from_sats;
				time_now = hub_timer_get_elapsed_ms();
				time_diff = time_now - time_start;
				
				for (uint8_t qq = 0; qq < cur_num_of_sat; qq++) {
					pkt_now[qq] = rowct_from_satellite[qq];
					pkt_diff[qq] = pkt_now[qq] - pkt_start[qq];

					snprintf(report,109,"\r\nPacket rate[%d]: %u pkts/ %u ms = %f pkts/s", qq, pkt_diff[qq], time_diff,
							(float) ((float)pkt_diff[qq] / ((float)(time_diff) / 1000.0)));
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

				snprintf(report,109,"\r\nTX chan: %u RX chan: %u", get_txchan_from_lcp(), get_rxchan_from_lcp());
				debug_putstring((uint8_t*)report);
				
				snprintf(report,109,"\r\nSat ID: [0]=0x%x [1]=0x%x [2]=0x%x [3]=0x%x [4]=0x%x CUR NUM OF SAT: %d",
					cur_sat_ids[0], cur_sat_ids[1], cur_sat_ids[2], cur_sat_ids[3], cur_sat_ids[4], cur_num_of_sat);
				debug_putstring((uint8_t*)report);

//				snprintf(report,109, "\r\nBuffer deficit %d/%d. %sending. %sensing.",
//														rows_pushed - rows_popped, cbufNum(&outbound_queue),
//				sending?"S":"Not s", sensing?"S":"Not s");
//				debug_putstring((uint8_t*)report);
//				
//				snprintf(report,109, "\r\nCbuf lost: %u.",cwati_cbuf_full);
//				debug_putstring((uint8_t*)report);
//				
//				debug_putstring((uint8_t*)"\r\nRequest timeout [");
//				for(int qq=0;qq<=cur_num_of_sat;++qq) {
//					snprintf(report,99,"%d, ",requests_timed_out[qq]);
//					debug_putstring((uint8_t*)report);
//				}
//				snprintf(report,99,"X]");
//				debug_putstring((uint8_t*)report);
//				debug_putstring((uint8_t*)"\r\nBy source [");
//				for(int qq=0;qq<cur_num_of_sat;++qq) {
//					snprintf(report,99,"%d, ",ct_rows_received[qq]);
//					debug_putstring((uint8_t*)report);
//				}
//				debug_putstring((uint8_t*)"X]");
//				debug_putstring((uint8_t*)"\r\nBy wait [");
//				for(int qq=0;qq<cur_num_of_sat;++qq) {
//					snprintf(report,99,"%d, ",ct_packets_received_at[qq]);
//					debug_putstring((uint8_t*)report);
//				}
//				debug_putstring((uint8_t*)"X]");
//				debug_putstring((uint8_t*)"\r\nT+[");
//				for(int qq=0;qq<cur_num_of_sat;++qq) {
//					snprintf(report,99,"%d, ",last_timestamp_seen_at[qq]);
//					debug_putstring((uint8_t*)report);
//				}
//				debug_putstring((uint8_t*)"X]");
//				debug_putstring((uint8_t*)"\r\nT adj[");
//				for(int qq=0;qq<cur_num_of_sat;++qq) {
//					snprintf(report,99,"%d, ",satellite_timestamp_adj[qq]);
//					debug_putstring((uint8_t*)report);
//				}
//				debug_putstring((uint8_t*)"X]");
//				debug_putstring((uint8_t*)"\r\nT gaps[");
//				for(int qq=0;qq<cur_num_of_sat;++qq) {
//					snprintf(report,99,"%d, ",ct_gap_rows[qq]);
//					debug_putstring((uint8_t*)report);
//				}
//				debug_putstring((uint8_t*)"X]");
//				debug_putstring((uint8_t*)"\r\nCr[");
//				for(int qq=0;qq<17;++qq) {
//					snprintf(report,99,"%d, ",corruption_histogram[qq]);
//					debug_putstring((uint8_t*)report);
//				}
//				debug_putstring((uint8_t*)"X]");
//				debug_putstring((uint8_t*)"\r\nCo[");
//				for(int qq=0;qq<3;++qq) {
//					snprintf(report,99,"%d, ",corruption_correlation[qq]);
//					debug_putstring((uint8_t*)report);
//				}
//				debug_putstring((uint8_t*)"X]");
				debug_putstring("\n");
				
				// Try to prevent this message being printed many times in a row, even if the counter
				// doesn't advance.
				main_loop_cycle_counter++;				
			}
#endif /* PRINT_DEBUG */
    }
}





