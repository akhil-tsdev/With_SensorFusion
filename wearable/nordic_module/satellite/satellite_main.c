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

#include "boards.h"
#include "cbuf.h"
#include "crc16.h"
#include "crc32.h"
#include "hub_timer.h"
#include "led_trace.h"
#include "mpu9250_freq.h"
#include "net_common.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "pivot3_0_poc.h"
#include "rfcbuf.h"
#include "satellite_main.h"
#include "simple_uart.h"
#include "ts_spi_master.h"

/// Starts at 1 for satellites. Must be different for EVERY satellite.
uint32_t my_sat_id = 1;
uint8_t  my_sat_idx = INVALID_SAT_IDX;
uint32_t my_hub_id = 297849456;
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
uint32_t communicationTestCounter[MAX_SENSORS] = {0};
uint8_t communicationTestCounterResend = 0;

// Keep track of number of satellites in the system
uint32_t cur_num_of_sat = 3;
uint32_t cur_sat_ids[MAX_SATELLITES] = {0};
uint32_t num_of_sensors[MAX_SATELLITES] = {0};
uint32_t sat_id_before_me = 0;
bool     last_to_speak = false;			/* Need to keep track if we're last in this round robin */
uint32_t last_talk_to_22f = 0;

// We only want to bother communicating with the Freescale if one of us has
// something to say to the other. We can't quite know when the Freescale
// will have something to say, so we use an exponential back-off to approximate.
#define ATOM_US 20
#define MS_TO_TIME_OUT_ESB 2000
#define CYCLES_TO_TIME_OUT_ESB (MS_TO_TIME_OUT_ESB * 1000 / ATOM_US)
#define CYCLES_PER_SPI_ROUND 1 /*((DELAY_MS*1000)/ATOM_US)*/

// These were added to debug the time travel issue found on 7 July.
uint32_t last_timestamp_seen = 0;
uint8_t next_jump_idx = 0;
struct{uint32_t before;uint32_t after;} jumps_seen[16] = {{0,0}};

bool last_spi_had_row = false;

// Keep track of last RF communication
//uint32_t cycles_since_rx = 0;
//uint32_t heartbeat_period_to_22f = (1000 / TS_INITIAL_SENDING_FREQ) - 2 ;  /* ms default */
uint32_t heartbeat_period_to_22f = MAX_SENSORS_IN_CS;  /* ms default */

// When running on an eval stack, every press of Button 4
// will make this function return true once. Used to trigger
// UART logging.
bool control_button_pressed(void);


// Tracks synchronization with the hub. After being generated, a payload packet
// is kept and re-sent until the hub replies with a matching CRC (confirming that
// the packet has arrived safe and sound).
uint16_t last_crc_rcvd_from_hub = 0;
uint16_t expected_crc16 = 0;
uint16_t previous_crc16 = 0;
static sth_payload_t next_payload;
static bool first_payload_generated = false;
uint32_t last_hearing_from_hub;
uint32_t invalid_timestamp = INVALID_TIMESTAMP;
uint16_t invalid_timestamp_crc;

// System state, set according to commands from the hub.
bool start_dumping = false;
bool should_set_timestamp = false;
uint32_t timestamp_to_set = INVALID_TIMESTAMP;
//bool must_calibrate = false;
bool enable_command = false;
//bool must_change_lct_channel = false;
uint16_t last_rtc_id = 0;
//SAT COMMAND MANAGEMENF CF
uint32_t command_to_sat[MAX_SAT_COMMSTRUCT];

/// Contains readings from the Freescale, enqueued to be sent to the hub.
cbuf_t data_queue;

rfcbuf_t rfcbuf;


uint32_t rfcbufpop_discard = 0;

void reinit_esb() {
	net_init(satellite_rx_handler, 0, get_txchan_from_lcp(), get_rxchan_from_lcp());
}

//cf: 20160714 - for testing communication
int16_t operativeMode;
void setCalculatedFrame(sat_sensor_record_t * sensor_record){
		sensor_record_t* data = &(sensor_record->data);
	
	  operativeMode=data->quat_w/CPI_QUATERNION16_BIAS_MULTIPLIER;
		if(operativeMode==CPI_QUATERNION16_COUNTER_COMMAND){
			
			if(data->accel_x > 0){
				communicationTestCounter[sensor_record->sensor_idx]++;
			}
			else{
				communicationTestCounter[sensor_record->sensor_idx]=0;
			}
			
				data->accel_z = (int16_t)(communicationTestCounter[sensor_record->sensor_idx]%CPI_QUATERNION16_COUNTER_MAXVALU);
			
			data->gyro_x =  100 * communicationTestCounterResend + next_payload.header.row_ct; //CF: 100 * number of sending of same packet + row_ct
		}else if(operativeMode==CPI_QUATERNION16_VERSION_COMMAND){
			data->gyro_x = (int16_t)REVIS_VERSION_SATNRD;
		}
}

/// Builds a satellite-to-hub packet payload using data popped from the queue.
/// You MUST save the result someplace safe, else the rows will be lost!
sth_payload_t new_payload(void) {
 first_payload_generated = true;
 
 // Initial/default state: a packet with no rows.
 sth_payload_t payload = {0};
 sat_sensor_record_t data;
 packets_generated++;
 uint32_t packet_timestamp = INVALID_TIMESTAMP;
 payload.header.row_ct = 0;
 
 uint32_t last_timestamp;
 int32_t ts_diff = 0;
 
 if(start_dumping) {
  for(int i=0;i<SENSOR_RECORDS_PER_NET_PACKET;++i) {
   // For each additional row available to send...
   if(cbufPop(&data_queue,&data) == E_OK) {

    if (!i)
    {
     packet_timestamp = data.timestamp;
     last_timestamp = packet_timestamp;
    }
    
    ts_diff = data.timestamp - (last_timestamp);
    
		/* We can only take diff that fits uint8_t */
		if (ts_diff < 0 || ts_diff > UINT8_MAX) {
			break;
		}
		
    // Row looks good to include in this packet.
    
    setCalculatedFrame(&data);
    payload.sensor_records[i] = pack_sensor_record(data.data);
    payload.sensor_records[i].ts_diff = ts_diff;
		payload.sensor_records[i].sensor_idx = data.sensor_idx;
    last_timestamp = data.timestamp;
		payload.sensor_records[i].crc = 0;
    payload.sensor_records[i].crc = crc8_compute((uint8_t*)(&(payload.sensor_records[i])),sizeof(reduced_sensor_record_t));
    rows_sent++;
    payload.header.row_ct++;
   } else {
		 break;
	 }
  }
 }
 payload.header.timestamp = packet_timestamp;
 payload.header.timestamp_crc = crc16_compute((uint8_t*)(&packet_timestamp),sizeof(packet_timestamp),0);
// payload.diagnostic_pattern1 = 0x50;
// payload.diagnostic_pattern2 = 0xFA;
 payload.header.rtc_id_ack = last_rtc_id;

 /* cwati TODO in pivot YOGA, if we want to send more than one packet,
 * then we only set end_of_roundrobin = true on the very last packet.
 */
 if (last_to_speak) {
				 payload.header.end_of_roundrobin = true;
 } else {
				 payload.header.end_of_roundrobin = false;
 }

 payload.header.received_crc16 = 0;
 payload.header.received_crc16 = crc16_compute((uint8_t*)(&payload),sizeof(payload),0);
 return payload;
}

// TODO: this is an out-of-band return value. Find a better way to pass it
// back to the main loop.
uint8_t rows_replied = 0;

//void save_last_hub_crc(uint16_t last_crc) {
//	if (my_sat_idx != INVALID_SAT_IDX) {
//		last_crc_rcvd_from_hub = last_crc;
//	}
//}

/// Checks whether the hub has received the current response payload correctly
/// If so, loads up a new response payload to be sent.
void prepare_response() {
	uint32_t rows_sent_before;
	
	rows_replied = 0;
	if (last_crc_rcvd_from_hub == expected_crc16 || !first_payload_generated || !last_crc_rcvd_from_hub) {
		previous_crc16 = expected_crc16;
		rows_sent_before = rows_sent;
		next_payload = new_payload();
		rows_replied = rows_sent - rows_sent_before;
		communicationTestCounterResend = 0;
	} else if(last_crc_rcvd_from_hub == previous_crc16) {
		laggard_requests_seen++;
	} else {
		nonsense_requests_seen++;
	}
	communicationTestCounterResend++;
}

/// Replies to the hub with the currently prepared data payload. Blocks until sending is over.
void send_response(void) {
	sth_packet_t sth_packet;
	
	/* If we haven't got our valid Satellite ID, refrain from sending */
	if (my_sat_id != INVALID_SAT_ID) {
		sth_packet.header.sender = my_sat_id;
		sat_id_forCommand = my_sat_id;
	} else {
		return;
	}
	
	//CF:NOTE SPI COUNTER THERE(?)
	/*if(operativeMode==CPI_QUATERNION16_COUNTER_COMMAND){
		for(int i = 0; i < SENSOR_RECORDS_PER_NET_PACKET; i++)
			if(next_payload.sensor_records[i].accel_x>0){
				communicationTestCounter[1]++;
				next_payload.sensor_records[i].gyro_x = (int16_t)(communicationTestCounter[1]%CPI_QUATERNION16_COUNTER_MAXVALU);
			}
	}*/
	
	/* Must not modify packet here, otherwise expected_crc16 will be wrong */
	sth_packet.satellite_to_hub_payload = next_payload;
	
	bool result = send_packet(sth_packet);
//	last_to_speak = false;
	
	int timeout_10us = 0;
	while(fate_of_last_tx == TX_FATE_UNKNOWN) {
		nrf_delay_us(10);
		timeout_10us++;
		if(timeout_10us > 100000) {
			// It's been 1000ms. Assume something went wrong. 
			
			// Reset the RF library since we can't know if state is still good.
			reinit_esb();			
			break;
		}
	}

}

void satellite_rx_handler(void* packet, net_packet_type_t packet_type) {
	uint32_t sender_id;
	rx_packet_t incoming_packet;

	//debug
	uint32_t time_now = hub_timer_get_elapsed_ms();
	//end of debug
	
//	cycles_since_rx = 0;
	
	if (packet_type == hts_packet) {	
		sender_id = *(uint32_t*)(packet);
		/* Save time here by ignoring unknown packet right away. */
		if (sender_id != my_hub_id) {
			return;
		}
		
		incoming_packet.packet.hts = *(hts_packet_t*)(packet);
		incoming_packet.packet_type = hts_packet;
		incoming_packet.timestamp = hub_timer_get_elapsed_ms();
		if (rfcbufIsFull(&rfcbuf)) {
			rfcbufPopDiscard(&rfcbuf);
			rfcbufpop_discard++;
		}
		rfcbufPush(&rfcbuf, incoming_packet, hub_timer_get_elapsed_ms());
	} else if (packet_type == sth_packet) {
		/* Quick Store *
		 * WARNING!!! We only store information we're interested in in order to save time.
		 * CRC value inside the packet is not stored!!! If we later decide to check CRC
		 * update this cbuf!! */
		incoming_packet.packet.sth.satellite_to_hub_payload.header.end_of_roundrobin = \
			((sth_packet_t*)packet)->satellite_to_hub_payload.header.end_of_roundrobin;
		incoming_packet.packet_type = sth_packet;
		incoming_packet.packet.sth.header.sender = ((sth_packet_t*)packet)->header.sender;
		incoming_packet.timestamp = hub_timer_get_elapsed_ms();

		if (rfcbufIsFull(&rfcbuf)) {
			rfcbufPopDiscard(&rfcbuf);
			rfcbufpop_discard++;
		}
		rfcbufPushSatPkt(&rfcbuf, incoming_packet, hub_timer_get_elapsed_ms());
	}
}

static void update_sat_before_me() {
	uint8_t sat_idx;

	/* case for our id as sat_idx 0 */
	if (cur_sat_ids[0] == my_sat_id) {
		sat_id_before_me = cur_sat_ids[cur_num_of_sat - 1];
		return;
	}
	
	for (sat_idx = 1; sat_idx < cur_num_of_sat; sat_idx++) {
		if (cur_sat_ids[sat_idx] == my_sat_id) {
			sat_id_before_me = cur_sat_ids[sat_idx - 1];
		}
	}
	return;
}

static void set_sat_idx(uint32_t command_toSat[MAX_SAT_COMMSTRUCT]) {
//	uint32_t command_toSat[MAX_SAT_COMMSTRUCT];
//	
//	for (uint8_t qq; qq < MAX_SAT_COMMSTRUCT; qq++) {
//		command_toSat[qq] = command[qq];
//	}

	cur_num_of_sat = command_toSat[SAT_COMMSTRUCT_VALZ3];

	if (command_toSat[SAT_COMMSTRUCT_VALX1] < cur_num_of_sat) {
		cur_sat_ids[command_toSat[SAT_COMMSTRUCT_VALX1]] = command_toSat[SAT_COMMSTRUCT_VALY2];
		if (command_toSat[SAT_COMMSTRUCT_VALT4]) {
			cur_sensors_per_sat[command_toSat[SAT_COMMSTRUCT_VALX1]] = command_toSat[SAT_COMMSTRUCT_VALT4];
		}
		
		/* Update our order  of turn to speak to hub */
		if (command_toSat[SAT_COMMSTRUCT_VALY2] == my_sat_id) {
			my_sat_idx = command_toSat[SAT_COMMSTRUCT_VALX1];
		}
	} else {
		cur_sat_ids[command_toSat[SAT_COMMSTRUCT_VALX1]] = INVALID_SAT_ID;
	}

	for (int cc = cur_num_of_sat; cc < MAX_SATELLITES;cc++)
	{
		cur_sat_ids[cc] = 0;
		cur_sensors_per_sat[cc] = 0;
	}
	
	update_sat_before_me();
	
	if (cur_num_of_sat > 1) {
		if ((cur_num_of_sat - 1) == my_sat_idx) {
			last_to_speak = true;
		} else {
			last_to_speak = false;
		}
	} else {
		last_to_speak = true;
	}
	
}

void process_command(uint32_t commandFromHub[MAX_SAT_COMMSTRUCT]) {	
	uint32_t freq_divider, freq;
	
	// GL - wait for a command to be shipped before listening to others commands
	if (enable_command == false) 
	{
	enable_command = (my_sat_id == commandFromHub[SAT_COMMSTRUCT_SATID] || 
												 sat_id_forCommand == commandFromHub[SAT_COMMSTRUCT_SATID] ||
												 commandFromHub[SAT_COMMSTRUCT_SATID] == 0); //cf
	}
	if((my_sat_id != INVALID_SAT_ID) && !sat_id_forCommand) {
		sat_id_forCommand=my_sat_id;
	}
		
	//sat command management cf
	if(enable_command) {

	  if(commandFromHub[SAT_COMMSTRUCT_COMMA] == SAT_COMM_WIF_SETRTC) {
	  	should_set_timestamp = true;
	  	timestamp_to_set = commandFromHub[SAT_COMMSTRUCT_VALX1];
		  last_rtc_id = commandFromHub[SAT_COMMSTRUCT_VALY2];
		  if (timestamp_to_set == 0) {
		  	cbufInit(&data_queue);
		  }
	  }
	
//		must_calibrate = true;
		uint8_t cnt;
		for (cnt = 0; cnt < MAX_SAT_COMMSTRUCT; cnt++) {
				command_to_sat[cnt] = commandFromHub[cnt];
		}
		
		
		
		//cf 20160518 - change communication channel
		if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_SETBTCOMCH || command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_WIF_SETLCP){
			command_to_sat[SAT_COMMSTRUCT_VALY2] = set_lcp(command_to_sat[SAT_COMMSTRUCT_VALX1]); //the result is sent to the SAT's 22F
			//if(command_to_sat[SAT_COMMSTRUCT_VALY2])
				//reinit_esb();
		}
		
		//cf 20160714 - enable counter sending mode, for communication testing
		if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_TESTCOMSTC){
			for (uint8_t qq = 0; qq < MAX_SENSORS; qq++) {
				communicationTestCounter[qq] = 0;
			}
		}

		//cf 20161014 - MANAGEMENT OF START DUMPING WITH SAT COMMAND
		if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_START_DUMP){
				start_dumping = true;
		}
		
		//cf 20161014 - MANAGEMENT OF STOP DUMPING WITH SAT COMMAND
		if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_STOP_DUMPI){
				start_dumping = false;
				/* Empty cbuf FAST_DRAIN */
        cbufInit (&data_queue);
		}
		
		//cw 20170614 - SET NUMBER OF SATS IN THE SYSTEM

		if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_WIF_SETSAT){
			if (commandFromHub[SAT_COMMSTRUCT_VALZ3] != 0) {
				set_sat_idx(command_to_sat);
				
				/* Update round robin order */
				if (last_to_speak) {
					next_payload.header.end_of_roundrobin = true;
				} else {
					next_payload.header.end_of_roundrobin = false;		
				}
				
			  next_payload.header.received_crc16 = 0;
			  expected_crc16 = crc16_compute((uint8_t*)(&next_payload),sizeof(next_payload),0);
			  next_payload.header.received_crc16 = expected_crc16;
			}			
		}
		
		if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_WIF_SETSA2){
			if (commandFromHub[SAT_COMMSTRUCT_VALZ3] != 0) {
				set_sat_idx(command_to_sat);
				
				/* Update round robin order */
				if (last_to_speak) {
					next_payload.header.end_of_roundrobin = true;
				} else {
					next_payload.header.end_of_roundrobin = false;		
				}
				
			  next_payload.header.received_crc16 = 0;
			  expected_crc16 = crc16_compute((uint8_t*)(&next_payload),sizeof(next_payload),0);
			  next_payload.header.received_crc16 = expected_crc16;
			}
		}

		//cw 20170621 - UPDATE PERIOD TO TALK TO SATELLITE 22F
		if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_FRQDIVIDER){
			if (commandFromHub[SAT_COMMSTRUCT_VALX1] != 0) {
				freq_divider = commandFromHub[SAT_COMMSTRUCT_VALX1];
				
				/* Find the Hz */
				freq = TS_GYRO_FREQ / freq_divider;
				
				heartbeat_period_to_22f = 1000 /* 1s *// freq; /* in ms */
			}
		}		
	}
}

//static uint8_t get_sat_idx(uint32_t sat_ID) {
//	uint8_t sat_idx = INVALID_SAT_IDX;
//	
//	for (uint8_t idx = 0; idx < cur_num_of_sat; idx++) {
//		if (cur_sat_ids[idx] == sat_ID) {
//			sat_idx = idx;
//			break;
//		}
//	}
//	
//	return sat_idx;
//}

uint32_t process_rx_packet(uint8_t* end_of_rr, uint32_t* ts) {
	uint32_t 			sent_crc32 ;
	uint32_t 			calculated_crc32;
	bool					valid_packet = false;
	rx_packet_t		packet_to_process;
	uint32_t			sender = UINT32_MAX;
	
	//cwati todo there should be a mutex mechanism here, in case RF callback modifies
	//at the same time.
	if (rfcbufPop(&rfcbuf, &packet_to_process, ts) != E_OK) {
		return sender;
	}
	
	/* We got the packet from cbuf, now verify if packet is valid */
	if (packet_to_process.packet_type == sth_packet) {
		sender = packet_to_process.packet.sth.header.sender;
		*end_of_rr = packet_to_process.packet.sth.satellite_to_hub_payload.header.end_of_roundrobin;
		return sender;
	} else if (packet_to_process.packet_type == hts_packet) {
		sender = packet_to_process.packet.hts.header.sender;
		
		/* Let's not waste time trying to process empty packet */
		if (packet_to_process.packet.hts.hub_to_satellite_payload.command_toSat[SAT_COMMSTRUCT_COMMA] == \
			SAT_COMM_IDLE_CYCLE) {
			return sender;
		}
		
		sent_crc32 = packet_to_process.packet.hts.hub_to_satellite_payload.command_crc32;
		packet_to_process.packet.hts.hub_to_satellite_payload.command_crc32 = 0;
		calculated_crc32 = crc32(0xFFFFFFFF, &packet_to_process.packet.hts.hub_to_satellite_payload,
																	 sizeof(packet_to_process.packet.hts.hub_to_satellite_payload));
		
		valid_packet = calculated_crc32 == sent_crc32;
	}		

	/* If there is no valid packet to process, just return */
	if (valid_packet == FALSE) {
		return sender;
	}

	if (packet_to_process.packet_type == hts_packet) {	
		/* Save last hub CRC */
		if (my_sat_idx != INVALID_SAT_IDX) {
			last_crc_rcvd_from_hub = packet_to_process.packet.hts.hub_to_satellite_payload.last_seen_crc16[my_sat_idx];
		}
		process_command(&packet_to_process.packet.hts.hub_to_satellite_payload.command_toSat[0]);
	}
	
	return sender;
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

uint32_t cwati_spi_notok = 0, cwati_spi_ok = 0;

void spi_received(sat_to_nordic_t packet) {
	static bool first_time = true;
	sat_sensor_record_t cbuf_packet;
	uint16_t crc16, recv_crc16;
		
	/* Calculate CRC16 */
	recv_crc16 = packet.crc16;
	packet.crc16 = 0;
	crc16 = crc16_compute((uint8_t*)(&packet), sizeof(packet), 0);
		
	
	if (recv_crc16 != crc16) {
		last_spi_had_row = false;
		nrf_delay_us(100000);
		cwati_spi_notok++;
		return;
	} 
	
	cwati_spi_ok++;
	
		my_sat_id = packet.sat_id;
		my_hub_id = packet.hub_id;
	
	// sat and hub id
		if (first_time) {
		first_time = false;
	}
	
	
	if(lcp_valid(packet.nrdChannel))
		set_lcp(packet.nrdChannel);

	if(packet.data.timestamp == INVALID_TIMESTAMP) {
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
	memcpy(&cbuf_packet, &packet.data, sizeof(packet.data));
	
	/* Change sensor index from local index to system sensor index */
	if (my_sat_idx != INVALID_SAT_IDX) {
		cbuf_packet.sensor_idx = get_starting_sensor_idx_of_sat_idx(my_sat_idx) + cbuf_packet.sensor_idx;
		if (cbuf_packet.sensor_idx > (MAX_SENSORS - 1)) {
			return; //invalid
		}
	}
	
	if (cbufPush(&data_queue,cbuf_packet) != E_OK) {
		cbufPopDiscard(&data_queue);
		cbufPush(&data_queue,cbuf_packet);
	}
}

/// Decide what command to send the Freescale based on our own state.
nordic_to_sat_t command_for_spi(void) {
	nordic_to_sat_t command = {.sat_command[SAT_COMMSTRUCT_SATID] = 0, .sat_command[SAT_COMMSTRUCT_VALX1] = INVALID_TIMESTAMP};
//	bool sensor_should_be_quiet = CBUF_IS_REDZONE(&data_queue);
//	
//	if (sensor_should_be_quiet || (sending_enabled == false)) {
//		command.command |= SAT_SPI_WAIT;
//	} 
 
	//GL - enable_command is the old must_calibrate
	if (enable_command) {

		command.sat_command[SAT_COMMSTRUCT_SATID] = command_to_sat[SAT_COMMSTRUCT_SATID];//THIS POSITION ARE FREE - BEFORE CONTAIN THE 32BIT OLD HUB COMMAND
		command.sat_command[SAT_COMMSTRUCT_COMMA] = command_to_sat[SAT_COMMSTRUCT_COMMA];
		command.sat_command[SAT_COMMSTRUCT_VALX1] = command_to_sat[SAT_COMMSTRUCT_VALX1];
		command.sat_command[SAT_COMMSTRUCT_VALY2] = command_to_sat[SAT_COMMSTRUCT_VALY2];
		command.sat_command[SAT_COMMSTRUCT_VALZ3] = command_to_sat[SAT_COMMSTRUCT_VALZ3];

		enable_command=false;
	}
	
	if (should_set_timestamp) {
		command.sat_command[SAT_COMMSTRUCT_COMMA] = SAT_COMM_WIF_SETRTC;
		command.sat_command[SAT_COMMSTRUCT_VALX1] = timestamp_to_set;
		should_set_timestamp = false;
	}
//	
//	//cf automatic LCT channel change requested to the 22F
//	if(0){//must_change_lct_channel){
//		command.sat_command[SAT_COMMSTRUCT_SATID] = 0;
//		command.sat_command[SAT_COMMSTRUCT_COMMA] = SAT_COMM_SETBTCOMAU;
//		command.sat_command[SAT_COMMSTRUCT_VALX1] = MAX_LCP;
//		
//		must_change_lct_channel=false;
//	}
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
	
	int lcp_tmp=get_lcp();
	
	if (command.sat_command[SAT_COMMSTRUCT_COMMA] != 0) {
		command.sat_command[SAT_COMMSTRUCT_SATID] = command.sat_command[SAT_COMMSTRUCT_SATID];
	}
	
	/*
	if (start_dumping) {
		command.sat_command[SAT_COMMSTRUCT_SATID] = SAT_COMM_START_DUMP;
	} else {
		command.sat_command[SAT_COMMSTRUCT_SATID] = SAT_COMM_STOP_DUMPI;
	}
	*/
	return command;
}

/// Blocks until communication is done. Returns whether communication was successful.
/// Only one of the SPI master library and the ESB library can be active at once.
/// If you're using ESB, set rf_active, so that it can be turned off during the
/// communication. If rf_active=true and communication was successful, when this
/// returns RF will be off. Otherwise, it should be unchanged from its original state.
static bool do_one_spi_communication(bool rf_active) {
	nordic_to_sat_t command = command_for_spi();

	bool was_able_to_communicate = spi_send_command(command,spi_received);
	if(!was_able_to_communicate) {
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

//	time_cwati1 = hub_timer_get_elapsed_ms();
	if(do_one_spi_communication(true)) {
//		time_cwati2 = hub_timer_get_elapsed_ms();
//		time_cwati[cwati_idx++ % 100] = time_cwati2 - time_cwati1;
		
		bool communicated = true;
		rows_received=last_spi_had_row?1:0;
		while(last_spi_had_row && rows_received < row_ct && communicated) {
			rows_received++;
			communicated = do_one_spi_communication(false);
		}
		uesb_start_rx();

	}
	return rows_received;
}

//void hub_timer_self_test(void) {	
//	hub_timer_start();
//	uint32_t before = hub_timer_get_elapsed_ms();
//	nrf_delay_us(100000);
//	uint32_t after = hub_timer_get_elapsed_ms();
//	hub_timer_stop();
//}

int main(void)
{
		uint32_t  last_sender;
		uint8_t		last_pkt_end_of_rr;
		uint32_t	time_now, time_diff;
		uint32_t	recvd_pkt_ts;
		uint32_t	time_to_talk_l = 0, time_to_talk_h = 0;
		bool			did_my_turn;

		/* We assume hts_packet_t is of different size from sth_packet_t */
		if (sizeof(hts_packet_t) == sizeof(sth_packet_t)) {
			while(1) {
				/* You must modify handle_uesb_events() to differentiate packet! */
			}
		}
		
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
		rfcbufInit(&rfcbuf);

		invalid_timestamp_crc = crc16_compute((uint8_t*)(&invalid_timestamp),sizeof(invalid_timestamp),0);
			
    // Start listening for incoming transmissions
		reinit_esb();
		
		// Initialize the slow clock used for recording timestamps.
		hub_timer_start_clock();
		hub_timer_init();

//		hub_timer_self_test(); //cwati todo remove me
		
		hub_timer_start();
		
		/* Initialize payload */
		next_payload = new_payload();
		
    while (true)
    {   
			// Diagnostics. Print on UART if available. If not, put it in the extra bytes
			// of SPI communication so that it can be picked up by a Beagle or something.
		
#ifdef HAS_CONSOLE
			if(control_button_pressed()) {
				print_log_entries(dump_log());
				char trace_msg[100];
				snprintf(trace_msg,99,"\r\n%d RF comms, %d mine, %d laggard, %d weird. %d rows, %f per req",
								requests_seen,my_requests_seen,laggard_requests_seen,nonsense_requests_seen,
								rows_sent, rows_sent*1.0/packets_generated
				);
				simple_uart_putstring((uint8_t*) trace_msg);
				snprintf(trace_msg,99,"\r\n avg excessive diff %d. buffer deficit %d",
							   sum_excessive_diff/ct_excessive_diff, cbufNum(&data_queue));
				simple_uart_putstring((uint8_t*) trace_msg);
				
			}
#endif
        
			// Let the CPU sleep until the next interrupt comes along
			//__WFE();
			//__SEV();
			//__WFE();


			if (my_sat_idx == UINT8_MAX) {
				while (!rfcbufIsEmpty(&rfcbuf)) {
					process_rx_packet(&last_pkt_end_of_rr, &recvd_pkt_ts);
				}
			} else {

				last_sender = UINT32_MAX;
				last_pkt_end_of_rr = UINT8_MAX;
				
				while(!rfcbufIsEmpty(&rfcbuf)) {
					last_sender = process_rx_packet(&last_pkt_end_of_rr, &recvd_pkt_ts);
				}
				
				if (last_sender != UINT32_MAX) {
					if ((last_sender == my_hub_id) && (my_sat_idx != INVALID_SAT_IDX)) {
						
						/* Boundaries of time that we can talk even if we haven't heard sat before us...
						* This way, round robin can continue even if a sat doesn't speak:
						* H Sat0 Sat1 N/A Sat3 Sat4 ...
						*            |3ms|
						* P.S: The 3ms is defined in RR_MAX_PERIOD.
						*/
						did_my_turn = false;

						time_now = hub_timer_get_elapsed_ms();

						time_to_talk_l = time_now + (my_sat_idx * RR_MAX_PERIOD);
						time_to_talk_h = time_to_talk_l + RR_MAX_PERIOD;
					}
					/* Talk if:
					 * 1) Last to talk is hub and I'm first in line
					 * 2) Last sat to talk is sat ID before me and it's not end of round robin
					 */
					if (((my_sat_idx == 0) && (last_sender == my_hub_id)) ||
							((my_sat_idx != 0) && (last_sender == sat_id_before_me) && (last_pkt_end_of_rr != 1)))
					{
						time_now = hub_timer_get_elapsed_ms();
						time_diff = time_now - recvd_pkt_ts;
						
						/* If it's stale packet we don't respond */
						if (time_diff < RR_MAX_TIME_DIFF_FOR_SENDING) {
							prepare_response();
//								
//							 /* to make sure listeners are ready to listen,
//								* i.e., they have switched from TX to RX */
//								nrf_delay_ms(1);

							send_response();
							did_my_turn = true;
							time_to_talk_l = 0;
						}
					}
				}
				
				if (did_my_turn == false) {
					time_now = hub_timer_get_elapsed_ms();

					if ((time_now >= time_to_talk_l) && (time_now <= time_to_talk_h)) {
								prepare_response();
							send_response();
							time_to_talk_l = 0;
							did_my_turn = true;
					}
					last_sender = UINT32_MAX;
				}
			}


			time_now = hub_timer_get_elapsed_ms();
			time_diff = time_now - last_talk_to_22f;

			if (time_diff >= heartbeat_period_to_22f) {
				communicate_over_spi(cur_num_of_sensors);
				time_now = hub_timer_get_elapsed_ms();
				last_talk_to_22f = time_now;
			}
			


//			if (time_diff >= heartbeat_period_to_22f) {
//				//cwati todo test
//				cnt = time_diff / heartbeat_period_to_22f;
//				/* Communicate to spi as a heartbeat */
//				communicate_over_spi(cnt * cur_num_of_sensors);
//				last_talk_to_22f = hub_timer_get_elapsed_ms();
//			}
    }

}
