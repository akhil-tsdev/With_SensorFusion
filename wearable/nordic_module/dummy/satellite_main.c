/*
 * Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * satellite_main.c
 *
 */
#include <stdbool.h>
#include <stdint.h>
//#include <string.h>
//#include "app_util.h"
//#include "nrf.h"
//#include "uesb/micro_esb.h"
//#include "uesb/uesb_error_codes.h"

//#include "boards.h"
//#include "cbuf.h"
//#include "crc16.h"
//#include "crc32.h"
//#include "led_trace.h"
//#include "mpu9250_freq.h"
//#include "net_common.h"
//#include "nrf_delay.h"
//#include "nrf_gpio.h"
//#include "pivot3_0_poc.h"
//#include "rfcbuf.h"
//#include "satellite_main.h"
//#include "simple_uart.h"
//#include "ts_spi_master.h"

///// Starts at 1 for satellites. Must be different for EVERY satellite.
//uint32_t my_sat_id = INVALID_SAT_ID;
//uint8_t  my_sat_idx = INVALID_SAT_IDX;
//uint32_t my_hub_id;
//uint32_t sat_id_forCommand = 0;

//// Statistics for diagnosing problems.
//uint32_t requests_seen = 0;
//uint32_t my_requests_seen = 0;
//uint32_t laggard_requests_seen = 0;
//uint32_t nonsense_requests_seen = 0;
//uint32_t rows_sent = 0;
//uint32_t packets_generated = 0;
//uint32_t ct_excessive_diff = 0;
//uint32_t sum_excessive_diff = 0;
//uint32_t payload_crc_futile = 0;
//uint32_t communicationTestCounter = 0;

//// Keep track of number of satellites in the system
//uint32_t cur_num_of_sat = 6;
//uint32_t cur_sat_ids[MAX_SATELLITES] = {0};
//uint32_t num_of_sensors[MAX_SATELLITES] = {0};
//uint32_t sat_id_before_me = 0;
//bool     last_to_speak = false;			/* Need to keep track if we're last in this round robin */


//// We only want to bother communicating with the Freescale if one of us has
//// something to say to the other. We can't quite know when the Freescale
//// will have something to say, so we use an exponential back-off to approximate.
//#define ATOM_US 20
//#define MS_TO_TIME_OUT_ESB 2000
//#define CYCLES_TO_TIME_OUT_ESB (MS_TO_TIME_OUT_ESB * 1000 / ATOM_US)
//#define CYCLES_PER_SPI_ROUND 1 /*((DELAY_MS*1000)/ATOM_US)*/

//// These were added to debug the time travel issue found on 7 July.
//uint32_t last_timestamp_seen = 0;
//uint8_t next_jump_idx = 0;
//struct{uint32_t before;uint32_t after;} jumps_seen[16] = {{0,0}};

//bool last_spi_had_row = false;

//// Keep track of last RF communication
////uint32_t cycles_since_rx = 0;
//uint32_t heartbeat_period_to_22f = (8000 / ATOM_US);  /* 8 ms defalt */

//// When running on an eval stack, every press of Button 4
//// will make this function return true once. Used to trigger
//// UART logging.
//bool control_button_pressed(void);


//// Tracks synchronization with the hub. After being generated, a payload packet
//// is kept and re-sent until the hub replies with a matching CRC (confirming that
//// the packet has arrived safe and sound).
//uint16_t last_crc_rcvd_from_hub = 0;
//uint16_t expected_crc16 = 0;
//uint16_t previous_crc16 = 0;
//static sth_payload_t next_payload;
//static bool first_payload_generated = false;


//// System state, set according to commands from the hub.
//bool start_dumping = false;
//bool should_set_timestamp = false;
//uint32_t timestamp_to_set = INVALID_TIMESTAMP;
////bool must_calibrate = false;
//bool enable_command = false;
////bool must_change_lct_channel = false;
//uint16_t last_rtc_id = 0;
////SAT COMMAND MANAGEMENF CF
//uint32_t command_to_sat[MAX_SAT_COMMSTRUCT];

///// Contains readings from the Freescale, enqueued to be sent to the hub.
//cbuf_t data_queue;
//// If the queue gets too close to full, it'll hit a "red zone" and go
//// into a mode focused on draining values. Until it drains, SPI communication
//// will be limited; we will only call the Freescale to "refill" rows that were.
//// drained.
////static bool redzone_mode = false;
//#define CBUF_REDZONE_SIZE ((CB_SIZE)*2/3)
//#define CBUF_IS_REDZONE(cbuf) (CBUF_REDZONE_SIZE <= cbufNum(cbuf))
//#define CBUF_GREENZONE_SIZE 25


//// Packets received from the common RF code are stored here, and a flag
//// is set for the main loop to pick it up. This is because the event handler
//// has to be as short as possible - I've observed repeatedly that long event
//// handlers cause ESB library to break in hard-to-debug ways. Still not sure
//// why.
////volatile bool packet_waiting = false;
////volatile hts_packet_t hts_packet_to_process;
////volatile sth_packet_t sth_packet_to_process;
////volatile net_packet_type_t last_pkt_recvd;
//static uint32_t cwati_from_hub = 0, cwati_from_sat = 0,
//	cwati_valid_from_hub = 0, cwati_valid_from_sat = 0,
//	cwati_turn_from_hub = 0, cwati_turn_from_sat = 0, cwati_missed_turn_from_sat = 0;
//rfcbuf_t rfcbuf;

//void reinit_esb() {
//	net_init(satellite_rx_handler, 0, get_txchan_from_lcp(), get_rxchan_from_lcp());
//}

////cf: 20160714 - for testing communication
//int16_t operativeMode;
//void setCalculatedFrame(sensor_record_t * data){
//	  operativeMode=data->quat_w/CPI_QUATERNION16_BIAS_MULTIPLIER;
//		if(operativeMode==CPI_QUATERNION16_COUNTER_COMMAND){
//			
//			if(data->accel_x > 0){
//				communicationTestCounter++;
//			}
//			else{
//				communicationTestCounter=0;
//			}
//			
//			data->accel_z = (int16_t)(communicationTestCounter%CPI_QUATERNION16_COUNTER_MAXVALU);
//			data->gyro_x =  0; //(int16_t)(communicationTestCounter[1]%CPI_QUATERNION16_COUNTER_MAXVALU);
//		}else if(operativeMode==CPI_QUATERNION16_VERSION_COMMAND){
//			data->gyro_x = (int16_t)REVIS_VERSION_SATNRD;
//		}
//}

///// Builds a satellite-to-hub packet payload using data popped from the queue.
///// You MUST save the result someplace safe, else the rows will be lost!
//sth_payload_t new_payload(void) {
//	// TODO: This function could probably stand to be optimized a bit.
//	// Since it's called just before writing a response to the hub, it's on
//	// the critical path.
//	FLIP_TRACE(3);
//	first_payload_generated = true;
//	
//	// Initial/default state: a packet with no rows.
//	sth_payload_t payload = {0};
//	sat_cbuf_packet_t data;
//	packets_generated++;
//	uint32_t packet_timestamp = INVALID_TIMESTAMP;
//	payload.header.row_ct = 0;
//	
//	if(start_dumping && cbufPop(&data_queue,&data) == E_OK) {
//		// At least one row exists to be sent. Write whole-packet metadata.
//    setCalculatedFrame(&(data.data));
//		payload.sensor_records[0] = pack_sensor_record(data.data);
//		packet_timestamp = data.timestamp;
//		uint32_t last_timestamp = packet_timestamp;
//		rows_sent++;
//		payload.header.row_ct++;
//		for(int i=1;i<SENSOR_RECORDS_PER_NET_PACKET;++i) {
//			// For each additional row available to send...
//			if(cbufPeek(&data_queue,&data) == E_OK) {
//				// Rows after the first in a packet don't have a full timestamp.
//				// Instead, we send only the diff between the previous timestamp
//				// and this one. Normally packets should be something very close to
//				// 10ms apart.
//				int32_t ts_diff = data.timestamp - (last_timestamp);
//				if(ts_diff < 0 || ts_diff > 255) {
//					// Anomaly. This row is more than 1/4 second past the previous. Or
//					// has jumped BACKWARD in time! Either way, there's not enough room
//					// in a ts_diff to describe that. End the packet here, and send this
//					// new and surprising row as the first timestamp of the next packet.
//					int32_t excessive_diff = (data.timestamp - packet_timestamp);
//					if(excessive_diff > 0 && excessive_diff < 10000)
//					{
//						ct_excessive_diff++;
//						sum_excessive_diff += (data.timestamp - packet_timestamp);
//					}
//					
//					// Make sure the packet is properly terminated.
//					// TODO: might not be necessary? We do an = {0} up top.
//					memset(&(payload.sensor_records[i]), 0, sizeof(esb_sensor_record_t));
//					break;
//				}
//				// Row looks good to include in this packet.
//				cbufPop(&data_queue,&data);
//				setCalculatedFrame(&(data.data));
//				payload.sensor_records[i] = pack_sensor_record(data.data);
//				payload.sensor_records[i].ts_diff = ts_diff;
//				last_timestamp = data.timestamp;
//				rows_sent++;
//				payload.header.row_ct++;
//			}
//			else {
//				// TODO: might not be necessary? We do an = {0} up top.
//				memset(&(payload.sensor_records[i]), 0, sizeof(esb_sensor_record_t));
//			}
//		}
//	}
//	payload.header.timestamp = packet_timestamp;
//	payload.header.timestamp_crc = crc16_compute((uint8_t*)(&packet_timestamp),sizeof(packet_timestamp),0);
//	payload.header.rtc_id_ack = last_rtc_id;

//	expected_crc16 = crc16_compute((uint8_t*)(&payload),sizeof(payload),0);
//	return payload;
//}

//// TODO: this is an out-of-band return value. Find a better way to pass it
//// back to the main loop.
//uint8_t rows_replied = 0;

////void save_last_hub_crc(uint16_t last_crc) {
////	if (my_sat_idx != INVALID_SAT_IDX) {
////		last_crc_rcvd_from_hub = last_crc;
////	}
////}

///// Checks whether the hub has received the current response payload correctly
///// If so, loads up a new response payload to be sent.
//void prepare_response() {
//	uint32_t rows_sent_before;
//	
//	rows_replied = 0;
//	if (last_crc_rcvd_from_hub == expected_crc16 || !first_payload_generated) {
//		previous_crc16 = expected_crc16;
//		rows_sent_before = rows_sent;
//		//next_payload = new_payload();
//		rows_replied = rows_sent - rows_sent_before;
//	} else if(last_crc_rcvd_from_hub == previous_crc16) {
//		laggard_requests_seen++;
//	} else {
//		nonsense_requests_seen++;
//	}
//}

///// Replies to the hub with the currently prepared data payload. Blocks until sending is over.
//void send_response(void) {
//	sth_packet_t sth_packet;
//	
//	/* If we haven't got our valid Satellite ID, refrain from sending */
//	if (my_sat_id != INVALID_SAT_ID) {
//		sth_packet.header.sender = my_sat_id;
//		sat_id_forCommand = my_sat_id;
//	} else {
//		return;
//	}
//	
//	//CF:NOTE SPI COUNTER THERE(?)
//	/*if(operativeMode==CPI_QUATERNION16_COUNTER_COMMAND){
//		for(int i = 0; i < SENSOR_RECORDS_PER_NET_PACKET; i++)
//			if(next_payload.sensor_records[i].accel_x>0){
//				communicationTestCounter[1]++;
//				next_payload.sensor_records[i].gyro_x = (int16_t)(communicationTestCounter[1]%CPI_QUATERNION16_COUNTER_MAXVALU);
//			}
//	}*/
//	
//	sth_packet.satellite_to_hub_payload = next_payload;
//	
//	/* cwati TODO in pivot YOGA, if we want to send more than one packet,
//	 * then we only set end_of_roundrobin = true on the very last packet.
//	 */
//	if (last_to_speak) {
//		sth_packet.satellite_to_hub_payload.header.end_of_roundrobin = true;
//	} else {
//		sth_packet.satellite_to_hub_payload.header.end_of_roundrobin = false;		
//	}
//	bool result = send_packet(sth_packet);
////	last_to_speak = false;
//	
//	int timeout_10us = 0;
//	while(fate_of_last_tx == TX_FATE_UNKNOWN) {
//		nrf_delay_us(10);
//		timeout_10us++;
//		if(timeout_10us > 100000) {
//			// It's been 1000ms. Assume something went wrong. 
//			
//			// Reset the RF library since we can't know if state is still good.
//			reinit_esb();			
//			break;
//		}
//	}

//}

//uint32_t cwati_pop_discard = 0;

//void satellite_rx_handler(void* packet, net_packet_type_t packet_type) {
//	uint32_t sender_id;
//	rx_packet_t incoming_packet;
//	
////	cycles_since_rx = 0;
//	
//	if (packet_type == hts_packet) {
////		cwati_from_hub++;
//		sender_id = *(uint32_t*)(packet);
//		/* Save time here by ignoring unknown packet right away. */
//		if (sender_id != my_hub_id) {
//			return;
//		}
//		incoming_packet.packet.hts = *(hts_packet_t*)(packet);
//		incoming_packet.packet_type = hts_packet;
//		if (rfcbufIsFull(&rfcbuf)) {
//			rfcbufPopDiscard(&rfcbuf);
//			cwati_pop_discard++;
//		}
//		rfcbufPush(&rfcbuf, incoming_packet);
//	} else if (packet_type == sth_packet) {
////		cwati_from_sat++;

//		/* Quick Store *
//		 * WARNING!!! We only store information we're interested in in order to save time.
//		 * CRC value inside the packet is not stored!!! If we later decide to check CRC
//		 * update this cbuf!! */
//		incoming_packet.packet.sth.satellite_to_hub_payload.header.end_of_roundrobin = \
//			((sth_packet_t*)packet)->satellite_to_hub_payload.header.end_of_roundrobin;
//		incoming_packet.packet_type = sth_packet;
//		incoming_packet.packet.sth.header.sender = ((sth_packet_t*)packet)->header.sender;
//		
//		if (rfcbufIsFull(&rfcbuf)) {
//			rfcbufPopDiscard(&rfcbuf);
//		}
//		rfcbufPushSatPkt(&rfcbuf, incoming_packet);
////		sender_id = *(uint32_t*)(packet);			
////		if (sender_id != sat_id_before_me) {
////			return;
////		} else {
////			/* WARNING!!! We only store information we're interested in in order to save time.
////			 * CRC value inside the packet is not stored!!! If we later decide to check CRC
////			 * update this cbuf!! */
////			incoming_packet.packet.sth.satellite_to_hub_payload.header.end_of_roundrobin = \
////				((sth_packet_t*)packet)->satellite_to_hub_payload.header.end_of_roundrobin;
////			incoming_packet.packet_type = sth_packet;
////			incoming_packet.packet.sth.header.sender = ((sth_packet_t*)packet)->header.sender;
////			
////			if (rfcbufIsFull(&rfcbuf)) {
////				rfcbufPopDiscard(&rfcbuf);
////			}
////			rfcbufPushSatPkt(&rfcbuf, incoming_packet);
////		}
//	}
//}

/////// Extra validation to be done on the claim that a packet is waiting to be
/////// processed. Clears packet_waiting if false.
////bool waiting_rx_packet_valid(void) {
////	hts_payload_t hts_payload;
////	uint32_t payload_timestamp;
////	uint16_t recv_crc;
////	uint32_t sent_crc32 ;
////	uint32_t calculated_crc32;
////	uint16_t timestamp_crc;
////	
////	packet_waiting = false;
////	
////	// Check that the control packet hasn't been corrupted.
////	if (last_pkt_recvd == hts_packet) {
////		hts_payload = hts_packet_to_process.hub_to_satellite_payload;
////		sent_crc32 = hts_payload.command_crc32;
////		hts_payload.command_crc32 = 0;
////		calculated_crc32 = crc32(0xFFFFFFFF, &hts_payload,
////																	 sizeof(hts_payload));
////		
////		packet_waiting = calculated_crc32 == sent_crc32;
////		
////	} else if (last_pkt_recvd == sth_packet) {	
////		recv_crc = sth_packet_to_process.satellite_to_hub_payload.header.timestamp_crc;
////		payload_timestamp = sth_packet_to_process.satellite_to_hub_payload.header.timestamp;

////		timestamp_crc = crc16_compute((uint8_t*)(&payload_timestamp),sizeof(payload_timestamp),0);
////		
////		packet_waiting = (recv_crc == timestamp_crc);
////	}

////	return packet_waiting;
////}

//static void update_sat_before_me() {
//	uint8_t sat_idx;

//	/* case for our id as sat_idx 0 */
//	if (cur_sat_ids[0] == my_sat_id) {
//		sat_id_before_me = cur_sat_ids[cur_num_of_sat - 1];
//		return;
//	}
//	
//	for (sat_idx = 1; sat_idx < cur_num_of_sat; sat_idx++) {
//		if (cur_sat_ids[sat_idx] == my_sat_id) {
//			sat_id_before_me = cur_sat_ids[sat_idx - 1];
//		}
//	}
//	return;
//}

//static void set_sat_idx(uint32_t command_toSat[MAX_SAT_COMMSTRUCT]) {
////	uint32_t command_toSat[MAX_SAT_COMMSTRUCT];
////	
////	for (uint8_t qq; qq < MAX_SAT_COMMSTRUCT; qq++) {
////		command_toSat[qq] = command[qq];
////	}

//	cur_num_of_sat = command_toSat[SAT_COMMSTRUCT_VALZ3];

//	if (command_toSat[SAT_COMMSTRUCT_VALX1] < cur_num_of_sat) {
//		cur_sat_ids[command_toSat[SAT_COMMSTRUCT_VALX1]] = command_toSat[SAT_COMMSTRUCT_VALY2];
//		
//		/* Update our order  of turn to speak to hub */
//		if (command_toSat[SAT_COMMSTRUCT_VALY2] == my_sat_id) {
//			my_sat_idx = command_toSat[SAT_COMMSTRUCT_VALX1];
//		}
//	} else {
//		cur_sat_ids[command_toSat[SAT_COMMSTRUCT_VALX1]] = INVALID_SAT_ID;
//	}

//	for (int cc = cur_num_of_sat; cc < MAX_SATELLITES;cc++)
//	{
//		cur_sat_ids[cc] = 0;
//	}
//	
//	update_sat_before_me();
//	
//	if (cur_num_of_sat > 1) {
//		if ((cur_num_of_sat - 1) == my_sat_idx) {
//			last_to_speak = true;
//		} else {
//			last_to_speak = false;
//		}
//	} else {
//		last_to_speak = true;
//	}
//	
//}

//void process_command(uint32_t commandFromHub[MAX_SAT_COMMSTRUCT]) {
//	uint32_t freq_divider, freq;
//	
//	// GL - wait for a command to be shipped before listening to others commands
//	if (enable_command == false) 
//	{
//	enable_command = (my_sat_id == commandFromHub[SAT_COMMSTRUCT_SATID] || 
//												 sat_id_forCommand == commandFromHub[SAT_COMMSTRUCT_SATID] ||
//												 commandFromHub[SAT_COMMSTRUCT_SATID] == 0); //cf
//	}
//	if((my_sat_id != INVALID_SAT_ID) && !sat_id_forCommand) {
//		sat_id_forCommand=my_sat_id;
//	}
//		
//	//sat command management cf
//	if(enable_command) {

//	  if(commandFromHub[SAT_COMMSTRUCT_COMMA] == SAT_COMM_WIF_SETRTC) {
//	  	should_set_timestamp = true;
//	  	timestamp_to_set = commandFromHub[SAT_COMMSTRUCT_VALX1];
//		  last_rtc_id = commandFromHub[SAT_COMMSTRUCT_VALY2];
//		  if (timestamp_to_set == 0) {
//		  	cbufInit(&data_queue);
//		  }
//	  }
//	
////		must_calibrate = true;
//		uint8_t cnt;
//		for (cnt = 0; cnt < MAX_SAT_COMMSTRUCT; cnt++) {
//				command_to_sat[cnt] = commandFromHub[cnt];
//		}
//		
//		
//		
//		//cf 20160518 - change communication channel
//		if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_SETBTCOMCH || command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_WIF_SETLCP){
//			command_to_sat[SAT_COMMSTRUCT_VALY2] = set_lcp(command_to_sat[SAT_COMMSTRUCT_VALX1]); //the result is sent to the SAT's 22F
//			//if(command_to_sat[SAT_COMMSTRUCT_VALY2])
//				//reinit_esb();
//		}
//		
//		//cf 20160714 - enable counter sending mode, for communication testing
//		if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_TESTCOMSTC){
//			communicationTestCounter=0;
//		}

//		//cf 20161014 - MANAGEMENT OF START DUMPING WITH SAT COMMAND
//		if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_START_DUMP){
//				start_dumping = true;
//		}
//		
//		//cf 20161014 - MANAGEMENT OF STOP DUMPING WITH SAT COMMAND
//		if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_STOP_DUMPI){
//				start_dumping = false;
//				/* Empty cbuf FAST_DRAIN */
//        cbufInit (&data_queue);
//		}
//		
//		//cw 20170614 - SET NUMBER OF SATS IN THE SYSTEM
//		if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_WIF_SETSAT){
//			if (commandFromHub[SAT_COMMSTRUCT_VALZ3] != 0) {
//				set_sat_idx(command_to_sat);
//			}
//		}

//		//cw 20170621 - UPDATE PERIOD TO TALK TO SATELLITE 22F
//		if(command_to_sat[SAT_COMMSTRUCT_COMMA] == SAT_COMM_FRQDIVIDER){
//			if (commandFromHub[SAT_COMMSTRUCT_VALX1] != 0) {
//				freq_divider = commandFromHub[SAT_COMMSTRUCT_VALX1];
//				
//				/* Find the Hz */
//				freq = TS_GYRO_FREQ / freq_divider;
//				
//				uint32_t ms;
//				if (freq <= 100) {
//					/* If 100 Hz or slower, we can talk every 8 ms */
//					heartbeat_period_to_22f = (8000 / ATOM_US);
//				} else {
//					/* If frequency is higher, we need to talk more often */
//					ms = 1000 /* 1s */ / freq;  /* in ms */
//					heartbeat_period_to_22f = (ms * 1000 / ATOM_US); /* microsecond */
//					heartbeat_period_to_22f = (heartbeat_period_to_22f * 8) / 10; /* Reduce 20% */
//				}
//			}
//		}		
//	}
//}

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

//uint32_t process_rx_packet(void) {
////	hts_payload_t hts_payload;
//	bool					turn_to_speak = false;
////	uint8_t				first_sat_idx;
//	uint32_t 			payload_timestamp;
//	uint16_t 			recv_crc;
//	uint32_t 			sent_crc32 ;
//	uint32_t 			calculated_crc32;
//	uint16_t 			timestamp_crc;
//	bool					valid_packet = false;
//	rx_packet_t		packet_to_process;
//	uint32_t			sender = UINT32_MAX;
//	
//	//cwati todo there should be a mutex mechanism here, in case RF callback modifies
//	//at the same time.
//	if (rfcbufPop(&rfcbuf, &packet_to_process) != E_OK) {
//		return sender;
//	}
//	
//	/* We got the packet from cbuf, now verify if packet is valid */
//	if (packet_to_process.packet_type == sth_packet) {
//		sender = packet_to_process.packet.sth.header.sender;
//		return sender;
//	} else if (packet_to_process.packet_type == hts_packet) {
//		sender = packet_to_process.packet.hts.header.sender;
//		sent_crc32 = packet_to_process.packet.hts.hub_to_satellite_payload.command_crc32;
//		packet_to_process.packet.hts.hub_to_satellite_payload.command_crc32 = 0;
//		calculated_crc32 = crc32(0xFFFFFFFF, &packet_to_process.packet.hts.hub_to_satellite_payload,
//																	 sizeof(packet_to_process.packet.hts.hub_to_satellite_payload));
//		
//		valid_packet = calculated_crc32 == sent_crc32;
//	}		
////	} else if (packet_to_process.packet_type == sth_packet) {	
////		recv_crc = packet_to_process.packet.sth.satellite_to_hub_payload.header.timestamp_crc;
////		payload_timestamp = packet_to_process.packet.sth.satellite_to_hub_payload.header.timestamp;

////		timestamp_crc = crc16_compute((uint8_t*)(&payload_timestamp),sizeof(payload_timestamp),0);
////		
////		valid_packet = (recv_crc == timestamp_crc);
////	}

//	/* If there is no valid packet to process, just return */
//	if (valid_packet == FALSE) {
//		return sender;
//	}
//	
////	requests_seen++;
////	FLIP_TRACE(1);
////	
//	/* Sat can talk if:
//		1) Asked explicitly by the hub
//	*/
//	if (packet_to_process.packet_type == hts_packet) {
//		cwati_valid_from_hub++;		
//		
//		/* Save last hub CRC */
//		if (my_sat_idx != INVALID_SAT_IDX) {
//			last_crc_rcvd_from_hub = packet_to_process.packet.hts.hub_to_satellite_payload.last_seen_crc16[my_sat_idx];
//		}
//		process_command(&packet_to_process.packet.hts.hub_to_satellite_payload.command_toSat[0]);
//		if (packet_to_process.packet.hts.hub_to_satellite_payload.recipient == my_sat_id) {
//			turn_to_speak = true;
//			cwati_turn_from_hub++;
//		}
//	}
//	
//	return sender;
////		} else {
////			
////			/* Keep track if we're last in this round robin */
////			first_sat_idx = get_sat_idx(hts_payload.recipient);
////			
////			if (first_sat_idx == INVALID_SAT_IDX) {
////				return;
////			}
////			
////			if (cur_num_of_sat > 1) {
////				
////				/* 
////				 * For example, we have sat idx: 0, 1, 2, 3, 4.
////         * If first sat to talk is sat 0, then we will talk last if we are sat_idx 4.
////				 * If first sat to talk is something other than zero, then we will talk last
////				 *    if our index is (first_sat_idx_to_talk - 1).
////				 */
////				if (first_sat_idx == 0) {
////					if ((cur_num_of_sat - 1) == my_sat_idx) {
////						last_to_speak = true;
////					}
////				} else {
////					if ((first_sat_idx - 1) == my_sat_idx) {
////						last_to_speak = true;
////					}
////				}
////			}
////		}
////	} else if (packet_to_process.packet_type == sth_packet) {
////		cwati_valid_from_sat++;		

////		/* Sat can talk if:
////		 2) The preceding satellite has spoken and the packet doesn't contain 
////		    END_OF_ROUND_ROBIN_PACKET.
////	   */
////		if ((packet_to_process.packet.sth.header.sender == sat_id_before_me) && \
////			(packet_to_process.packet.sth.satellite_to_hub_payload.header.end_of_roundrobin != true)) {
////			turn_to_speak = true;
////			cwati_turn_from_sat++;
////		} else {
////			cwati_missed_turn_from_sat++;
////		}
////	}

////	if (turn_to_speak) {
////		my_requests_seen++;
//////		nrf_delay_ms(5); //cwati todo test.  give time for hub to process packet.
////		prepare_response();
////		send_response();
////	}
//}

//static void ui_init(void)
//{
//	uint32_t buttons[] = BUTTONS_LIST;
//	uint32_t leds[] = LEDS_LIST;
//	uint32_t i;

//	for (i = 0; i < BUTTONS_NUMBER; i++)
//	{
//			nrf_gpio_cfg_input(buttons[i],NRF_GPIO_PIN_PULLUP);
//	}
//	for (i = 0; i < LEDS_NUMBER; i++)
//	{
//			nrf_gpio_cfg_output(leds[i]);
//	}
//	//simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
//}

//bool control_button_pressed(void) {
//	uint32_t buttons[] = BUTTONS_LIST;
//	static bool control_button_was_down = false;
//	bool control_button_is_down = !nrf_gpio_pin_read(buttons[3]);
//	bool press = (!control_button_was_down && control_button_is_down);
//	control_button_was_down = control_button_is_down;
//	return press;	
//}

//void spi_received(sat_to_nordic_t packet) {
//	static bool first_time = true;
//	sat_cbuf_packet_t cbuf_packet;
//	uint16_t crc16, recv_crc16;
//		
//	/* Calculate CRC16 */
//	recv_crc16 = packet.crc16;
//	packet.crc16 = 0;
//	crc16 = crc16_compute((uint8_t*)(&packet), sizeof(packet), 0);
//		
//	if (recv_crc16 != crc16) {
//		last_spi_had_row = false;
//		nrf_delay_us(100000);
//		return;
//	}
//		my_sat_id = packet.sat_id;
//		my_hub_id = packet.hub_id;
//	
//	// sat and hub id
//		if (first_time) {

//	
//		first_time = false;
//	}
//	
//	
//	if(lcp_valid(packet.nrdChannel))
//		set_lcp(packet.nrdChannel);
//	
//	
//	
//	if(packet.timestamp == INVALID_TIMESTAMP) {
//		last_spi_had_row = false;
//		return;
//	}

//	last_spi_had_row = true;

//	static uint32_t cwati_cbuf_discard_cnt = 0;
//	/* Discard old data */
//	if(cbufIsFull(&data_queue)) {
//		cwati_cbuf_discard_cnt++;
//		cbufPopDiscard(&data_queue);
//	}
//	
//	/* The SPI packet received contains satellite ID,
//	 * but we won't store it into our cbuf */
//	cbuf_packet.data = packet.data;
//	cbuf_packet.timestamp = packet.timestamp;
//	
//	cbufPush(&data_queue,cbuf_packet);
//}

///// Decide what command to send the Freescale based on our own state.
//nordic_to_sat_t command_for_spi(void) {
//	nordic_to_sat_t command = {.sat_command[SAT_COMMSTRUCT_SATID] = 0, .sat_command[SAT_COMMSTRUCT_VALX1] = INVALID_TIMESTAMP};
////	bool sensor_should_be_quiet = CBUF_IS_REDZONE(&data_queue);
////	
////	if (sensor_should_be_quiet || (sending_enabled == false)) {
////		command.command |= SAT_SPI_WAIT;
////	} 
// 
//	//GL - enable_command is the old must_calibrate
//	if (enable_command) {

//		command.sat_command[SAT_COMMSTRUCT_SATID] = command_to_sat[SAT_COMMSTRUCT_SATID];//THIS POSITION ARE FREE - BEFORE CONTAIN THE 32BIT OLD HUB COMMAND
//		command.sat_command[SAT_COMMSTRUCT_COMMA] = command_to_sat[SAT_COMMSTRUCT_COMMA];
//		command.sat_command[SAT_COMMSTRUCT_VALX1] = command_to_sat[SAT_COMMSTRUCT_VALX1];
//		command.sat_command[SAT_COMMSTRUCT_VALY2] = command_to_sat[SAT_COMMSTRUCT_VALY2];
//		command.sat_command[SAT_COMMSTRUCT_VALZ3] = command_to_sat[SAT_COMMSTRUCT_VALZ3];

//		enable_command=false;
//	}
//	
//	if (should_set_timestamp) {
//		command.sat_command[SAT_COMMSTRUCT_COMMA] = SAT_COMM_WIF_SETRTC;
//		command.sat_command[SAT_COMMSTRUCT_VALX1] = timestamp_to_set;
//		should_set_timestamp = false;
//	}
////	
////	//cf automatic LCT channel change requested to the 22F
////	if(0){//must_change_lct_channel){
////		command.sat_command[SAT_COMMSTRUCT_SATID] = 0;
////		command.sat_command[SAT_COMMSTRUCT_COMMA] = SAT_COMM_SETBTCOMAU;
////		command.sat_command[SAT_COMMSTRUCT_VALX1] = MAX_LCP;
////		
////		must_change_lct_channel=false;
////	}
//	//cf 23/04/2016
//	/*
//	if (must_set_lcp) {
//		command.sat_command[SAT_COMMSTRUCT_SATID] |= (SAT_SPI_SET_LCP);
//		command.sat_command[SAT_COMMSTRUCT_COMMA] = command_to_sat[SAT_COMMSTRUCT_COMMA];
//		command.sat_command[SAT_COMMSTRUCT_VALX1] = command_to_sat[SAT_COMMSTRUCT_VALX1];
//		command.sat_command[SAT_COMMSTRUCT_VALY2] = command_to_sat[SAT_COMMSTRUCT_VALY2];
//		command.sat_command[SAT_COMMSTRUCT_VALZ3] = command_to_sat[SAT_COMMSTRUCT_VALZ3];
//		
//		must_set_lcp=false;
//	}*/
//	
//	int lcp_tmp=get_lcp();
//	
//	if (command.sat_command[SAT_COMMSTRUCT_COMMA] != 0) {
//		command.sat_command[SAT_COMMSTRUCT_SATID] = command.sat_command[SAT_COMMSTRUCT_SATID];
//	}
//	
//	/*
//	if (start_dumping) {
//		command.sat_command[SAT_COMMSTRUCT_SATID] = SAT_COMM_START_DUMP;
//	} else {
//		command.sat_command[SAT_COMMSTRUCT_SATID] = SAT_COMM_STOP_DUMPI;
//	}
//	*/
//	return command;
//}

///// Blocks until communication is done. Returns whether communication was successful.
///// Only one of the SPI master library and the ESB library can be active at once.
///// If you're using ESB, set rf_active, so that it can be turned off during the
///// communication. If rf_active=true and communication was successful, when this
///// returns RF will be off. Otherwise, it should be unchanged from its original state.
//bool do_one_spi_communication(bool rf_active) {
//	nordic_to_sat_t command = command_for_spi();
////	if(CBUF_IS_REDZONE(&data_queue)) {
////		redzone_mode = true;
////	}

//	/* [PV-900] You must stop RX when transmitting via SPI for now */
//	if(rf_active) {
//		if(uesb_stop_rx() != UESB_SUCCESS) {
//			return false;
//		}
//	}
//	bool was_able_to_communicate = spi_send_command(command,spi_received);
//	if(!was_able_to_communicate) {
//		if(rf_active) {
//			uesb_start_rx();
//		}
//		return false;
//	}

//	return true;
//}

///// Try to communicate enough to read row_ct rows. Returns the number
///// of rows received. This function assumes that RF is turned on, and
///// does the right thing to handle that: turns it off before communicating,
///// and turns it back on before it returns.
//int communicate_over_spi(int row_ct) {
//	int rows_received = 0;

//	if(do_one_spi_communication(true)) {
//		bool communicated = true;
//		rows_received=last_spi_had_row?1:0;
//		while(last_spi_had_row && rows_received < row_ct && communicated) {
//			rows_received++;
//			communicated = do_one_spi_communication(false);
//		}
//		uesb_start_rx();

//	}
//	return rows_received;
//}

int main(void)
{
	while(1) {}
}
