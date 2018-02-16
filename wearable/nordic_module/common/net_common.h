/*
 * Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * net_common.h
 *
 */
#ifndef __NET_COMMON_H__
#define __NET_COMMON_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define PACK_PRAGMA __packed
#include "common_types.h"

/* Round Robin parameters */
#define RR_MAX_QUIET_WAIT_DUMPING					10	/* ms.  Max time hub waits for packet during streaming */
#define RR_MAX_QUIET_WAIT_NODUMPING				RR_MAX_PERIOD*3	/* ms cwati max satelllites is now 3*/
#define RR_MAX_TIME_DIFF_FOR_SENDING			3		/* ms.  Expiration time of stale data */
#define RR_MAX_PERIOD											5		/* ms.  Allocating this much max time for sending */

/// Payload of a packet sent by the hub to its satellites.
typedef  struct {
	/// Whenever talking to a satellite, the hub echoes the CRC of the
	/// last packet it received from that satellite. That way, the
	/// satellite knows whether it has to retransmit.
	uint32_t last_seen_crc16[MAX_SATELLITES];
	/// The satellite who should reply to this packet. "Recipient" is a
	/// little misleading as in reality all satellites at least read the
	/// packet's command.
	uint32_t recipient;
	/// CRC for the whole payload (assuming |command_crc32| itself is 0).
	uint32_t command_crc32;
	/// COM COMMADD FOR SAT: [0] SAT_ID - 0 FOR ALL SAT; [1] COMMAND; [2-4] VALUE 1,2,3
	uint32_t command_toSat[MAX_SAT_COMMSTRUCT];
} hts_payload_t;


/// A shorter sensor record struct, to increase records per packet.
typedef __packed struct {
	uint8_t crc;	// Packet CRC
	uint8_t ts_diff;	// How many milliseconds since the last timestamp in the packet?
	uint8_t sensor_idx;	// System sensor index
	int16_t accel_x;	// Accelerometer's x-axis value
	int16_t accel_y;	// Accelerometer's y-axis value
	int16_t accel_z; 	// Accelerometer's z-axis value
	int16_t gyro_x;		// Gyroscope's x-axis value
	int16_t gyro_y;		// Gyroscope's y-axis value
	int16_t gyro_z;		// Gyroscope's z-axis value
#if ENABLE_MAG
	int16_t mag_x;		// Magnetometer's x-axis value
	int16_t mag_y;		// Magnetometer's y-axis value
	int16_t mag_z;		// Magnetometer's z-axis value
#endif
	int16_t quat_w;
	int16_t quat_x;
	int16_t quat_y;
	int16_t quat_z;
} reduced_sensor_record_t;

// To easily swap out these records for something else.
typedef reduced_sensor_record_t esb_sensor_record_t;

#if PIVOT_3_0
// This should be auto-calculated but I'm scared
#define SENSOR_RECORDS_PER_NET_PACKET MAX_SENSORS_IN_CS //cw test for 32-bit id
#else
// This should be auto-calculated but I'm scared
#if ENABLE_MAG
#define SENSOR_RECORDS_PER_NET_PACKET 6
#else
#define SENSOR_RECORDS_PER_NET_PACKET 5
#endif
#endif

/// Header data of a variable length satellite-to-hub payload.
/// Separate from the rest of the packet because these fields
/// are used to determine packet validity and length.
typedef  struct {
	// First timestamp of the data in this packet.
	uint32_t timestamp;
//	// CRC for the timestamp ONLY. All other integrity can be inferred.
//	// TODO: does this need to be a 32-bit CRC?
	uint16_t timestamp_crc;
	/* ACK of the last RTC ID received */
	uint16_t rtc_id_ack;
	// How many records of sensor data in the packet?
	uint8_t row_ct;
	// If this is the end of this round robin
	uint8_t end_of_roundrobin;
	//crc16 for NET_PACKET
  uint16_t received_crc16;
} sth_header_t;

/// Payload of a packet sent by a satellite to its hub.
typedef  struct {
	esb_sensor_record_t sensor_records[SENSOR_RECORDS_PER_NET_PACKET];
	sth_header_t header;
} sth_payload_t;


typedef  struct {
		/// Hardware ID
		uint32_t sender;	
	//crc16 for NET_PACKET
	uint16_t received_crc16;
} packet_header_t;

typedef  struct {
	packet_header_t header;
	sth_payload_t satellite_to_hub_payload;
} sth_packet_t;

typedef  struct {
	packet_header_t header;
	hts_payload_t hub_to_satellite_payload;
} hts_packet_t;

/// Converts a common_types sensor record into the compact RF format.
static inline esb_sensor_record_t pack_sensor_record(sensor_record_t sensor_record) {
	// Mostly just a "copy varA to varB" implementation for the 
	// moment, but that may change.
	esb_sensor_record_t ret = {
		.accel_x = sensor_record.accel_x,
		.accel_y = sensor_record.accel_y,
		.accel_z = sensor_record.accel_z,
		.gyro_x = sensor_record.gyro_x,
		.gyro_y = sensor_record.gyro_y,
		.gyro_z = sensor_record.gyro_z,
#if ENABLE_MAG
		.mag_x = sensor_record.mag_x,
		.mag_y = sensor_record.mag_y,
		.mag_z = sensor_record.mag_z,
#endif
		.quat_w = sensor_record.quat_w,
		.quat_x = sensor_record.quat_x,
		.quat_y = sensor_record.quat_y,
		.quat_z = sensor_record.quat_z
	};
	return ret;
}

typedef enum {
	hts_packet,
	sth_packet,
} net_packet_type_t;

#if HUB_NORDIC
#if HUB_SNIFFER
typedef void (*net_rx_callback)(uint32_t packet_len, uint32_t sender, int8_t rssi);
bool send_packet(hts_packet_t);

#else
/// Called when a packet comes in. Callee must do all data integrity checks.
typedef void (*net_rx_callback)(sth_packet_t);

/// Returns false if a synchronous error was detected. (True does not guarantee packet delivery.)
bool send_packet(hts_packet_t);
#endif /* HUB_SNIFFER */
#else
#if SAT_NORDIC
/// Called when a packet comes in. Callee must do all data integrity checks.
typedef void (*net_rx_callback)(void*, net_packet_type_t);

/// Returns false if a synchronous error was detected. (True does not guarantee packet delivery.)
bool send_packet(sth_packet_t);
#else
#error "You must define either HUB_NORDIC or SAT_NORDIC!"
#endif /* SAT_NORDIC */
#endif /* HUB_NORDIC */
/// Called when a packet could not be sent. Maybe.
typedef void (*net_nack_callback)(void);

/// Initializes RF communication. This *will* force a reinitialization
/// (losing any in-progress communication) if called after the first time.
void net_init(net_rx_callback, net_nack_callback, uint32_t tx_channel, uint32_t rx_channel);

/// Returns the size in bytes of a satellite-to-hub packet's variable length payload.
size_t sth_payload_size(sth_payload_t *payload_ptr);

uint32_t get_txchan_from_lcp(void);
uint32_t get_rxchan_from_lcp(void);
uint32_t get_next_lcp(void);
uint32_t get_lcp(void);
bool set_lcp(uint16_t new_lcp);
bool lcp_valid(uint16_t new_lcp);
void reinit_esb(void);

// Diagnostic stats
extern uint32_t diag_tx_attempt_ct;
extern uint32_t diag_tx_ack_ct;
extern uint32_t diag_tx_nack_ct;
extern uint32_t diag_rx_ct;
extern uint32_t diag_rx_after_ack_ct;
extern uint32_t diag_rx_after_nack_ct;
extern volatile bool rf_packet_received;

typedef enum {TX_FATE_UNKNOWN, TX_ACKED, TX_NACKED} tx_fate_t;
extern tx_fate_t fate_of_last_tx;

#endif /* __NET_COMMON_H__ */
