/*
 * Copyright © 2015 Turingsense, Inc.
 *
 * hub_main.h
 *
 */
#ifndef __HUB_MAIN_DTM_H__
#define __HUB_MAIN_DTM_H__

#include "common_types.h"
#include "dtm_common_types.h"
#include "uesb/micro_esb.h"

// TODO: in a world where multiple hubs might exist, an additional
// "hub id" or "hub serial number" might be needed. But it should be a
// separate variable.
/// Whenever the system uses a device ID, satellites start at 1.
/// The hub is 0 by convention.
#define DEVICE_ID 				0xabcd1

// Should probably be 15 in production. Or, perhaps, configured by Freescale.
#define TMP_NUM_OF_SAT 		5

// How many times RTC is sent out
#define RTC_SENDING_CNT 	10

// How many times LCP value is sent out
#define LCP_SENDING_CNT		10

// How many times COM/VAL  is sent out
#define COM_SENDING_CNT		15

/* DTM */
extern uint16_t last_received_crc16;
extern uint16_t dtm_cmd_type;
extern uint16_t dtm_freq;
extern uint16_t dtm_pkt_len;
extern uint16_t dtm_power;
extern uint16_t dtm_pkt_type ;
extern bool dtm_new_cmd;
extern uint16_t dtm_tx_interval_ms;

extern uint8_t g_ts_radio_dtm_debug_level;
extern uint8_t g_ts_radio_dtm_cw;
extern uint8_t g_ts_radio_dtm_tx_package_type;
extern uint16_t g_ts_radio_dtm_tx_package_length;
extern uint8_t g_ts_radio_dtm_rx_package_type;
extern uint16_t g_ts_radio_dtm_rx_package_length;
extern uint8_t g_ts_radio_dtm_tx_channel_id;
extern uint8_t g_ts_radio_dtm_rx_channel_id;
extern bool g_ts_radio_dtm_modulated;
extern uesb_tx_power_t g_ts_radio_dtm_power_level;
extern uint8_t g_ts_radio_dtm_tx_packet[256];
extern uint8_t g_ts_radio_dtm_rx_packet[256];

uesb_tx_power_t convert_dtm_power(uint16_t dtm_power);

#endif /* __HUB_MAIN_DTM_H__ */
