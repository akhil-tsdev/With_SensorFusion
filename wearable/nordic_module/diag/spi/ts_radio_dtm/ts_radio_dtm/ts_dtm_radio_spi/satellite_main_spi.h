/*
 * Copyright (c) 2016 TuringSense
 * All rights reserved.
 *
 * satellite_main_spi.h
 *
 */
#ifndef __SATELLITE_MAIN_SPI_H__
#define __SATELLITE_MAIN_SPI_H__

extern uint8_t g_ts_radio_dtm_debug_level;
extern uint8_t g_ts_radio_dtm_cw;
extern uint8_t g_ts_radio_dtm_tx_package_type;
extern uint16_t g_ts_radio_dtm_tx_package_length;
extern uint8_t g_ts_radio_dtm_rx_package_type;
extern uint16_t g_ts_radio_dtm_rx_package_length;
extern uint8_t g_ts_radio_dtm_tx_channel_id;
extern uint8_t g_ts_radio_dtm_rx_channel_id;
extern uesb_tx_power_t g_ts_radio_dtm_power_level;
extern bool g_ts_radio_dtm_modulated;
extern uint8_t g_ts_radio_dtm_tx_packet[256];
extern uint8_t g_ts_radio_dtm_rx_packet[256];
extern uint16_t dtm_tx_interval_ms;

#endif /* __SATELLITE_MAIN_SPI_H__ */
