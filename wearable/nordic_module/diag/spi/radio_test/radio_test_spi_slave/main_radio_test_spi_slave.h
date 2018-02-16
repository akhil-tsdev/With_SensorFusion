/* Copyright (c) 2016 Turingsense Inc. All Rights Reserved.
 *
 * main_radio_test_spi_slave.h
 *
 */

#ifndef __MAIN_RADIO_TEST_SPI_SLAVE_H__
#define __MAIN_RADIO_TEST_SPI_SLAVE_H__

extern uint8_t mode_;
extern uint8_t txpower_;
extern uint8_t channel_start_;
extern uint8_t channel_end_;
extern uint8_t delayms_;

void process_command(void);
void prep_tx_crc16(void);
void process_command(void);
uint8_t get_tx_power(void);
uint8_t get_datarate(void);
#endif /* __MAIN_RADIO_TEST_SPI_SLAVE_H__ */
