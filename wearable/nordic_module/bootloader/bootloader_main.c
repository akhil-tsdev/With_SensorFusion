/*
 * Copyright (c) 2016 TuringSense
 * All rights reserved.
 *
 * bootloader_main.c
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "app_util.h"
#include "nrf.h"

#include "crc32.h"
#include "boards.h"
#include "nrf_gpio.h"
//#include "simple_uart.h"
//#include "fifo.h"

#include "crc16.h"
#include "nrf_delay.h"

#include "led_trace.h"

#include "app_error.h"
#include "app_util_platform.h"
#include "crc16.h"
#include "nrf_delay.h"
#include "bsp.h"
#include "app_timer.h"
//#include "app_gpiote.h"
#include "nordic_common.h"
#include "bootloader_spi_types.h"

#include "bootloader_main.h"
#include "bootloader_slave_spi.h"
#include "hub_timer.h"
#include "nordic_bootloader.h"

#include "nordic_ts_flash.h"

#if EVAL_BOARD
#include "hub_eval_board.h"
const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;
#endif

// variable to control bootloader mode
static uint8_t nordic_bl_mode = NORDIC_BOOTLOADER_MODE_INVALID;
static uint32_t last_bl_line;
static uint16_t last_crc16_received;
uint32_t bootloader_active = 1;
static bool first_time = true;
static bool is_in_bl_mode = true;

/************************************************
	SPI transfer protocol for bootloader:
1. NORDIC_BOOTLOADER_MODE_INVALID
	This is when nordic is being initialized.
	NORDIC_BOOTLOADER_READY signal is transferred from
	nordic to 22F
2. NORDIC_BOOTLOADER_MODE_START
	This is when READY signal has been sent successfully
	At this point, nordic will wait for any command from 22F
3. NORDIC_BOOTLOADER_MODE_LOAD
	This is after 22F send NORDIC_BOOTLOADER_TX signal
	During this mode, 22F and nordic will keep exchanging
	data and acknowledgement with the following flow:
	22F -> NORDIC_BOOTLOADER_TX + data -> nordic
	22F <- NORDIC_BOOTLOADER_ACK <- nordic (once write is completed)
	22F -> NORDIC_BOOTLOADER_ACK -> nordic
	22F <- NORDIC_BOOTLOADER_READY <- nordic
	... continue for the next data ...
4. NORDIC_BOOTLOADER_MODE_ERASE
	This is after 22F send NORDIC_BOOTLOADER_ERASE signal
5. NORDIC_BOOTLOADER_MODE_STOP
	This is after 22F send NORDIC_BOOTLOADER_DONE signal
*************************************************/
/***************** SPI stuff *******************/
#define STALL_REINIT_THRESHOLD 3
#define TRANSFER_TIMEOUT_US 5000
#define SPI_MASTER_HW SPI_MASTER_0
#define NUM_BOOT_SIGNAL 4
#define BOOT_SIGNAL_PIN 18
#define ATOM_US 20
#define MS_TO_WAIT_FOR_BOOT_SIGNAL NORDIC_BL_WAIT_MS /* cwati change me */
#define CYCLES_TO_WAIT_FOR_BOOT (MS_TO_WAIT_FOR_BOOT_SIGNAL / ATOM_US)

uint32_t mem_address = APPLICATION_BASE_ADDRESS;
// SPI responses are stored here until read.
sat_to_nordic_boot_t spi_boot_rx;
nordic_to_sat_boot_t spi_boot_tx;
//static bool received_bl_start_signal;
//static int num_boot_signal_received;
bool is_code_0_valid =false;
bool is_code_1_valid =false;

int	timeout_count;
	
static bool rx_crc_ok() {
	/* Calculate CRC16 */
	uint16_t crc16, recv_crc16;
	recv_crc16 = spi_boot_rx.crc16;
	spi_boot_rx.crc16 = 0;
	crc16 = spi_boot_rx.crc16 = crc16_compute((uint8_t*)(&spi_boot_rx), sizeof(spi_boot_rx), 0);
		
	if (recv_crc16 != crc16) {
		// fail crc check
		return false;
	}

	return true;	
}

void process_rx(void)
{
	if (rx_crc_ok()) {
		
			if (first_time) {
				if (spi_boot_rx.boot_cmd == NORDIC_BOOTLOADER_START) {
					nordic_bl_mode = NORDIC_BOOTLOADER_MODE_START;
					is_in_bl_mode = true;
				} else if (spi_boot_rx.boot_cmd == NORDIC_BOOTLOADER_TX) {
					nordic_bl_mode = NORDIC_BOOTLOADER_MODE_LOAD;
					last_crc16_received = spi_boot_rx.crc16;
					if (bootloader_write_batch(spi_boot_rx.hex)) {
						last_bl_line = spi_boot_tx.line_num = spi_boot_rx.line_num;
					} else {
						last_bl_line = spi_boot_tx.line_num = UINT32_MAX - spi_boot_rx.line_num;
					}
					first_time = false;
					is_in_bl_mode = true;
					
					/*
					is_load = true;
					spi_boot_tx.back[0] = *(uint32_t *) mem_address;
					spi_boot_tx.back[1] = *(uint32_t *) (mem_address + 4);
					spi_boot_tx.back[2] = *(uint32_t *) (mem_address + 8);
					spi_boot_tx.back[3] = *(uint32_t *) (mem_address + 12);
					mem_address += 16;
					*/
				} else if (spi_boot_rx.boot_cmd == NORDIC_BOOTLOADER_ERASE) {
					bootloader_erase_app(APPLICATION_BASE_ADDRESS, spi_boot_rx.line_num, usb_update);
					nordic_bl_mode = NORDIC_BOOTLOADER_MODE_ERASE;

					first_time = false;
					is_in_bl_mode = true;
				} else if (spi_boot_rx.boot_cmd == NORDIC_BOOTLOADER_DONE) {
					nordic_bl_mode = NORDIC_BOOTLOADER_MODE_STOP;
					
					// finished code transfer, now check if the code is valid
					//bootloader_check_code();
					is_code_0_valid = is_code_valid(bank0);
					if (is_code_valid(bank0)) {
						// if the code is valid, then jump to the application
						jump_to_app();
					} else {
						// if the code is not valid, then restart from beginning
						nordic_bl_mode = NORDIC_BOOTLOADER_MODE_INVALID;
					}
				} else if (spi_boot_rx.boot_cmd == NORDIC_BOOTLOADER_READ) {
					nordic_bl_mode = NORDIC_BOOTLOADER_MODE_READ;
					mem_address = spi_boot_rx.line_num;
					for (uint32_t i = 0; i < 4; i++) {
						spi_boot_tx.back[i] = *(uint32_t *) (mem_address + (i * 4));
					}
					first_time = false;
					is_in_bl_mode = true;
				}//cwati
//				} else {
//					// any other command, we will jump to existing application
//					// if it exist
//					if (is_code_0_valid) {
//						timeout_count--;
//						if (timeout_count <= 0) {
//							jump_to_app();
//						}
//					}
//				}
			}
			if (spi_boot_rx.boot_cmd == NORDIC_BOOTLOADER_ACK) {
				nordic_bl_mode = NORDIC_BOOTLOADER_MODE_START;
				/*
				if (!first_time && is_load) {
					spi_boot_tx.back[0] = *(uint32_t *) mem_address;
					spi_boot_tx.back[1] = *(uint32_t *) (mem_address + 4);
					spi_boot_tx.back[2] = *(uint32_t *) (mem_address + 8);
					spi_boot_tx.back[3] = *(uint32_t *) (mem_address + 12);
					mem_address += 16;
				}
				*/
				first_time = true;
			}
		}
}

/****************** END SPI stuff *******************/

///****************** GPIO stuff for triggering boot loader mode ******************/

//void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
//{
//	if (event_pins_low_to_high || event_pins_high_to_low) {
//		num_boot_signal_received++;
//		if (num_boot_signal_received > NUM_BOOT_SIGNAL) {
//			received_bl_start_signal = true;
//		}
//	}
//}

//static void init_gpio(void)
//{
//	static app_gpiote_user_id_t  m_gpiote_user_id;
//	uint32_t err_code;
//	received_bl_start_signal = false;
//	num_boot_signal_received = 0;
//	
//	nrf_gpio_cfg_input(BOOT_SIGNAL_PIN, NRF_GPIO_PIN_PULLDOWN);   //Configure button pin as input
//	APP_GPIOTE_INIT(10);               			//Only initialize once. Increase value of APP_GPIOTE_MAX_USERS if needed

//	err_code = app_gpiote_user_register(&m_gpiote_user_id, 1 << BOOT_SIGNAL_PIN, 1 << BOOT_SIGNAL_PIN, gpiote_event_handler);
//	APP_ERROR_CHECK(err_code);

//	err_code = app_gpiote_user_enable(m_gpiote_user_id);
//	APP_ERROR_CHECK(err_code);
//}
///****************** END GPIO stuff for triggering boot loader mode ******************/

void jump_to_app(void)
{
	hub_timer_stop();
	spi_slave_deinit();
	bootloader_active = 0;
	interrupts_disable();
	bjump();
}

#if PRINT_DEBUG
static void hub_timer_self_test(void) {
//	char report[110] = {0};
	
	hub_timer_start();
	uint32_t before = hub_timer_get_elapsed_ms();
	nrf_delay_us(100000);
	uint32_t after = hub_timer_get_elapsed_ms();

//	snprintf(report,109,"\r\nelapsed timer=0x%x",before);
//	debug_putstring((uint8_t*)report);
//	snprintf(report,109,"\r\nelapsed timer=0x%x",after);
//	debug_putstring((uint8_t*)report);
	hub_timer_stop();
}
#endif

extern void SWI0_IRQHandler_Bootloader(void);

static bool fw_update_requested() {
	uint32_t valid_bank, request;
	
	valid_bank = flash_read_mem(NORDIC_2ND_BANK_VALID_ADDR);
	request = flash_read_mem(NORDIC_FW_UPDATE_REQ_ADDR);
	
	if ((request == UPDATE_REQUESTED) && \
			(valid_bank == BANK_IS_VALID)) {
				return true;
			}

	return false;
}

/* Minimum erase is 1 page, 0x400 or 1024 bytes.
 * When we erase, we want to make sure we only erase the part we're trying to erase
 * and recover the other data.
 */
static void ts_reset_update_request(void) {
	uint32_t addr, counter;
	uint32_t saved[256];
	
	/* Save existing flash */
	for (addr = TS_FLASH_START_ADDR, counter = 0; \
			 addr < (TS_FLASH_START_ADDR + 0x400); \
			 addr += 4, counter++) {
		saved[counter] = flash_read_mem(addr);
	}
			 
	/* Erase flash page */
	flash_erase_mem(TS_FLASH_START_ADDR);

	for (addr = TS_FLASH_START_ADDR, counter = 0; \
			 addr < (TS_FLASH_START_ADDR + 0x400); \
			 addr += 4, counter++) {

		if ((addr != NORDIC_2ND_BANK_VALID_ADDR) && (addr != NORDIC_FW_UPDATE_REQ_ADDR)) {
			flash_write_mem(addr, saved[counter]);
		}
	}
}

int main(void)
{
	/* These 2 interrupts were called sometimes.
   * If you don't disable it here, when they're called it would hang the software due to the 
   * Default_Handler routine.	*/
//	NVIC_DisableIRQ(SPI0_TWI0_IRQn);
//	NVIC_DisableIRQ(RADIO_IRQn);
	interrupts_disable();
	
	uint32_t elapsed_ms = 0;
	
	// Set 16 MHz crystal as our 16 MHz clock source (as opposed to internal RCOSC)
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
	{
			// Wait
	}

	// Initialize the slow clock used for recording timestamps.
	hub_timer_start_clock();
	int ret = hub_timer_init();
#if PRINT_DEBUG
//	snprintf(report,109, "\r\nhub_timer_init ret %d\n", ret);
//	debug_putstring((uint8_t*)report);

	//test rtc timer
	hub_timer_self_test();
#endif		
	/* Start timer */
	hub_timer_start();

	bootloader_active = 1;

	nordic_bl_mode = NORDIC_BOOTLOADER_MODE_INVALID;
	spi_boot_rx.boot_cmd = 0;
	is_code_0_valid = is_code_valid(bank0);
	timeout_count = CYCLES_TO_WAIT_FOR_BOOT;
	if (!is_code_0_valid) {
		// if there is no valid code, then just run bootloader mode
		is_in_bl_mode = true;
	}
	
	//bool is_load = false;
	spi_boot_tx.major_version = BOOTLOADER_MAJOR_VERSION;
	spi_boot_tx.minor_version = BOOTLOADER_MINOR_VERSION;
	
	spi_slave_example_init();

	/* Dual-bank
		1) check if BANK1 code is valid
		2) copy code from BANK1 to BANK0
		3) erase BANK1
		4) Continue on... if no bootloader, then jump to BANK0
	*/
	is_code_1_valid = is_code_valid(bank1);
	if (is_code_1_valid && (fw_update_requested())) {
#if EVAL_BOARD
		// Configure LED-pins as outputs.
    LEDS_CONFIGURE(LEDS_MASK);
#endif
		
		/* Erase BANK0.  Time consuming.  Should erase just used ones. TODO.*/
		bootloader_erase_app(APPLICATION_BASE_ADDRESS, APPLICATION_MAX_SIZE, backup_update);

		/* Copy from BANK1 to BANK0 */
		ts_copy_app();

		ts_reset_update_request();

		/* Erase BANK1.  Time consuming.  Should erase just used ones. TODO.*/
		bootloader_erase_app(APPLICATION_BACKUP_ADDRESS, APPLICATION_MAX_SIZE, backup_update);

		/* We don't jump to app... we wait for intercept from USB.  If there's
		 * no intercept then we'll jump to app. */
	}

	while (true)
	{
		switch (nordic_bl_mode) {
			case NORDIC_BOOTLOADER_MODE_INVALID:
				
				/* Why are we still here?  CWati todo, check this. */
				elapsed_ms = hub_timer_get_elapsed_ms();
				if (elapsed_ms >= MS_TO_WAIT_FOR_BOOT_SIGNAL) {
					if (is_code_valid(bank0)) {
						// if the code is valid, then jump to the application
						jump_to_app();
					}
				}
				// this is the first time we enter the loop	
				spi_boot_tx.last_crc16_recvd = 0;
				last_bl_line = 0;
				first_time = true;
				spi_boot_tx.reply_cmd = 0;
				if (is_in_bl_mode) {
					spi_boot_tx.line_num = 0;
				} else {
					spi_boot_tx.line_num = timeout_count;
				}
				break;
			case NORDIC_BOOTLOADER_MODE_START:
				// send a signal to 22F to indicate that bootloader is
				// ready to receive any command
				spi_boot_tx.reply_cmd = NORDIC_BOOTLOADER_READY;
				spi_boot_tx.line_num = 0;
				break;
			case NORDIC_BOOTLOADER_MODE_LOAD:
			case NORDIC_BOOTLOADER_MODE_ERASE:
				// this is after a TX or ERASE command
				// send an acknowledgement to 22F
				spi_boot_tx.reply_cmd = NORDIC_BOOTLOADER_ACK;
				spi_boot_tx.last_crc16_recvd = last_crc16_received;
				spi_boot_tx.line_num = last_bl_line;
				break;
			case NORDIC_BOOTLOADER_MODE_READ:
				// this is after a READ command
				// send a READY signal to 22F
				spi_boot_tx.reply_cmd = NORDIC_BOOTLOADER_ACK;
				spi_boot_tx.line_num = mem_address;
				break;
			default:
				spi_boot_tx.reply_cmd = NORDIC_BOOTLOADER_READY;
				spi_boot_tx.line_num = 0;
		}
		
		//nrf_delay_us(ATOM_US);
        
//		// Let the CPU sleep until the next interrupt comes along
//			__WFE();
//			__SEV();
//			__WFE();
	}
}

