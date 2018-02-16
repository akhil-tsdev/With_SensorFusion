/* Copyright (c) 2016 TuringSense
 * nordic_bl_22f.c
 *
 *  Created on: Jul 7, 2016
 *      Author: smantik
 */

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "nordic_bl_22f.h"
#include "fsl_device_registers.h"
#include "fsl_os_abstraction.h"
#include "fsl_dspi_master_driver.h"
#include "fsl_dspi_slave_driver.h"
#include "fsl_debug_console.h"
#include "sat_module.h"
//#include "board.h"
//#include "mpu9250_firmware.h"
//#include "mpu9250_freq.h"
#include "crc16.h"

/*******************************************************************
 * global variables
 *******************************************************************/
static sat_to_nordic_boot_t spi_boot_tx;
//static nordic_to_sat_boot_t spi_boot_rx;
static sat_to_nordic_boot_t spi_boot_rx;
static uint8_t spi_boot_tx_arr[SPI_BOOT_TRANSFER_SIZE] = {0};
static uint8_t spi_boot_rx_arr[SPI_BOOT_TRANSFER_SIZE] = {0};

// get_hex_from_usb
// It will read hex data from usb one line at a time
// It starts by sending a "Ready" signal to the usb so
// the python script can start sending the data.
// The data will be read as
// 1. first the length of the characters
// 2. the number of chars (as given by #1)
char hex_to_char(uint8_t hex)
{
	switch (hex) {
	case 0: return '0';
	case 1: return '1';
	case 2: return '2';
	case 3: return '3';
	case 4: return '4';
	case 5: return '5';
	case 6: return '6';
	case 7: return '7';
	case 8: return '8';
	case 9: return '9';
	case 10: return 'a';
	case 11: return 'b';
	case 12: return 'c';
	case 13: return 'd';
	case 14: return 'e';
	case 15: return 'f';
	default: return 'x';
	}
}
void get_hex_from_usb(intel_hex_t *hex)
{
    uint16_t	size;
    char	in_char;
    char	buf[MAX_HEX_LENGTH + 1];

    memset(hex, 0, sizeof(intel_hex_t));	// first, clear up the variable

    debug_printf("READY\n\r");	// send a "Ready" signal
    debug_scanf("%u", &size);	// read the length of the data
    int i;
    for (i = 0; i < size; i++) {
    	in_char = debug_getchar();
		if (i < MAX_HEX_LENGTH) {	// make sure it doesn't go beyond the buf size
	    	buf[i] = hex->data[i] = in_char;
		}
    }

    /* make sure the size does't go over the buffer limit */
    if (size > MAX_HEX_LENGTH) {
    	size = MAX_HEX_LENGTH;
    }
    hex->size = size;
    buf[size] = '\0';
    debug_printf("Read: [%s]\n\r", buf);
#if 0
    /* for debugging purpose, print back the hex received */
    //char	buf[MAX_HEX_LENGTH + 1];
    debug_printf("Read: [");
    for (i = 0; i < size; i++) {
    	//buf[size * 2] = hex_to_char(hex->data[i] >> 4);
    	//buf[(size * 2) + 1] = hex_to_char(hex->data[i] % 16);
    	//debug_printf("%s", hex->data[i]);
    	debug_printf("%x", hex->data[i]);
    }
    //buf[size * 2] = '\0';
    //debug_printf("Read: [%s]\n\r", buf);
    debug_printf("]\n\r");
#endif
}

uint16_t get_hex_string_from_usb(char *hex, uint16_t max_length)
{
	uint16_t	size;
	char		in_char;
	//char		buf[max_length + 1];

	memset(hex, 0, sizeof(char) * max_length);	// first, clear up the variable

	debug_printf("READY\n\r");	// send a "Ready" signal
	debug_scanf("%u", &size);	// read the length of the data
	int i;
	for (i = 0; i < size; i++) {
		in_char = debug_getchar();
		if (i < max_length) {	// make sure it doesn't go beyond the buf size
			//buf[i] =
			hex[i] = in_char;
		}
	}

	/* make sure the size does't go over the buffer limit */
	if (size > max_length) {
		size = max_length;
	}
	//buf[size] = '\0';
	//debug_printf("Read: [%s]\n\r", buf);
	return size;
}

//
//
//void send_one_hex(void)
//{
//    dspi_status_t 			dspiResult;
//    uint32_t 				wordsTransfer = 0;
//    static uint16_t			last_boot_crc16_sent = 0;
//    uint16_t				generated_crc16, temp_crc16;
//    bool					need_spi_resend = false;
//
//	/* Generate CRC16 */
//	spi_boot_tx.crc16 = 0;
//	generated_crc16 = crc16_compute((uint8_t*)&spi_boot_tx, sizeof(spi_boot_tx), 0);
//	spi_boot_tx.crc16 = last_boot_crc16_sent = generated_crc16;
//
//	do {
//		/* 06/17/17 cwati size of spi tx & rx should be the same for reliability
//		 * in the SPI transfer.
//		 */
//		//debug_printf("Copy data\n\r");
//		memcpy(&spi_boot_tx_arr, &spi_boot_tx, sizeof(spi_boot_tx));
//		memcpy(&spi_boot_rx_arr, &spi_boot_rx, sizeof(spi_boot_rx));
//
//		debug_printf("Transfer CRC 0x%x\n\r", spi_boot_tx.crc16);
//		dspiResult = DSPI_DRV_SlaveTransfer(knRF51822SpiInstance,
//											(uint8_t*)&spi_boot_tx_arr,
//											(uint8_t*)&spi_boot_rx_arr,
//											SPI_BOOT_TRANSFER_SIZE);
//
//		if (dspiResult != kStatus_DSPI_Success) {
//			debug_printf("\r\n ERROR: Can not start SPI to Nordic \r\n");
//			//blinkLEDerror(redLed, 4);
//		};
//
//		/* Check SPI response */
//		//debug_printf("Wait for SPI response\n\r");
//		while (DSPI_DRV_SlaveGetTransferStatus(knRF51822SpiInstance, &wordsTransfer)== kStatus_DSPI_Busy) {}
//
//		memcpy(&spi_boot_rx, &spi_boot_rx_arr, sizeof(spi_boot_rx));
//
//		/* Check CRC16 */
//		temp_crc16 = spi_boot_rx.crc16;
//		spi_boot_rx.crc16 = 0;
//
//		debug_printf("Check CRC\n\r");
//		/*
//		char temp_hex[MAX_HEX_LENGTH + 1];
//		for (int i = 0; i < MAX_HEX_LENGTH; i++) {
//			temp_hex[i] = spi_boot_rx.hex.data[i];
//		}
//		//memcpy (temp_hex, &spi_boot_rx.hex.data, MAX_HEX_LENGTH + 1);
//		temp_hex[MAX_HEX_LENGTH] = '\0';
//		debug_printf("GOT: %d[%s] CRC: 0x%x\n\r", spi_boot_rx.hex.size, temp_hex, temp_crc16);
//		*/
//		generated_crc16 = crc16_compute ((uint8_t*) &spi_boot_rx, sizeof(spi_boot_rx), 0);
//		if (generated_crc16 == temp_crc16) {
//			debug_printf("Success\n\r");
//#if 0
//			debug_printf("\r\nGot clean crc16 from Nordic DEBUG: 0x%x\n", generated_crc16);
//
//			/* Check last CRC16 received by Nordic */
//			if (last_boot_crc16_sent != spi_boot_rx.last_crc16_recvd) {
//					debug_printf("\r\nGot crc16 from Nordic DEBUG last crc sent: 0x%x crc received: 0x%x\n",
//							last_boot_crc16_sent, spi_boot_rx.last_crc16_recvd);
//				need_spi_resend = true;
//			} else {
//					debug_printf("\r\nGot Right crc16 from Nordic DEBUG: 0x%x Exiting...\n", spi_boot_rx.last_crc16_recvd);
//				debug_printf("\r\nCommand has been successfully requested to the RF Chip. "
//							 "\r\nRF chip will continue transmitting unless you enter new command...\n");
//				need_spi_resend = false;
//			}
//#endif
//		} else {
//				debug_printf("\r\nGot bad crc16 from Nordic DEBUG: 0x%x Expected:0x%x\n", temp_crc16, generated_crc16);
//			need_spi_resend = true;
//		}
//
//		if (!need_spi_resend) {
//			break;
//		}
//
//		OSA_TimeDelay(500); /* ms */ //cwati remove me
//		debug_printf("Resend\n\r");
//
//		break;	// just break for now
//
//	} while(1);
//
//	OSA_TimeDelay(1000); /* ms */
//}

#include "ts_rtc.h"
#include "main_22f_nordic_bl.h"
uint16_t init_hex_from_usb(void)
{
	uint16_t	num_lines;
	uint32_t    time_now, time_start, time_diff;

	time_start = k22f_get_rtc();

	// first, tell python script that we are ready to start
	// transmitting hex data
#if DEBUG_BL
	debug_printf("Starting Nordic bootloader\n\r");
	debug_printf("\n\r");
#endif
	debug_printf("STARTTXHEX\n\r");

	// we read the nordic hex files one line at a time
	// first get the number of lines
	debug_printf("READY\n\r");
	debug_scanf("%u", &num_lines);

	time_now = k22f_get_rtc();
	time_diff = time_now - time_start;
	nordic_bl_mode_time[bl_init1] += time_diff;
	nordic_bl_mode_cnt[bl_init1]++;

	return num_lines;
}


void end_hex_from_usb(void)
{
	// give signal to usb that everything is fine
	debug_printf("ENDTXHEX\n\r");
}


void upload_nordic_hex(void)
{
    //intel_hex_t	hex;
    uint16_t	num_lines;
    int	        i;

    memset(&spi_boot_tx, 0, sizeof(spi_boot_tx));	// clear up the variable

    /*
    // first, tell python script that we are ready to start
    // transmitting hex data
    debug_printf("\n\rSTARTTXHEX\n\r");

    // we read the nordic hex files one line at a time
    // first get the number of lines
    debug_printf("READY\n\r");
    debug_scanf("%u", &num_lines);

    spi_boot_tx.total_lines = num_lines;
    */
    //spi_boot_tx.total_lines = init_hex_from_usb();

    for (i = 0; i < num_lines; i++) {
    	spi_boot_tx.line_num = i;
        get_hex_from_usb(&(spi_boot_tx.hex));
        //send_one_hex();
    }

    // give signal to usb that everything is fine
    debug_printf("ENDTXHEX\n\r");
}




