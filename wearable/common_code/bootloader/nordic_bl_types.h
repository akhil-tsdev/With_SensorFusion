/* Copyright (c) 2016 TuringSense
 * nordic_bl_types.h
 *
 *  Created on: Jul 7, 2016
 *      Author: smantik
 */

#ifndef __NORDIC_BL_TYPES_H_
#define __NORDIC_BL_TYPES_H_

#include "stdint.h"
#include "hexparser.h"

#define MAX_HEX_LENGTH 101
#define MAX_HEX_BIN_LENGTH 25

#define BOOTLOADER_MAJOR_VERSION 2
#define BOOTLOADER_MINOR_VERSION 8

typedef struct intel_hex_ {
	/* size of the hex data */
	uint16_t size;

	/* hex data (max length = 100) */
	char data[MAX_HEX_LENGTH];
} intel_hex_t;

typedef struct intel_hex_bin_ {
	/* This is the compact binary version of the intel hex data */
	/* It will drop the ":" hex mark */
	/* size of the hex data */
	uint16_t size;

	/* hex data (max length = 40) */
	uint8_t data[MAX_HEX_BIN_LENGTH];
} intel_hex_bin_t;

typedef hexparser_record hex_record_t;

#define NUM_HEX_PER_SPI_PACKET 8

typedef struct sat_to_nordic_boot_ {
	/* boot command */
	uint32_t boot_cmd;	// need to be the first 32bit so it will match with timestamp from sat_to_nordic_t

	/* the current line number */
	uint32_t line_num;

	/* the hex data */
	hex_record_t hex[NUM_HEX_PER_SPI_PACKET];

	/* To validate the SPI transcation */
	uint16_t crc16;
} sat_to_nordic_boot_t;

typedef struct nordic_to_sat_boot_ {
	uint32_t reply_cmd;		// reply command from nordic back to sat
	uint32_t line_num;
	uint32_t back[4];
	uint16_t last_crc16_recvd;
	uint16_t major_version;
	uint16_t minor_version;
} nordic_to_sat_boot_t;

typedef struct padded_nordic_to_sat_boot_ {
	uint16_t 				crc16;
	nordic_to_sat_boot_t	packet;
} padded_nordic_to_sat_boot_t;


#define SPI_BOOT_TRANSFER_SIZE	(sizeof(sat_to_nordic_boot_t))

#ifndef MAX
#define MAX(a,b) ((a>b)?(a):(b))
#endif
#define SPI_BOOT_LENGTH MAX(sizeof(sat_to_nordic_boot_t),sizeof(padded_nordic_to_sat_boot_t))

#define NORDIC_BOOTLOADER_START	(UINT32_MAX - 1)
#define NORDIC_BOOTLOADER_READY (UINT32_MAX - 2)
#define NORDIC_BOOTLOADER_TX	(UINT32_MAX - 3)
#define NORDIC_BOOTLOADER_ACK	(UINT32_MAX - 4)
#define NORDIC_BOOTLOADER_DONE	(UINT32_MAX - 5)
#define NORDIC_BOOTLOADER_ERASE	(UINT32_MAX - 6)
#define NORDIC_BOOTLOADER_READ	(UINT32_MAX - 7)

typedef enum {
	NORDIC_BOOTLOADER_MODE_INVALID,		// not in bootloader
	NORDIC_BOOTLOADER_MODE_START,		// start bootloader (after sat sends "START BOOTLOADER" signal)
	NORDIC_BOOTLOADER_MODE_INIT,		// after nordic sends acknowledgement signal
	NORDIC_BOOTLOADER_MODE_READY,		// waiting mode to wait for the next command
	NORDIC_BOOTLOADER_MODE_LOAD,		// data loading mode (sending hex data from sat to nordic)
	NORDIC_BOOTLOADER_MODE_STOP,		// end of data tx
	NORDIC_BOOTLOADER_MODE_RESET,		// signal to reset nordic
	NORDIC_BOOTLOADER_MODE_ERASE,		// erase the current app data
	NORDIC_BOOTLOADER_MODE_READ,		// read the code data
	NORDIC_BOOTLOADER_MODE_FINISH		// when finished, jump back to 22F Bootloader [PV-907]
} nordic_bl_mode_t;

//bool convert_intel_hex_to_bin(intel_hex_t hex, intel_hex_bin_t *bin);
bool convert_intel_hex_to_hex_record(intel_hex_t hex, hex_record_t *record);
uint16_t convert_string_to_hex_record(char * hex_string, uint16_t length, hex_record_t *record);
//void print_hex_bin(intel_hex_bin_t hex);
void print_hex_record(hex_record_t hex);

#endif /* __NORDIC_BL_TYPES_H_ */
