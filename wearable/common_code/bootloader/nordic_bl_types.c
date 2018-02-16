/*
 * nordic_bl_types.h
 *
 *  Created on: Jul 7, 2016
 *      Author: smantik
 */

#include <stdbool.h>
#include <string.h>
#include "nordic_bl_types.h"

/*
bool convert_intel_hex_to_bin(intel_hex_t hex, intel_hex_bin_t *bin)
{
	memset(bin, 0, sizeof(intel_hex_bin_t));
	bin->size = (hex.size - 1) >> 1;
	if (hex.data[0] != ':') {
		return false;	// need to start with ":"
	}
	if ((bin->size << 1) + 1 != hex.size) {
		return false;	// original hex need to be odd number
	}
	int i, j;
	for (i = 1, j = 0; i < hex.size; i++) {
		uint8_t value;
		switch (hex.data[i]) {
			case '0':
				value = 0;
				break;
			case '1':
				value = 1;
				break;
			case '2':
				value = 2;
				break;
			case '3':
				value = 3;
				break;
			case '4':
				value = 4;
				break;
			case '5':
				value = 5;
				break;
			case '6':
				value = 6;
				break;
			case '7':
				value = 7;
				break;
			case '8':
				value = 8;
				break;
			case '9':
				value = 9;
				break;
			case 'a':
			case 'A':
				value = 10;
				break;
			case 'b':
			case 'B':
				value = 11;
				break;
			case 'c':
			case 'C':
				value = 12;
				break;
			case 'd':
			case 'D':
				value = 13;
				break;
			case 'e':
			case 'E':
				value = 14;
				break;
			case 'f':
			case 'F':
				value = 15;
				break;
			default:
				value = 0;
		}
		if (i % 2 == 1) {
			value = value << 4;
			bin->data[j] = value;
		} else {
			bin->data[j] += value;
			j++;
		}
	}
	return true;
}


void print_hex_bin(intel_hex_bin_t hex)
{
	debug_printf("%d[:", hex.size);
	int i;
	for (i = 0; i < hex.size; i++) {
		debug_printf("%02X", hex.data[i]);
	}
	debug_printf("]\n\r");
}
*/


bool convert_intel_hex_to_hex_record(intel_hex_t hex, hex_record_t *record)
{
	if (!hexparser_parse_string(hex.data, hex.size, record)) {
		return false;
	}
	//if (!hexparser_parse_bytes(hex.data, hex.size, record)) {
	//	return false;
	//}
	return hexparser_is_record_valid(record);
}

uint16_t convert_string_to_hex_record(char * hex_string, uint16_t length, hex_record_t *record)
{
	uint16_t index = 0;
	uint16_t count = 0;
	while(index < length) {
		if (!hexparser_parse_string(hex_string, length, record)) {
			return count;
		}
		if (!hexparser_is_record_valid(record)) {
			return count;
		}
		uint16_t num_bytes = (record->byte_count << 1) + 11;
		hex_string = &(hex_string[num_bytes]);
		index += num_bytes;
		record++;
		count++;
	}
	//if (!hexparser_parse_bytes(hex.data, hex.size, record)) {
	//	return false;
	//}
	return count;
}

void print_hex_record(hex_record_t hex)
{
#if 0
	int i;
	debug_printf("HEX: %x %x %x [", hex.byte_count, hex.address, hex.type);
	for (i = 0; i < hex.byte_count; i++) {
		debug_printf("%x", hex.data.bytes[i]);
	}
	debug_printf("] %x\n\r", hex.checksum);
	debug_printf("    TYPE: ");
	switch(hex.type) {
	case INVALID_RECORD:
		debug_printf("INVALID\n\r");
		break;
	case DATA_RECORD:
		debug_printf("DATA\n\r");
		break;
	case END_OF_FILE_RECORD:
		debug_printf("ENDOFFILE\n\r");
		break;
	case EXTENDED_SEGMENT_ADDRESS_RECORD:
		debug_printf("EXTENDEDSEGMENTADDRESS\n\r");
		break;
	case START_SEGMENT_ADDRESS_RECORD:
		debug_printf("STARTSEGMENTADDRESS\n\r");
		break;
	case EXTENDED_LINEAR_ADDRESS_RECORD:
		debug_printf("EXTENDEDLINEARADDRESS\n\r");
		break;
	case START_LINEAR_ADDRESS_RECORD:
		debug_printf("STARTLINEARAADDRESS\n\r");
		break;
	default:
		debug_printf("UNKNOWN\n\r");
	}
#endif
}
