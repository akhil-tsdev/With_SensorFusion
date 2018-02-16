/*
 * nordic_bl_22f.h
 *
 *  Created on: Jul 7, 2016
 *      Author: smantik
 */

#ifndef __NORDIC_BL_22F_H_
#define __NORDIC_BL_22F_H_

#include "nordic_bl_types.h"

// get_hex_from_usb
// It will read hex data from usb one line at a time
// It starts by sending a "Ready" signal to the usb so
// the python script can start sending the data.
// The data will be read as
// 1. first the length of the characters
// 2. the number of chars (as given by #1)
void get_hex_from_usb(intel_hex_t *hex);
uint16_t get_hex_string_from_usb(char *hex, uint16_t max_length);

void upload_nordic_hex(void);

//void send_one_hex(void);

uint16_t init_hex_from_usb(void);

void end_hex_from_usb(void);

#endif /* SOURCES_NORDIC_BL_TYPES_H_ */
