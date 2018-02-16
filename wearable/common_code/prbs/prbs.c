/* Copyright (c) 2016 Turingsense
 *
 * January 25th, 2016
 *
 * prbs.c
 *
 * Generating random number, using PRBS 7,15,23, or 31.
 * PRBS7 = x^7 + x^6 + 1
 * PRBS15 = x^15 + x^14 + 1
 * PRBS23 = x^23 + x^18 + 1
 * PRBS31 = x^31 + x^28 + 1
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#define MY_PRBS_START_VALUE		0x02

uint8_t prbs7() {
	static uint8_t a = MY_PRBS_START_VALUE;

	int newbit = (((a >> 6) ^ (a >> 5)) & 1);
	a = ((a << 1) | newbit) & 0x7f;

	return a;	
}

uint16_t prbs15() {
	static uint16_t a = MY_PRBS_START_VALUE;

    int newbit = (((a >> 14) ^ (a >> 13)) & 1);
    a = ((a << 1) | newbit) & 0x7fff;

	return a;
}

uint32_t pbrs23() {
	static uint32_t a = MY_PRBS_START_VALUE;

	int newbit = (((a >> 22) ^ (a >> 17)) & 1);
	a = ((a << 1) | newbit) & 0x7FFFFF;

	return a;
}

uint32_t prbs31() {
	static uint32_t a = MY_PRBS_START_VALUE;

	int newbit = (((a >> 30) ^ (a >> 27)) & 1);
	a = ((a << 1) | newbit) & 0x7FFFFFFF;

	return a;
}
