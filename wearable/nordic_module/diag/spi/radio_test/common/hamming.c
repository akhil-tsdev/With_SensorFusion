/* From hackersdelight.org, which gave the license:
	You are free to use, copy, and distribute any of the code on this web site,
	whether modified by you or not. You need not give attribution. This includes
	the algorithms (some of which appear in Hacker's Delight), the Hacker's
	Assistant, and any code submitted by readers.
 */

#include "hamming.h"

static uint8_t parity(uint32_t x) {
   x = x ^ (x >> 1);
   x = x ^ (x >> 2);
   x = x ^ (x >> 4);
   x = x ^ (x >> 8);
   x = x ^ (x >> 16);
   return x & 1;
}

uint8_t hamming_code_for_data(uint32_t data) {	
	uint32_t p0, p1, p2, p3, p4, p5, p6, p;
	uint32_t t1, t2, t3;

	// First calculate p[5:0] ignoring u[0].
	p0 = data ^ (data >> 2);
	p0 = p0 ^ (p0 >> 4);
	p0 = p0 ^ (p0 >> 8);
	p0 = p0 ^ (p0 >> 16);        // p0 is in posn 1.

	t1 = data ^ (data >> 1);
	p1 = t1 ^ (t1 >> 4);
	p1 = p1 ^ (p1 >> 8);
	p1 = p1 ^ (p1 >> 16);        // p1 is in posn 2.

	t2 = t1 ^ (t1 >> 2);
	p2 = t2 ^ (t2 >> 8);
	p2 = p2 ^ (p2 >> 16);        // p2 is in posn 4.

	t3 = t2 ^ (t2 >> 4);
	p3 = t3 ^ (t3 >> 16);        // p3 is in posn 8.

	p4 = t3 ^ (t3 >> 8);         // p4 is in posn 16.

	p5 = p4 ^ (p4 >> 16);        // p5 is in posn 0.

	p = ((p0>>1) & 1) | ((p1>>1) & 2) | ((p2>>2) & 4) |
		  ((p3>>5) & 8) | ((p4>>12) & 16) | ((p5 & 1) << 5);

	p = p ^ (-(data & 1) & 0x3F);   // Now account for u[0].

	// Add SECDED parity bit.
	p = p | (parity(data ^ p) << 6);

	return p;
}

int hamming_correct_data(uint8_t check_bits, uint32_t *data) {
	uint32_t po, p, syn, b;

	// What check bits would we *expect* for this data?
	po = parity(check_bits ^ *data); 
	p = (hamming_code_for_data(*data) & 0x3f);
	// Syndrome (exclusive of overall parity bit).
	syn = p ^ (check_bits & 0x3F);           
	if (po == 0) {
		if (syn == 0) return 0;   // If no errors, return 0.
		else return 2;            // Two errors, return 2.
	}
	// One error occurred.
	
	// If the syndrome has 0-1 bits set, the error is in the
	// check bits; the data itself is OK.
	if (((syn - 1) & syn) == 0)
		return 1;

	// One error, and syn bits 5:0 tell where it is in the data.

	b = syn - 31 - (syn >> 5); // Map syn to range 0 to 31.
	// if (syn == 0x1f) b = 0;    // (These two lines equiv.
	// else b = syn & 0x1f;       // to the one line above.)
	*data = *data ^ (1 << b);      // Correct the bit.
	return 1;
}
