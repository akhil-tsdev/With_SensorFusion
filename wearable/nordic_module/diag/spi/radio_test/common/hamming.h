#include <stdint.h>

/** Computes the six parity check bits for the
   "information" bits given in the 32-bit word |data|. The
   check bits are p[5:0]. p[6] holds an overall parity bit.

   Bit   Checks these bits of |data|
   p[0]  0, 1, 3, 5, ..., 31 (0 and the odd positions).
   p[1]  0, 2-3, 6-7, ..., 30-31 (0 and positions xxx1x).
   p[2]  0, 4-7, 12-15, 20-23, 28-31 (0 and posns xx1xx).
   p[3]  0, 8-15, 24-31 (0 and positions x1xxx).
   p[4]  0, 16-31 (0 and positions 1xxxx).
   p[5]  1-31 */
uint8_t hamming_code_for_data(uint32_t data);


/** This function looks at the received seven check bits and 32 information
   bits and determines how many errors occurred (under the presumption that it
	 must be 0, 1, or 2). It returns with 0, 1, or 2, meaning that no errors, one
	 error, or two errors occurred. It corrects the information word received
	 if there was one error in it. */
int hamming_correct_data(uint8_t check_bits, uint32_t *data);
