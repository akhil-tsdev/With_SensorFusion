/*
 * turingsense_platform.h
 *
 *  Created on: Mar 7, 2015
 *      Author: End User
 */

#ifndef TURINGSENSE_PLATFORM_H_
#define TURINGSENSE_PLATFORM_H_

/* The following functions must be defined for this platform:
 * i2c_write(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char const *data)
 *      NOTE: actually uses SPI
 * i2c_read(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char *data)
 *      NOTE: actually uses SPI
 * delay_ms(unsigned long num_ms)
 * get_ms(unsigned long *count)
 * reg_int_cb(void (*cb)(void), unsigned char port, unsigned char pin)
 * labs(long x)
 * fabsf(float x)
 * min(int a, int b)
 */

/*
int spi_write(unsigned char reg_addr,
		      unsigned char length,
		      unsigned char const *data);

int spi_read(unsigned char reg_addr,
	         unsigned char length,
			 unsigned char *data);
*/

int i2c_write(unsigned char sa,
		      unsigned char reg_addr,
		      unsigned char length,
		      const unsigned char *data);

int i2c_read(unsigned char sa,
		     unsigned char reg_addr,
		     unsigned char length,
		     unsigned char *data);

void delay_ms(unsigned long num_ms);

void get_ms(unsigned long *count);

void reg_int_cb(void (*cb)(void),
		        unsigned char port,
		        unsigned char pin);

#define log_i       MPL_LOGI
#define log_e       MPL_LOGE

#define min(a,b) ((a<b)?a:b)

// XXX should define following as appropriate macro:
void __no_operation(void);

#endif /* TURINGSENSE_PLATFORM_H_ */
