#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fsl_os_abstraction.h"
#include "fsl_dspi_master_driver.h"
#include "fsl_i2c_master_driver.h"

#include "../Sources/dspi.h"  /* for kSensorInstance, masterSpiDevice */

#include "../Sources/main.h"  /* for kMPU9250AuxI2CInstance, mpu9250_reg_int_cb() */

#include "inv_platform.h"
#include "log.h"

#define MPU9250_FIFO_SIZE 512
#define MAX_SPI_XFER (MPU9250_FIFO_SIZE + 1)

static uint8_t spi_write_buffer[MAX_SPI_XFER];
static uint8_t spi_read_buffer[MAX_SPI_XFER];

/**
 *  @brief      SPI register write.
 *  @param[in]  reg_addr   address of SPI register.
 *  @param[in]  length     count of data bytes to be written
 *  @param[in]  data       data to be written
 *  @return     0 if successful.
 */
static int spi_write(unsigned char reg_addr,
		      unsigned char length,
		      const unsigned char *data)
{
	dspi_status_t error;

	spi_write_buffer[0] = reg_addr;
	memcpy(& spi_write_buffer[1], data, length);
	error = DSPI_DRV_MasterTransferDataBlocking(kSensorInstance,
												&masterSpiDevice,
												spi_write_buffer,
												spi_read_buffer,
												length + 1,
												1000); // timeout in microseconds
	return error != kStatus_DSPI_Success;
}

static i2c_device_t i2c_slave =
{
    .address = 0x0c,
    .baudRate_kbps = 400
};

static int aux_i2c_write(unsigned char sa,
						 unsigned char reg_addr,
						 unsigned char length,
						 const unsigned char *data)
{
    i2c_status_t error;

    i2c_slave.address = sa;
    error = I2C_DRV_MasterSendDataBlocking(kMPU9250AuxI2CInstance,
	                                       &i2c_slave,
	                                       &reg_addr,
	                                       1,
	                                       (uint8_t *) data,
	                                       length,
	                                       500);
	return error != kStatus_I2C_Success;
}

int i2c_write(unsigned char sa,
		      unsigned char reg_addr,
		      unsigned char length,
		      const unsigned char *data)
{
	if (sa == 0x68)
		return spi_write(reg_addr, length, data);
	else if (sa == 0x0c)
		return aux_i2c_write(sa, reg_addr, length, data);
	printf("unexpected write to I2C addr %02x, reg %02x, len %02x\r\n", sa, reg_addr, length);
	while(true)
		;
}

/**
 *  @brief      SPI register write.
 *  @param[in]  reg_addr   address of SPI register.
 *  @param[in]  length     count of data bytes to be read
 *  @param[out] data       data read
 *  @return     0 if successful.
 */
static int spi_read(unsigned char reg_addr,
	         unsigned char length,
			 unsigned char *data)
{
	dspi_status_t error;

	memset(spi_write_buffer, 0, length + 1);
	spi_write_buffer[0] = reg_addr | 0x80;
	error = DSPI_DRV_MasterTransferDataBlocking(kSensorInstance,
												&masterSpiDevice,
												spi_write_buffer,
												spi_read_buffer,
												length + 1,
												1000); // timeout in microseconds
	memcpy(data, & spi_read_buffer[1], length);
	return error != kStatus_DSPI_Success;
}

static int aux_i2c_read(unsigned char sa,
						unsigned char reg_addr,
						unsigned char length,
						unsigned char *data)
{
    i2c_status_t error;

    i2c_slave.address = sa;
	error = I2C_DRV_MasterReceiveDataBlocking(kMPU9250AuxI2CInstance,
	                                       	  &i2c_slave,
	                                       	  &reg_addr,
	                                       	  1,
	                                       	  data,
	                                       	  length,
	                                       	  500);
	return error != kStatus_I2C_Success;
}

int i2c_read(unsigned char sa,
		     unsigned char reg_addr,
		     unsigned char length,
		     unsigned char *data)
{
	if (sa == 0x68)
		return spi_read(reg_addr, length, data);
	else if (sa == 0x0c)
		return aux_i2c_read(sa, reg_addr, length, data);
	printf("unexpected read of I2C addr %02x, reg %02x, len %02x\r\n", sa, reg_addr, length);
	while(true)
		;
}

void delay_ms(unsigned long num_ms)
{
	OSA_TimeDelay(num_ms);
}

void get_ms(unsigned long *count)
{
	*count = OSA_TimeGetMsec();
}

void reg_int_cb(void (*cb)(void),
		        unsigned char port,
		        unsigned char pin)
{
	mpu9250_reg_int_cb(cb);
}

static void vlog(const char *severity, const char *fmt, va_list ap)
{
	char buf[200];
	vsnprintf(buf, sizeof(buf), fmt, ap);
	printf("%s: %s", severity, buf);
}

// XXX should define following as appropriate macro in header only
void __no_operation(void)
{
}
