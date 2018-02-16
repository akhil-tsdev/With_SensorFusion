/*
 * Copyright (c) 2017 TuringSense
 *
 * lsm6dsl_firmware.c
 *
 * Interface to hw API talking to STM LSM6DSL
 */
#include "fsl_debug_console.h"
#include "fsl_dspi_master_driver.h"
#include "fsl_dspi_shared_function.h"
#include "fsl_interrupt_manager.h"
#include "fsl_os_abstraction.h"
#include "fsl_uart_driver.h"

#include "lsm6dsl_firmware.h"

bool lsm6dsl_spi_test(void) {
#define LSM6DSL_SPI_REGS	9
	const uint8_t reg[LSM6DSL_SPI_REGS] = {
			LSM6DSL_FIFO_CTRL1,
			LSM6DSL_INT1_CTRL,
			LSM6DSL_INT2_CTRL,
			LSM6DSL_TAP_CFG,
			LSM6DSL_TAP_THS_6D,
			LSM6DSL_INT_DUR2,
			LSM6DSL_FREE_FALL,
			LSM6DSL_MD1_CFG,
			LSM6DSL_MD2_CFG};
	uint8_t current_data;
	uint8_t read_data;
	uint8_t good_cnt = 0, bad_cnt = 0;

	debug_printf("\r\n ************ RESULTS SPI SENSORS TESTING ************\r\n");

	for (uint8_t cnt = 0; cnt < LSM6DSL_SPI_REGS; cnt++) {
		for (current_data = 0; current_data < UINT8_MAX; current_data++) {
			sensor_write_reg(reg[cnt], current_data);
			sensor_read_regs(reg[cnt], 1, &read_data);
			if (read_data == current_data) {
				//it passes
				good_cnt++;
			} else {
				bad_cnt++;
			}
		}
	}

	if (bad_cnt) {
		debug_printf("[SPI2TI:FAILED]\r\n");
		return false;
	} else {
		 debug_printf("[SPI2TI:OK]\r\n");
	}
	return true;
}

bool lsm6dsl_init_regs(void) {
	  // GYROSCOPE INITIALIZATION

	  /* TODO TODO Change the registers into macros TODO TODO */

	  /* Enable register address automatically incremented during a multiple byte
	     access with a serial interface */
	  /* Enable BDU */
	  sensor_write_reg(LSM6DSL_CTRL3_C,0x44);

	  /* FIFO mode selection -> BYPASS*/
	  sensor_write_reg(LSM6DSL_FIFO_CTRL5,0x00);

	  /* Output data rate selection - power down */
	  /* Full scale selection -> FS_HIGH*/
	  sensor_write_reg(LSM6DSL_CTRL2_G,0x5C);

	  sensor_write_reg(LSM6DSL_CTRL1_XL,0x55);

	return true;
}


bool lsm6dsl_detect_sensor(uint8_t sensor_num, uint8_t* whoami) {
	int status = kStatus_DSPI_Timeout;
	uint8_t data[LSM6DSL_READ_SIZE];

	*whoami = 0;

  	update_maxwell_cs(sensor_num);

	sensor_read_regs(LSM6DSL_REG_WHOAMI, 1, data);
	*whoami = data[0];

	return true;
}


bool lsm6dsl_start(void)
{
	//TO DO...
	return true;
}

bool lsm6dsl_read(int sensors,  // bit field, SENSOR_x above
				  sensor_data_t *sensor_data)
{
	uint8_t data[12];

	sensor_data->valid_sensors = 0;
	sensor_data->timestamp = OSA_TimeGetMsec();

	sensor_read_regs(LSM6DSL_OUTX_L_G, 12, data);

	//cwati TODO PIVOT_3_0_POC.  The following is causing issues with PIVOT 3.0 POC
//	if (! data[MPU9250_REG_OFFSET(MPU9250_REG_INT_STATUS)] & 0x01)
//		return false;

	int i = 0; //MPU9250_REG_OFFSET(MPU9250_REG_GYRO_XOUT_H);
	if (sensors & SENSOR_GYRO)
	{
		for (int axis = X; axis <= Z; axis++)
		{
			sensor_data->gyro[axis] = ( (int16_t) (data[i+1]<<8) ) | ( (int16_t) data[i] );
			i += 2;
		}
		sensor_data->valid_sensors |= SENSOR_GYRO;

	}

	if (sensors & SENSOR_ACCEL)
	{
		for (int axis = X; axis <= Z; axis++)
		{
			sensor_data->accel[axis] = ( (int16_t) (data[i+1]<<8)) |((int16_t) data[i]);
			i += 2;
		}
		sensor_data->valid_sensors |= SENSOR_ACCEL;
	}

	return true;
}

