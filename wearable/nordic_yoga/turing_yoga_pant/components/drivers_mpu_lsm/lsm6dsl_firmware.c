/*
 * Copyright (c) 2017 TuringSense
 *
 * lsm6dsl_firmware.c
 *
 * Interface to hw API talking to STM LSM6DSL sensor
 */


#include "lsm6dsl_firmware.h"
#include "nrf_delay.h"

extern void set_mpu9250_sensor_num(uint8_t num);

bool lsm6dsl_init_regs(void) {
	  // GYROSCOPE INITIALIZATION

	  /* TODO TODO Change the registers into macros TODO TODO */

	  /* Enable register address automatically incremented during a multiple byte
	     access with a serial interface */
	  /* Enable BDU */
	  write_reg(LSM6DSL_CTRL3_C,0x44);

	  /* FIFO mode selection -> BYPASS*/
	  write_reg(LSM6DSL_FIFO_CTRL5,0x00);

	  /* Output data rate selection - power down */
	  /* Full scale selection -> FS_HIGH*/
	  write_reg(LSM6DSL_CTRL2_G,0x5C);

	  write_reg(LSM6DSL_CTRL1_XL,0x55);
	
	
		// To enable the magnetometer 
		write_reg(LSM6DSL_CTRL10_C, 0x04); //Enable embedded functionalities (pedometer, tilt, significant motion detection,  --> sensor hub <-- IMP and ironing)
		write_reg(LSM6DSL_MASTER_CONFIG, 0x0B); //Sensor hub I2C master enable

	return true;
}


bool lsm6dsl_detect_sensor(uint8_t sensor_num, uint8_t* whoami) {
	uint8_t data[LSM6DSL_READ_SIZE];

	*whoami = 0;

	set_mpu9250_sensor_num(sensor_num);
	read_regs(LSM6DSL_REG_WHOAMI, 1, data);
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
	sensor_data->timestamp = 0;       //OSA_TimeGetMsec();  CT-- Commented the time part.

	read_regs(0x22, 12, data);

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


void write_stm_mag_reg(uint8_t reg_addr, uint8_t data)
{
	

	

	
}

uint8_t read_stm_mag_reg(uint8_t reg_addr)
{
	uint8_t data;

	
	write_reg(LSM6DSL_CTRL2_G,0x0C);          //   Gyro power down
	write_reg(LSM6DSL_CTRL1_XL,0x05);         //   Accel power down
	
	// Reason for power down because we can only access the slave sensors if the accel and gyro are power down.
	
	write_reg(LSM6DSL_FUNC_CFG_ACCESS,   0x80); //Enable access to the embedded functions configuration registers bank A
	nrf_delay_ms(10);
	write_reg(LSM6DSL_SLAVE0_CONFIG,  0x01);
	write_reg(LSM6DSL_SLV0_ADD, LIS2MDL_SLAVE_ADDRESS_WRITE);   // write, magnetometer address
	//read_regs(LSM6DSL_SLV0_ADD, 1 , &data);
	data = 0;
	write_reg(LSM6DSL_SLV0_SUBADD,  reg_addr);
	//write_reg(LSM6DSL_DATAWRITE_SRC_MODE_SUB_SLV0, reg_addr);
	write_reg(LSM6DSL_SLV0_ADD, LIS2MDL_SLAVE_ADDRESS_READ);      // write, magnetometer address
	write_reg(LSM6DSL_FUNC_CFG_ACCESS,   0x00);                   //Disable access to the embedded functions configuration registers bank A
	nrf_delay_ms(10);
	data = 0;
	write_reg(LSM6DSL_CTRL2_G,0x5C);         //   Gyro power on
	write_reg(LSM6DSL_CTRL1_XL,0x55);         //   Accel power on
	read_regs(LSM6DSL_SENSORHUB1_REG, 1, &data);
	nrf_delay_ms(10);
	data = 0;
	read_regs(LSM6DSL_FUNC_SRC1, 1 , &data);
	read_regs(LSM6DSL_FUNC_SRC2, 1 , &data);

	

	

	
	return data;
}


uint8_t  detect_stm_mag_sensor(void)
{
	uint8_t data = read_stm_mag_reg(LIS2MDL_WHO_AM_I_REG);
	return data;
}

