/*
 * Copyright (c) 2015, Turingsense.
 * All rights reserved.
 *
 * cwati
 *
 * This project for the SENSOR MODUULE (22F) to talk to
 * Invensense.
 *
 * Porting code from Embedded Masters' code for 22F FRDM to sensor module.
 */
/*******************************************************************************
 * Standard C Included Files
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*******************************************************************************
 * SDK Included Files
 ******************************************************************************/
#include "board.h"
#include "fsl_uart_driver.h"
#include "fsl_edma_driver.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_os_abstraction.h"
#include "fsl_interrupt_manager.h"
#include "fsl_gpio_driver.h"

#include "fsl_gpio_common.h"
#include "fsl_uart_common.h"

/*******************************************************************************
 * Application Included Files
 ******************************************************************************/
#include "main.h"
#include "data_file.h"
#include "terminal.h"
#include "dspi.h"
#include "edma.h"

/*******************************************************************************
 * Invensense Embedded Motion Processing Library  Included Files
 ******************************************************************************/
#include "../inv_md6/mltypes.h"
#include "../inv_md6/inv_mpu.h"
#include "../inv_md6/mpl.h"
#include "../inv_md6/quaternion_supervisor.h"
#include "../inv_md6/fusion_9axis.h"
#include "../inv_md6/fast_no_motion.h"
#include "../inv_md6/gyro_tc.h"
#include "../inv_md6/compass_vec_cal.h"
#include "../inv_md6/mag_disturb.h"
#include "../inv_md6/eMPL_outputs.h"
#include "../inv_md6/hal_outputs.h"
#include "../inv_md6/data_builder.h"
#include "../inv_md6/ml_math_func.h"
#include "../inv_md6/packet.h"
#include "../inv_md6/log.h"
#include "../inv_md6/inv_mpu_dmp_motion_driver.h"

/*******************************************************************************
 * CWATI Included Files
 ******************************************************************************/
//#include "MPU9250_SPI.h"
#define MPU9250_READ_FLAG   	    0x80

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SPI0_TXCMD (SPI_PUSHR_PCS(0x01) | SPI_PUSHR_CTAS(0x00))

#define DEFAULT_MPU_HZ 20
#define COMPASS_READ_MS 100

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
/* Semaphore for checking eDMA channel completion. */
semaphore_t g_statusSem;
/* GPIO pins */
gpio_output_pin_user_config_t ledPinGreen;

/* DMA Buffers */
uint32_t g_masterRxBuffer[64];
uint32_t g_slaveRxBuffer[64];
volatile uint16_t g_slaveTxCount;
volatile uint32_t g_errorFlag;

/* Check buffers */
extern const uint32_t g_dataBuffer[];
extern const uint32_t g_expectedReturn[];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void pin_setup(void);

/*******************************************************************************
 * Code
 ******************************************************************************/

static volatile uint32_t mpu9250_int_count;
static volatile bool mpu9250_int_flag;
static void (*mpu9250_int_cb)(void);

enum _more_pins
{
	kMpu9250IntPin = GPIO_MAKE_PIN(HW_GPIOC, 11U),
};

static const gpio_input_pin_user_config_t mpu9250_int_pin =
{
		.pinName = kMpu9250IntPin,
		.config =
		{
				.isPullEnable = false,
				.interrupt = kPortIntFallingEdge,
		}
};

#if 1
void PORTC_IRQHandler(void)
#else
void mpu9250_isr(void)
#endif
{
	GPIO_DRV_ClearPinIntFlag(mpu9250_int_pin.pinName);
	mpu9250_int_count++;
	mpu9250_int_flag = true;
	if (mpu9250_int_cb)
		(*mpu9250_int_cb)();
}

 static void mpu9250_configure_irq_pin(void)
{
	 mpu9250_int_count = 0;
	 mpu9250_int_flag = false;
	 mpu9250_int_cb = NULL;
#if 0
	//OSA_InstallIntHandler(g_portIrqId[GPIO_EXTRACT_PORT(mpu9250_int_pin.pinName)], mpu9250_isr);
	INT_SYS_InstallHandler(g_portIrqId[GPIO_EXTRACT_PORT(mpu9250_int_pin.pinName)], mpu9250_isr);
#endif
	PORT_HAL_SetMuxMode(g_portBaseAddr[GPIO_EXTRACT_PORT(mpu9250_int_pin.pinName)],
						GPIO_EXTRACT_PIN(mpu9250_int_pin.pinName),
						kPortMuxAsGpio);
	GPIO_DRV_InputPinInit(& mpu9250_int_pin);
}

void mpu9250_set_irq_enable(bool v)
{
	if (v)
		INT_SYS_EnableIRQ(g_portIrqId[GPIO_EXTRACT_PORT(mpu9250_int_pin.pinName)]);
	else
		INT_SYS_DisableIRQ(g_portIrqId[GPIO_EXTRACT_PORT(mpu9250_int_pin.pinName)]);
}

void mpu9250_reg_int_cb(void (*cb)(void))
{
	// printf("registered interrupt handler\r\n");
	mpu9250_int_cb = cb;
}


// Note that individual reads and writes of a scalar type are atomic,
// so do not need to be in a critical section.
void mpu9250_wait_for_int(void)
{
	while (! mpu9250_int_flag)
		;
	mpu9250_int_flag = false;
}

/******************************************************************************/

bool uart_rx_data_avail(void)
{
	uint32_t baseAddr = g_uartBaseAddr[BOARD_DEBUG_UART_INSTANCE];
	return UART_HAL_IsRxDataRegFull(baseAddr);
}

uint8_t uart_rx_get_byte(void)
{
	uint32_t baseAddr = g_uartBaseAddr[BOARD_DEBUG_UART_INSTANCE];
	uint8_t c;
	UART_HAL_Getchar(baseAddr, & c);
	return c;
}

void config_scratchRegPins(void) {
	int tmp;

	/* GREEN */
	ledPinGreen.config.outputLogic = 0;
	ledPinGreen.config.slewRate = kPortSlowSlewRate;
	ledPinGreen.config.driveStrength = kPortHighDriveStrength;
	ledPinGreen.pinName = GPIO_MAKE_PIN(HW_GPIOA, 2);

	GPIO_DRV_OutputPinInit(&ledPinGreen);

	/* LEDs active low, will turn on */
    GPIO_DRV_ClearPinOutput(ledPinGreen.pinName);
}

#define BUFFER_SIZE 64
uint8_t mts_buffer[BUFFER_SIZE];
uint8_t stm_buffer[BUFFER_SIZE];

void write_reg(uint8_t reg_addr, uint8_t data)
{
	dspi_status_t Error;
	mts_buffer[0] = reg_addr;
	mts_buffer[1] = data;
	Error = DSPI_DRV_MasterTransferDataBlocking(kSensorInstance, &masterSpiDevice,
						mts_buffer, stm_buffer, 2, 1000);
}

void read_regs(uint8_t reg_addr, size_t count, uint8_t *data)
{
	dspi_status_t Error;
	mts_buffer[0] = 0x80 | reg_addr;
	memset(& mts_buffer[1], 0, count);  // not really necessary
	memset(stm_buffer, 0, count+1); // also not really necessary
	Error = DSPI_DRV_MasterTransferDataBlocking(kSensorInstance, &masterSpiDevice,
						mts_buffer, stm_buffer, count + 1, 1000);
	memcpy(data, & stm_buffer[1], count);
}

void write_mag_reg(uint8_t reg_addr, uint8_t data)
{
	write_reg(0x63, data);      // I2C_SLV0_DO
	write_reg(0x25, 0x0c);      // I2C_SLV0_ADDR: write, magnetometer address
	write_reg(0x26, reg_addr);  // I2C_SLV0_REG
	write_reg(0x27, 0x81);      // I2C_SLV0_CTRL: enable, one byte

	OSA_TimeDelay(30);			// delay for write to complete, should happen every 10 ms

	write_reg(0x27, 0x00);      // I2C_SLV0_CTRL: disable

	OSA_TimeDelay(30);          // delay for write to stop happening
}

// This function, which never returns, will configure the MPU-9250 and will print the raw sensor
// data to the debug serial output (actually over USB on the K22F Freedom board).  The output
// format is:

// XX -nnnnn -nnnnn -nnnnn -nnnnn -nnnnn -nnnnn -nnnnn XX -nnnnn -nnnnn -nnnnn XX
//
// ^  accelX accelY accelZ tenp   gyroX  gyroY  gyroZ  ^  magX   magY   magZ   ^
// int                                                 mag                     mag
// status                                              ST1                     ST2
//
// where values indicated as XX are hexadecimal and -nnnnn are decimal.
//
// WARNING: mag axes not oriented same as accel, gyro (data sheet section 9.1)
void dump_raw_data(void)
{
	write_reg(0x68, 0x07);  // signal path reset = gyro_rst + accel_rst + tmp_rst
	write_reg(0x6a, 0x07);  // user_ctl = fifo_rst + i2c_mst_rst + sig_cond_rst
								 // fifo_rst and i2c_mst_rst auto-clear after one clock cycle
								 // sig_cond_rst might also auto-clear; doc is unclear
	OSA_TimeDelay(10);
	write_reg(0x60, 0x00);  // clear signal path reset
	write_reg(0x6a, 0x20);  // user_ctl = I2C_MST_EN

	write_reg(0x19, 0x07);  // SMPLRT_DIV: divide by 8 (1 kHz results in 125 Hz data output)
	write_reg(0x1a, 0x06);  // CONFIG = 0 - FIFO_MODE disabled, EXT_SYNC_SET disabled,
	                        //    DPLF_CFG = 6
	                        //    (together with Fchoice=11b below,
	                        //       gyro bandwidth = 5 Hz, Fs = 1 kHz,
	                        //       temp bandwidth = 5 Hz
	write_reg(0x1b, 0x18);  // gyro sensitivity to +/- 2000 deg/s, self-test disabled, Fchoice = 11b
	write_reg(0x1c, 0x18);  // accel sensitivity to +/- 16g, self-test disabled
	write_reg(0x1d, 0x16);  // Fchoice = 1, A_DPLF_CFG = 6: bandwidth = 5 Hz, Fs = 1 kHz
	write_reg(0x37, 0x80);  // int active low, push-pull, 50 us pulse, cleared by reading INT_STATUS
    //    FSYNC pin does not cause interrupt, no I2C bypass
	write_reg(0x38, 0x01);  // enable data ready interrupt
	write_reg(0x6b, 0x00);  // PWR_MGMT_1 = 0
	write_reg(0x68, 0x80);  // I2C_MST_DELAY_CTRL = DELAY_ES_SHADOW
	write_reg(0x24, 0x50);  // I2C_MST_CTRL - not MULT_MST_EN, WAIT_FOR_ES, !SLV_3_FIFO_EN, I2C_MST_P_NSR, I2C_MST_CLK = 348 kHz

	write_reg(0x67, 0x80);  // I2C_MST_DELAY_CTRL:  DELAY_ES_SHADOW
	write_reg(0x34, 0x00);  // I2C_SLV4_CTRL: I2C_MST_DLY: read every sample

	write_mag_reg(0x0a, 0x12); // CNTL1: BIT (16-bit), continuous mode 2 (100 Hz)

	write_reg(0x25, 0x8c);  // I2C_SLV0_ADDR: read from 0x0c
	write_reg(0x26, 0x02);  // start at ST1
	write_reg(0x27, 0x88);  // I2C_SLV0_CTRL: enable auto read, length 8 bytes

	// mpu9250_set_irq_enable(true); // not necessary, enabled by GPIO_DRV_InputPinInit() called from mpu9250_configure_irq_pin().

	// This infinite loop waits for an interrupt, reads a sample, and outputs it to
	// the serial port.
	while (1)
	{
		uint8_t data[23];
		read_regs(0x3a, 23, data);

		mpu9250_wait_for_int();

		printf("%6u ", mpu9250_int_count);
		printf("%02x ", data[0]);  // INT_STATUS
		// ACCEL X, Y, Z, TEMP, GYRO X, Y, Z
		for (int i = 1; i < 15; i+=2)
		{
			int16_t d = (data[i]<<8) | data[i+1];
			printf("%6d ", d);
		}

		printf("%02x ", data[15]); // MAG STATUS1
		// MAG X, Y, Z - NOTE: mag axes not oriented same as accel, gyro (data sheet section 9.1)
		for (int i = 16; i < 22; i+=2)
		{
			int16_t d = data[i] | (data[i+1] << 8);
			printf("%6d ", d);
		}
		printf("%02x\r\n", data[22]); // MAG STATUS2

#if 1
		if (uart_rx_data_avail ())
			{
				uint8_t c = uart_rx_get_byte() & 0x7f;
				if ((c >= '0') && (c <= '9'))
					printf("Received a %c\r\n", c);
			}
#endif

#if 0
		OSA_TimeDelay(100);  // This delay introduced because full 125 Hz output rate
		                     // causes my development machine to get severely bogged down.
#endif
	}
}

#if 1
// AK8963 in MPU-9250
#define TEST_I2C_ADDR         0x0c
#define TEST_I2C_WAI_REG      0x00
#define TEST_I2C_WRITE_REG    0x0a  /* CNTL1 */
#define TEST_I2C_WRITE_VAL_A  0x02  /* continuous mode 1, 14-bit */
#define TEST_I2C_WRITE_VAL_B  0x12  /* continuous mode 1, 16-bit */
#else
// FXOS8700CQ on K22 Freedom board
#define TEST_I2C_ADDR         0x1c
#define TEST_I2C_WAI_REG      0x0d
#define TEST_I2C_WRITE_REG    0x09  /* F_SETUP */
#define TEST_I2C_WRITE_VAL_A  0x00
#define TEST_I2C_WRITE_VAL_B  0xff
#endif

void aux_i2c_test(void)
{
    i2c_status_t error;
    i2c_device_t i2c_slave =
    {
        .address = TEST_I2C_ADDR,   // FXOS8700CQ
        .baudRate_kbps = 400
    };
    uint8_t cmd[1];
    uint8_t rbuf[1];
    uint8_t wbuf[1];

	wbuf [0] = TEST_I2C_WRITE_VAL_A;

	while(1)
    {
    	cmd [0] = TEST_I2C_WAI_REG;
    	error = I2C_DRV_MasterReceiveDataBlocking(kMPU9250AuxI2CInstance,
    											  &i2c_slave,
    											  cmd,
    											  sizeof(cmd),
    											  rbuf,
    											  sizeof(rbuf),
    											  500);
    	printf("reading WAI: error %d, data 0x%02x\r\n", error, rbuf[0]);

    	cmd [0] = TEST_I2C_WRITE_REG;
    	error = I2C_DRV_MasterSendDataBlocking(kMPU9250AuxI2CInstance,
    										   &i2c_slave,
    										   cmd,
    										   sizeof(cmd),
    										   wbuf,
    										   sizeof(wbuf),
    										   500);
    	printf("writing 0x%02x data 0x%02x: error %d\r\n", cmd[0], wbuf[0], error);

    	error = I2C_DRV_MasterReceiveDataBlocking(kMPU9250AuxI2CInstance,
    										   	   &i2c_slave,
    										   	   cmd,
    										   	   sizeof(cmd),
    										   	   rbuf,
    										   	   sizeof(rbuf),
    										   	   500);
    	printf("read 0x%02x: error %d data 0x%02x\r\n", cmd[0], error, rbuf[0]);

    	wbuf [0] ^= (TEST_I2C_WRITE_VAL_A ^ TEST_I2C_WRITE_VAL_B);

    	OSA_TimeDelay(1000);
    }
}

int get_tick_count(unsigned long *count)
{
	*count = OSA_TimeGetMsec();
	return 0;
}

#define TEMP_READ_MS    (500)

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

struct rx_s
{
	unsigned char header[3];
	unsigned char cmd;
};

// debugging features
#define COUNT_RAW_SAMPLES
#define DMP_QUAT_TEST

struct hal_s
{
	volatile unsigned char new_gyro;
	unsigned int report;
	struct rx_s rx;
	unsigned char dmp_on;
	unsigned short dmp_features;
#ifdef DMP_QUAT_TEST
	long lq[4];
#endif
#ifdef COUNT_RAW_SAMPLES
	unsigned long quat_count;
	unsigned long accel_count;
	unsigned long gyro_count;
	unsigned long mag_count;
	unsigned long temp_count;
	unsigned long execute_on_data_count;
#endif
};

static struct hal_s hal;

static void setup_gyro(void)
{
	unsigned char mask = INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_XYZ_COMPASS;
	mpu_set_sensors(mask);
	mpu_configure_fifo(mask);
}

void send_status_compass() {
	long data[3] = { 0 };
	int8_t accuracy = { 0 };
	unsigned long timestamp;
	inv_get_compass_set(data, &accuracy, (inv_time_t*) &timestamp);
	MPL_LOGI("Compass: %7.4f %7.4f %7.4f ",
			data[0]/65536.f, data[1]/65536.f, data[2]/65536.f);
	MPL_LOGI("Accuracy= %d\r\n", accuracy);

}

static void read_from_mpl(void)
{
	long msg, data[9];
	int8_t accuracy;
	unsigned long timestamp;
	float float_data[3] = {0};

	if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*) &timestamp))
	{
		eMPL_send_quat(data);
		if (hal.report & PRINT_QUAT)
			eMPL_send_data(PACKET_DATA_QUAT, data);
	}
	if ((hal.report & PRINT_ACCEL) && inv_get_sensor_type_accel(data, &accuracy, (inv_time_t*) &timestamp))
		eMPL_send_data(PACKET_DATA_ACCEL, data);
	if ((hal.report & PRINT_GYRO) && inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t*) &timestamp))
		eMPL_send_data(PACKET_DATA_GYRO, data);
	if ((hal.report & PRINT_COMPASS) && inv_get_sensor_type_compass(data, &accuracy, (inv_time_t*) &timestamp))
		eMPL_send_data(PACKET_DATA_COMPASS, data);
	if ((hal.report & PRINT_EULER) && inv_get_sensor_type_euler(data, &accuracy, (inv_time_t*) &timestamp))
		eMPL_send_data(PACKET_DATA_EULER, data);
	if ((hal.report & PRINT_ROT_MAT) && inv_get_sensor_type_rot_mat(data, &accuracy, (inv_time_t*) &timestamp))
		eMPL_send_data(PACKET_DATA_ROT, data);
	if ((hal.report & PRINT_HEADING) && inv_get_sensor_type_heading(data, &accuracy, (inv_time_t*) &timestamp))
		eMPL_send_data(PACKET_DATA_HEADING, data);
	if ((hal.report & PRINT_LINEAR_ACCEL) && inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*) &timestamp))
		MPL_LOGI("Linear Accel: %7.5f %7.5f %7.5f\r\n", float_data[0], float_data[1], float_data[2]);
	// XXX no pedometer and motion events
}

static void handle_input(char c)
{
	switch (c)
	{
	case 'a': hal.report ^= PRINT_ACCEL;        break;
	case 'g': hal.report ^= PRINT_GYRO;         break;
	case 'c': hal.report ^= PRINT_COMPASS;      break;
	case 'e': hal.report ^= PRINT_EULER;        break;
	case 'r': hal.report ^= PRINT_ROT_MAT;      break;
	case 'q': hal.report ^= PRINT_QUAT;         break;
	case 'h': hal.report ^= PRINT_HEADING;      break;
	case 'i': hal.report ^= PRINT_LINEAR_ACCEL; break;
	case 'w': send_status_compass();            break;
	case 'd': mpu_reg_dump();                   break;
#if 0
	case '1': mpu_set_sample_rate(10);  inv_set_gyro_sample_rate(100000L); inv_set_accel_sample_rate(100000L); break;
	case '2': mpu_set_sample_rate(20);  inv_set_gyro_sample_rate( 50000L); inv_set_accel_sample_rate( 50000L); break;
	case '3': mpu_set_sample_rate(40);  inv_set_gyro_sample_rate( 25000L); inv_set_accel_sample_rate( 25000L); break;
	case '4': mpu_set_sample_rate(50);  inv_set_gyro_sample_rate( 20000L); inv_set_accel_sample_rate( 20000L); break;
	case '5': mpu_set_sample_rate(100); inv_set_gyro_sample_rate( 10000L); inv_set_accel_sample_rate( 10000L); break;
#endif
#ifdef COUNT_RAW_SAMPLES
	case 'z': MPL_LOGI("Sample counts: %lu quat, %lu accel, %lu gyro, %lu mag, %lu temp, %lu execute\r\n", hal.quat_count, hal.accel_count, hal.gyro_count, hal.mag_count, hal.temp_count, hal.execute_on_data_count);
#ifdef DMP_QUAT_TEST
		MPL_LOGI("Last quat: [%08x, %08x, %08x, %08x]\r\n", hal.lq[0], hal.lq[1], hal.lq[2], hal.lq[3]);
#endif /* DMP_QUAT_TEST */
#endif
	default:
		break;
	}
	hal.rx.cmd = 0;
}

void gyro_data_ready_cb(void)
{
	hal.new_gyro = 1;
}

const unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

struct platform_data_s {
    signed char orientation[9];
};

static const struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

static const struct platform_data_s compass_pdata = {
	.orientation = { 0, 1, 0,
					 1, 0, 0,
					 0, 0, -1}
};

static void tap_cb(unsigned char direction, unsigned char count)
{
    switch (direction) {
    case TAP_X_UP:
        MPL_LOGI("Tap X+ ");
        break;
    case TAP_X_DOWN:
        MPL_LOGI("Tap X- ");
        break;
    case TAP_Y_UP:
        MPL_LOGI("Tap Y+ ");
        break;
    case TAP_Y_DOWN:
        MPL_LOGI("Tap Y- ");
        break;
    case TAP_Z_UP:
        MPL_LOGI("Tap Z+ ");
        break;
    case TAP_Z_DOWN:
        MPL_LOGI("Tap Z- ");
        break;
    default:
        return;
    }
    MPL_LOGI("x%d\n", count);
    return;
}

static void android_orient_cb(unsigned char orientation)
{
	switch (orientation) {
	case ANDROID_ORIENT_PORTRAIT:
        MPL_LOGI("Portrait\n");
        break;
	case ANDROID_ORIENT_LANDSCAPE:
        MPL_LOGI("Landscape\n");
        break;
	case ANDROID_ORIENT_REVERSE_PORTRAIT:
        MPL_LOGI("Reverse Portrait\n");
        break;
	case ANDROID_ORIENT_REVERSE_LANDSCAPE:
        MPL_LOGI("Reverse Landscape\n");
        break;
	default:
		return;
	}
}

static void mpl_init()
{
	struct int_param_s int_param;
	inv_error_t result;

	memset(& hal, 0, sizeof(hal));

	result = mpu_init(& int_param);
	if (result)
	{
		printf("Could not initialize MPU-9250.\n");
		while (true) { }
	}

	// I thought mpu_init() would register the interrupt handler; isn't that
	// what int_param is for? But it doesn't.
	mpu9250_reg_int_cb(&gyro_data_ready_cb);

	result = inv_init_mpl();
	if (result)
	{
		printf("Could not initialize MPL.\n");
		while (true) { }
	}

    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
    inv_enable_fast_nomot();
    inv_enable_gyro_tc();
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();
    inv_enable_eMPL_outputs();

    result = inv_start_mpl();
	if (result)
	{
		printf("Could not start the MPL.\n");
		while (true) { }
	}

	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);

	mpu_set_sample_rate(DEFAULT_MPU_HZ);

#undef COMPASS_TIMING_HACK

	#ifdef COMPAS_TIMING_HACK
	// XXX temporarily use same compass rate as accel & gyro,
	// see comment further down about new_compass = 1
	mpu_set_compass_sample_rate(DEFAULT_MPU_HZ);
#else
	mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif

	mpu_set_accel_fsr(16); // g
	mpu_set_lpf(5); // Hz

	// read back config for verification
	unsigned short gyro_rate;
	unsigned char accel_fsr;
	unsigned short gyro_fsr, compass_fsr;

	mpu_get_sample_rate(&gyro_rate);
	mpu_get_gyro_fsr(&gyro_fsr);
	mpu_get_accel_fsr(&accel_fsr);
	mpu_get_compass_fsr(&compass_fsr);
#if 0
	printf("sample rate: %d Hz, accel_fsr: %d g, gyro_fsr: %d dps, compass_fsr: %d\r\n", gyro_rate, accel_fsr, gyro_fsr, compass_fsr);
#endif

	inv_set_gyro_sample_rate(1000000L / gyro_rate);
	inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_TIMING_HACK
	// XXX temporarily use same compass rate as accel & gyro,
	// see comment further down about new_compass = 1
	inv_set_compass_sample_rate(1000000L / gyro_rate);
#else
	inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif

#if 0
	long dbg_accel_rate, dbg_gyro_rate, dbg_compass_rate;
	inv_get_accel_sample_rate_ms(& dbg_accel_rate);
	inv_get_gyro_sample_rate_ms(& dbg_gyro_rate);
	inv_get_compass_sample_rate_ms(& dbg_compass_rate);
	printf("accel %ld ms, gyro %ld ms, compass %ld ms\r\n", dbg_accel_rate, dbg_gyro_rate, dbg_compass_rate);
#endif

	// Set chip-to-body orientation matrix.
	// Set hardware units to dps/g's/degrees scaling factor.

	inv_set_gyro_orientation_and_scale(
			inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long)gyro_fsr<<15);

	inv_set_accel_orientation_and_scale(
			inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long)accel_fsr<<15);

	inv_set_compass_orientation_and_scale(
			inv_orientation_matrix_to_scalar(compass_pdata.orientation),
			(long)compass_fsr<<15);

    dmp_load_motion_driver_firmware();
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);

    hal.dmp_features = (DMP_FEATURE_6X_LP_QUAT |
    					DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT |
    				    DMP_FEATURE_SEND_RAW_ACCEL |
    				    DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_CAL_GYRO);
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    inv_set_quat_sample_rate(1000000L / DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
}

static void mpl_demo(void)
{
	unsigned char new_compass = 0;
	unsigned char new_temp = 0;
	unsigned long timestamp;
	unsigned long next_temp_ms = 0;
	unsigned long next_compass_ms = 0;

	hal.rx.cmd = 0;

	while (1)
	{
		unsigned long sensor_timestamp;
		int new_data = 0;
		long temperature;
		unsigned char more;
		unsigned short sensors;

		short gyro[3], accel_short[3], compass_short[3];  // mpu_read_fifo() provides gyro, accel, compass as shorts
		long quat[4];
		long accel[3], compass[3];  // but inv_build_accel() and inv_build_compass() want longs

		if (uart_rx_data_avail())
		{
			char c = (char) uart_rx_get_byte();
			handle_input(c);
		}
		get_tick_count(&timestamp);

        /* We're not using a data ready interrupt for the compass, so we'll
         * make our compass reads timer-based instead.
         */
#ifdef COMPASS_TIMING_HACK
		// This is a quick hack because I found that the Freescale OSA_TimeGetMsec() function
		// I've used to implement get_tick_count() is only returning a 16-bit value,
		// despite the use of an unsigned long data type. Argh!
		// XXX better fix needed!
		new_compass = 1;
#else
		if ((timestamp > next_compass_ms) && hal.new_gyro)
        {
            next_compass_ms = timestamp + COMPASS_READ_MS;
            new_compass = 1;
        }
#endif

		if (timestamp > next_temp_ms)
		{
			next_temp_ms = timestamp + TEMP_READ_MS;
			new_temp = 1;
		}

		if (!hal.new_gyro)
			continue;

		if (hal.dmp_on)
			dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
		else
		{
			unsigned char s2;
			mpu_read_fifo(gyro, accel_short, & sensor_timestamp, & s2, & more);
			sensors = s2;
		}

		if (!more)
			hal.new_gyro = 0;

		if (sensors & INV_XYZ_GYRO)
		{
			inv_build_gyro(gyro, sensor_timestamp);
	#ifdef COUNT_RAW_SAMPLES
			hal.gyro_count++;
	#endif
			new_data = 1;
			if (new_temp)
			{
				new_temp = 0;
				mpu_get_temperature(&temperature, &sensor_timestamp);
				inv_build_temp(temperature, sensor_timestamp);
	#ifdef COUNT_RAW_SAMPLES
				hal.temp_count++;
	#endif
			}
		}

		if (sensors & INV_XYZ_ACCEL)
		{
			accel[0] = (long)accel_short[0];
			accel[1] = (long)accel_short[1];
			accel[2] = (long)accel_short[2];
			inv_build_accel(accel, 0, sensor_timestamp);
		#ifdef COUNT_RAW_SAMPLES
			hal.accel_count++;
		#endif
			new_data = 1;
		}

		if (hal.dmp_on && (sensors & INV_WXYZ_QUAT))
		{
			inv_build_quat(quat, 0, sensor_timestamp);
#ifdef COUNT_RAW_SAMPLES
			hal.quat_count++;
#endif
#ifdef DMP_QUAT_TEST
			memcpy(hal.lq, quat, sizeof(hal.lq));
#endif
			new_data = 1;
		}

		if (new_compass)
		{
			new_compass = 0;
			if (!mpu_get_compass_reg(compass_short, & sensor_timestamp))
			{
				compass[0] = (long)compass_short[0];
				compass[1] = (long)compass_short[1];
				compass[2] = (long)compass_short[2];
				inv_build_compass(compass, 0, sensor_timestamp);
#ifdef COUNT_RAW_SAMPLES
				hal.mag_count++;
#endif
				new_data = 1;
			}
		}

		if (new_data)
		{
			inv_execute_on_data();
#ifdef COUNT_RAW_SAMPLES
			hal.execute_on_data_count++;
#endif
			read_from_mpl();
		}
	}
}

int main(void) {
	uint32_t userFreq = 1000000;  /* 1 MHz */
	dspi_master_state_t dspiMasterSensorState;
	i2c_master_state_t mpu9250AuxI2CState;
	hardware_init();
	OSA_Init();

	//dspiMasterSensorState =	(dspi_master_state_t *) OSA_MemAlloc(sizeof(dspi_master_state_t));

    /* Configure the UART TX/RX pins */
    configure_uart_pins(BOARD_DEBUG_UART_INSTANCE);

    /* Call this function to initialize the console UART.  This function
       enables the use of STDIO functions (printf, scanf, etc.) */
    dbg_uart_init();

    // printf("\r\nTuringSense\r\n");

	configure_spi_pins(kSensorInstance);
	cwati_dspi_edma_master_setup(&dspiMasterSensorState, kSensorInstance,
			userFreq, 8);

	configure_i2c_pins(kMPU9250AuxI2CInstance);
	I2C_DRV_MasterInit(kMPU9250AuxI2CInstance, &mpu9250AuxI2CState);

	config_scratchRegPins();  /* Turn on green LED */

	mpu9250_configure_irq_pin();

	// INT_SYS_EnableIRQGlobal();  // not necessary, already enabled

#if 0
	dump_raw_data();  // For raw data debug only - WARNING - never returns!
#endif

#if 0
	aux_i2c_test(); // WARNING - never returns!
#endif

	mpl_init();

	mpl_demo();  // For use with python demo - WARNING - never returns!
}

/******************************************************************************
 * EOF
 ******************************************************************************/
