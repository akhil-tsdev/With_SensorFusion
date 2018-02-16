/*
 * Copyright (c) 2017 TuringSense
 * mpu9250_maxwell.c to be used with KSDK 1.3.0
 *
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

//#include "fsl_dspi_master_driver.h"
#include "fsl_debug_console.h"
#include "fsl_dspi_shared_function.h"
//#include "fsl_gpio_common.h"  /* not needed with KSDK 1.1.0 ? */
#include "fsl_interrupt_manager.h"
#include "fsl_os_abstraction.h"
#include "fsl_uart_driver.h"

//#define DSPI_DRV_MasterTransferBlocking DSPI_DRV_MasterTransferDataBlocking  /* fn name changed in KSDK 1.1.0 */

#include "board.h"
#include "sat_module.h"
#include "ts_rtc.h"
#include "main_22f_on_board_diag_maxwell.h"
#include "ts_fusion.h"
/* MPU9250 header files are separated into 2 files,
 * one for firmware-related and the other one for
 * purely OSSF functionality.
 */
#include "mpu9250_maxwell.h"
#include "mpu9250_ossf.h"
#include "ts_fusion.h"
#include "tasks.h"
#define MPU9250_VERIFY_REG_WRITES 0

unsigned short	mag_rate = TS_MAG_RATE;	/* Compass rate */

#if GYRO_BANDWIDTH == 5
#  define FCHOICE_B 0
#  define DPLF_CFG 6
#elif GYRO_BANDWIDTH == 10
#  define FCHOICE_B 0
#  define DPLF_CFG 5
#elif GYRO_BANDWIDTH == 20
#  define FCHOICE_B 0
#  define DPLF_CFG 4
#elif GYRO_BANDWIDTH == 41
#  define FCHOICE_B 0
#  define DPLF_CFG 3
#elif GYRO_BANDWIDTH == 92
#  define FCHOICE_B 0
#  define DPLF_CFG 2
#elif GYRO_BANDWIDTH == 184
#  define FCHOICE_B 0
#  define DPLF_CFG 1
#else
#  error "invalid GYRO_BANDWIDTH"
#endif

#if GYRO_RANGE_DPS == 250
#  define GYRO_FS_SEL 0
#elif GYRO_RANGE_DPS == 500
#  define GYRO_FS_SEL 1
#elif GYRO_RANGE_DPS == 1000
#  define GYRO_FS_SEL 2
#elif GYRO_RANGE_DPS == 2000
#  define GYRO_FS_SEL 3
#else
#  error "invalid GYRO_RANGE_DPS"
#endif

#if ACCEL_BANDWIDTH == 5
#  define ACCEL_FCHOICE_B 0
#  define A_DPLF_CFG 6
#elif ACCEL_BANDWIDTH == 10
#  define ACCEL_FCHOICE_B 0
#  define A_DPLF_CFG 5
#elif ACCEL_BANDWIDTH == 20
#  define ACCEL_FCHOICE_B 0
#  define A_DPLF_CFG 4
#elif ACCEL_BANDWIDTH == 41
#  define ACCEL_FCHOICE_B 0
#  define A_DPLF_CFG 3
#elif ACCEL_BANDWIDTH == 92
#  define ACCEL_FCHOICE_B 0
#  define A_DPLF_CFG 2
#elif ACCEL_BANDWIDTH == 184
#  define ACCEL_FCHOICE_B 0
#  define A_DPLF_CFG 1
#else
#  error "invalid ACCEL_BANDWIDTH"
#endif

#if ACCEL_RANGE_G == 2
#  define ACCEL_FS_SEL 0
#elif ACCEL_RANGE_G == 4
#  define ACCEL_FS_SEL 1
#elif ACCEL_RANGE_G == 8
#  define ACCEL_FS_SEL 2
#elif ACCEL_RANGE_G == 16
#  define ACCEL_FS_SEL 3
#else
#  error "invalid ACCEL_RANGE_G"
#endif

#if 1
#define MAG_SUBSAMPLE 1
#else
#define MAG_SUBSAMPLE (GYRO_ACCEL_RATE/MAG_RATE)
#endif

static volatile bool running[MAX_SENSORS_IN_CS];
static volatile bool self_test;
static volatile bool data_avail;

#if DO_TEMP_COMP
typedef struct { float offset; float scale; } temp_comp_t;
static temp_comp_t gyro_temp_comp[3];
static temp_comp_t accel_temp_comp[3];
#endif

static uint8_t ak8963_sensitivity_cal[3];  // NOTE - in magnetometer coordinate system!

#define BUFFER_SIZE 65
static uint8_t mts_buffer[BUFFER_SIZE];
static uint8_t stm_buffer[BUFFER_SIZE];

uint8_t maxwell_select = 0;
const uint8_t invalid_maxwell_num = 0;

enum _more_pins
{
#if PROTO_BOARD
	kMpu9250SsPin   = GPIO_MAKE_PIN(HW_GPIOD,  4U),
	kMpu9250MosiPin = GPIO_MAKE_PIN(HW_GPIOD,  6U),
	kMpu9250IntPin  = GPIO_MAKE_PIN(HW_GPIOD,  3U), /* Proto Sensor module uses PTD3 for interrupt */
#else
	/* PRODUCTION1 or Eval Board */
	kMpu9250SsPin   = GPIO_MAKE_PIN(GPIOC_IDX,  4U),
	kMpu9250MosiPin = GPIO_MAKE_PIN(GPIOC_IDX,  6U),
	kMpu9250IntPin  = GPIO_MAKE_PIN(GPIOC_IDX,  8U), /* Production Sensor module uses PTC8 for interrupt */

	kMpu9250Cs1Pin  = GPIO_MAKE_PIN(GPIOD_IDX,  4U),
	kMpu9250Cs2Pin  = GPIO_MAKE_PIN(GPIOD_IDX,  5U),
	kMpu9250Cs3Pin  = GPIO_MAKE_PIN(GPIOD_IDX,  6U),
#endif
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

const dspi_device_t sensor_dspi_device =
{
	.bitsPerSec = 1000000L,
	.dataBusConfig.bitsPerFrame = 8,
	.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh,
	.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge,
	.dataBusConfig.direction = kDspiMsbFirst,
};

static dspi_master_state_t sensor_dspi_master_state;

#if PIVOT_3_0

static const dspi_master_user_config_t sensor_dspi_master_user_config1 =
{
	.whichCtar = kDspiCtar0,
	.whichPcs = kDspiPcs1,
	.pcsPolarity = kDspiPcs_ActiveLow,
	.isSckContinuous = false,
	.isChipSelectContinuous = true,
};
#endif /* PIVOT_3_0 */

#if PROTO_BOARD
void PORTD_IRQHandler(void)
{
	GPIO_DRV_ClearPinIntFlag(mpu9250_int_pin.pinName);
	data_avail = true;
}

void SPI1_IRQHandler(void)	/* sensor module uses SPI1 for invensense */
{
    DSPI_DRV_IRQHandler(HW_SPI1);
}
#else
/* PRODUCTION1 or eval board */
void PORTC_IRQHandler(void)
{
	GPIO_DRV_ClearPinIntFlag(mpu9250_int_pin.pinName);
	data_avail = true;
}

void SPI0_IRQHandler(void)	/* sensor module uses SPI1 for invensense */
{
    DSPI_DRV_IRQHandler(0); /* SPI0 */
}
#endif
//
//void mpu9250_set_irq_enable(bool v)
//{
//	if (v)
//		INT_SYS_EnableIRQ(g_portIrqId[GPIO_EXTRACT_PORT(mpu9250_int_pin.pinName)]);
//	else
//		INT_SYS_DisableIRQ(g_portIrqId[GPIO_EXTRACT_PORT(mpu9250_int_pin.pinName)]);
//}

void sensor_write_reg(uint8_t reg_addr, uint8_t data)
{
	dspi_status_t Error;

	mts_buffer[0] = reg_addr;
	mts_buffer[1] = data;

	/* Do not use 0 for now! maxwell_select cs0 will pick nobody.
	   I find that we have to toggle the CS to make the read reliable. */
	uint8_t tmp_cs = 0;
	UART_DRV_SendDataBlocking(REV_F_UART_INSTANCE, &tmp_cs, 1, 1000u);

	UART_DRV_SendDataBlocking(REV_F_UART_INSTANCE, &maxwell_select, 1, 1000u);

	Error = DSPI_DRV_MasterTransferBlocking(kSensorSpiInstance,
											& sensor_dspi_device,
											mts_buffer,
											stm_buffer,
											2,
											1000);

#if MPU9250_VERIFY_REG_WRITES
	uint8_t readback;

	readback = sensor_read_reg(reg_addr);
	if (readback != data)
	{
		printf("MPU9250 write fail, reg %02x, wrote %02x, read back %02x\r\n", reg_addr, data, readback);
		while (1)
		{ ; }
	}
#endif
}

void sensor_read_regs(uint8_t reg_addr, size_t count, uint8_t *data)
{
	dspi_status_t Error;
	mts_buffer[0] = 0x80 | reg_addr;
	memset(& mts_buffer[1], 0, count);  // not really necessary
	memset(stm_buffer, 0, count+1); // also not really necessary

	/* Do not use 0 for now! maxwell_select cs0 will pick nobody.
	   I find that we have to toggle the CS to make the read reliable. */
	uint8_t tmp_cs = 0;
	UART_DRV_SendDataBlocking(REV_F_UART_INSTANCE, &tmp_cs, 1, 1000u);

	UART_DRV_SendDataBlocking(REV_F_UART_INSTANCE, &maxwell_select, 1, 1000u);

	Error = DSPI_DRV_MasterTransferBlocking(kSensorSpiInstance,
											& sensor_dspi_device,
											mts_buffer,
											stm_buffer,
											count + 1,
											1000);
	memcpy(data, & stm_buffer[1], count);
}

static uint8_t sensor_read_reg(uint8_t reg_addr)
{
	uint8_t data;

	sensor_read_regs(reg_addr, 1, & data);
	return data;
}

// some registers such as MPU9250_REG_SIGNAL_PATH_RESET and MPU9250_REG_USER_CTRL won't read back
// the written values, so don't try.
static void sensor_write_reg_no_verify(uint8_t reg_addr, uint8_t data)
{
	dspi_status_t Error;

	mts_buffer[0] = reg_addr;
	mts_buffer[1] = data;

	Error = DSPI_DRV_MasterTransferBlocking(kSensorSpiInstance,
											& sensor_dspi_device,
											mts_buffer,
											stm_buffer,
											2,
											1000);
}

void read_reg_cwati () {
	unsigned char mts_buffer[128];
	unsigned char stm_buffer[128];
	for (int i = 0; i < 128; i++) {
		mts_buffer[i] = i | 0x80;
		stm_buffer[i] = 0x0;
	}
//	static uint32_t counter = 0;
	dspi_status_t Error;

	while(1) {
		Error = DSPI_DRV_MasterTransferBlocking(kSensorSpiInstance, &sensor_dspi_device,
					&mts_buffer[0], &stm_buffer[0], 128, 1000);

		OSA_TimeDelay(20.0 /*ms*/);
	}
}

static void write_mag_reg(uint8_t reg_addr, uint8_t data)
{
	sensor_write_reg(MPU9250_REG_I2C_SLV4_DO,   data);
	sensor_write_reg(MPU9250_REG_I2C_SLV4_ADDR, 0x0c);      // write, magnetometer address
	sensor_write_reg(MPU9250_REG_I2C_SLV4_REG,  reg_addr);
	sensor_write_reg(MPU9250_REG_I2C_SLV4_CTRL, 0x80+(MAG_SUBSAMPLE-1));      // enable

	OSA_TimeDelay(30);			// delay for write to complete, should happen every 10 ms

	sensor_write_reg(MPU9250_REG_I2C_SLV4_CTRL, (MAG_SUBSAMPLE-1));      // disable

	OSA_TimeDelay(30);          // delay for write to stop happening
}

//static uint8_t read_mag_reg0(uint8_t reg_addr)
//{
//	uint8_t data;
//
//	sensor_write_reg(MPU9250_REG_I2C_SLV0_ADDR, 0x8c);      // read, magnetometer address
//	sensor_write_reg(MPU9250_REG_I2C_SLV0_REG,  reg_addr);
//	sensor_write_reg(MPU9250_REG_I2C_SLV0_CTRL, 0x80+(MAG_SUBSAMPLE-1));      // enable
//
//	OSA_TimeDelay(30);			// delay for reaad to complete, should happen every 10 ms
//
//	sensor_read_regs(MPU9250_REG_I2C_SLV0_DI, 1, & data);
//
//	sensor_write_reg(MPU9250_REG_I2C_SLV0_CTRL, (MAG_SUBSAMPLE-1));      // disable
//
//	OSA_TimeDelay(30);          // delay for read to stop happening
//
//	return data;
//}
static uint8_t read_mag_reg_fast(uint8_t reg_addr)
{
	uint8_t data;

	sensor_write_reg(MPU9250_REG_I2C_SLV4_ADDR, 0x8c);      // read, magnetometer address
	sensor_write_reg(MPU9250_REG_I2C_SLV4_REG,  reg_addr);
	sensor_write_reg(MPU9250_REG_I2C_SLV4_CTRL, 0x80+(MAG_SUBSAMPLE-1));      // enable

	OSA_TimeDelay(10);			// delay for reaad to complete, should happen every 10 ms

	sensor_read_regs(MPU9250_REG_I2C_SLV4_DI, 1, & data);

	sensor_write_reg(MPU9250_REG_I2C_SLV4_CTRL, (MAG_SUBSAMPLE-1));      // disable

	OSA_TimeDelay(10);          // delay for read to stop happening

	return data;
}
static uint8_t read_mag_reg(uint8_t reg_addr)
{
	uint8_t data;

	sensor_write_reg(MPU9250_REG_I2C_SLV4_ADDR, 0x8c);      // read, magnetometer address
	sensor_write_reg(MPU9250_REG_I2C_SLV4_REG,  reg_addr);
	sensor_write_reg(MPU9250_REG_I2C_SLV4_CTRL, 0x80+(MAG_SUBSAMPLE-1));      // enable

	OSA_TimeDelay(30);			// delay for reaad to complete, should happen every 10 ms

	sensor_read_regs(MPU9250_REG_I2C_SLV4_DI, 1, & data);

	sensor_write_reg(MPU9250_REG_I2C_SLV4_CTRL, (MAG_SUBSAMPLE-1));      // disable

	OSA_TimeDelay(30);          // delay for read to stop happening

	return data;
}

static bool mpu9250_configure_spi(void)
{
	dspi_status_t status;
	uint32_t calculatedBaudRate;
	uint32_t calculatedPcsToSck, calculatedLastSckToPcs, calculatedAfterTransfer;

	status = DSPI_DRV_MasterInit(kSensorSpiInstance, & sensor_dspi_master_state, & sensor_dspi_master_user_config1);
	if (status != kStatus_DSPI_Success)
		return false;

    status = DSPI_DRV_MasterConfigureBus(kSensorSpiInstance, & sensor_dspi_device, & calculatedBaudRate);
	if (status != kStatus_DSPI_Success)
		return false;

    status = DSPI_DRV_MasterSetDelay(kSensorSpiInstance,
    								 kDspiPcsToSck,
    								 500, // delayInNanoSec
    								 & calculatedPcsToSck);
	if (status != kStatus_DSPI_Success)
		return false;

    status = DSPI_DRV_MasterSetDelay(kSensorSpiInstance,
    								 kDspiLastSckToPcs,
    								 500, // delayInNanoSec
    								 & calculatedLastSckToPcs);
	if (status != kStatus_DSPI_Success)
		return false;

	status = DSPI_DRV_MasterSetDelay(kSensorSpiInstance,
    								 kDspiAfterTransfer,
    								 500, // delayInNanoSec
    								 & calculatedAfterTransfer);
	if (status != kStatus_DSPI_Success)
		return false;

	/* You must pull CS0 high.  We are not using CS0. */
	sensor_spi_set_csn(0, 1);

	return true;
}

/**
 *  @brief      Set compass sampling rate.
 *  The compass on the auxiliary I2C bus is read by the MPU hardware at a
 *  maximum of 100Hz. The actual rate can be set to a fraction of the gyro
 *  sampling rate.
 *
 *  \n WARNING: The new rate may be different than what was requested. Call
 *  mpu_get_compass_sample_rate to check the actual setting.
 *  @param[in]  rate    Desired compass sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu9250_set_compass_sample_rate(unsigned short rate)
{
    unsigned char div;
    if (((!rate) | rate) > MAX_COMPASS_SAMPLE_RATE)
        return -1;
    div = TS_GYRO_FREQ / rate - 1;
    write_mag_reg(MPU9250_REG_I2C_SLV4_CTRL, div);
    return 0;
}

void mpu9250_init() {

	/* Init SPI based on CS0 */
	mpu9250_init_all_pins();

	mpu9250_init_regs();
}

static gpio_output_pin_user_config_t cs0;
static gpio_output_pin_user_config_t cs1;
static gpio_output_pin_user_config_t cs2;
static gpio_output_pin_user_config_t cs3;
static gpio_output_pin_user_config_t cs4;
static gpio_output_pin_user_config_t cs5;

static gpio_output_pin_user_config_t mosi;
static gpio_output_pin_user_config_t miso;
static gpio_output_pin_user_config_t clk;
/*
 * Val: 1 - means set it as GPIO and write 1 to it
 *      0 - means set it as CS
 */
void sensor_spi_set_csn(uint16_t cs, uint16_t val) {
	gpio_output_pin_user_config_t pin;

	switch (cs) {
	case 0:
		pin = cs0;
		if (val) {
			PORT_HAL_SetMuxMode(PORTC,4u,kPortMuxAsGpio);
			GPIO_DRV_SetPinOutput(pin.pinName);
		} else {
			PORT_HAL_SetMuxMode(PORTC,4u,kPortMuxAlt2);
		}
		break;
	case 1:
		pin = cs1;
		if (val) {
			PORT_HAL_SetMuxMode(PORTD,4u,kPortMuxAsGpio);
			GPIO_DRV_SetPinOutput(pin.pinName);
		} else {
			PORT_HAL_SetMuxMode(PORTD,4u,kPortMuxAlt2);
		}
		break;
	case 2:
		pin = cs2;
		if (val) {
			PORT_HAL_SetMuxMode(PORTD,5u,kPortMuxAsGpio);
			GPIO_DRV_SetPinOutput(pin.pinName);
		} else {
			PORT_HAL_SetMuxMode(PORTD,5u,kPortMuxAlt2);
		}
		break;
	case 3:
		pin = cs3;
		if (val) {
			PORT_HAL_SetMuxMode(PORTD,6u,kPortMuxAsGpio);
			GPIO_DRV_SetPinOutput(pin.pinName);
		} else {
			PORT_HAL_SetMuxMode(PORTD,6u,kPortMuxAlt2);
		}
		break;
	case 4:
		pin = cs4;
		if (val) {
			PORT_HAL_SetMuxMode(PORTC,0u,kPortMuxAsGpio);
			GPIO_DRV_SetPinOutput(pin.pinName);
		} else {
			PORT_HAL_SetMuxMode(PORTC,0u,kPortMuxAlt2);
		}
		break;
	case 5:
		pin = cs5;
		if (val) {
			PORT_HAL_SetMuxMode(PORTB,23u,kPortMuxAsGpio);
			GPIO_DRV_SetPinOutput(pin.pinName);
		} else {
			PORT_HAL_SetMuxMode(PORTB,23u,kPortMuxAlt3);
		}
		break;
	default:
		break;
	}
}

/* Set everything to 1 */
void mpu9250_init_cs_gpio(void) {

	cs1.config.outputLogic = 1;
	cs1.config.slewRate = kPortSlowSlewRate;
//	cs1.config.driveStrength = kPortHighDriveStrength;
	cs1.pinName = GPIO_MAKE_PIN(GPIOD_IDX, 4);
	GPIO_DRV_OutputPinInit(&cs1);
}

void update_maxwell_cs(uint8_t uart_num) {

	/* Maxwell only take highest 3 bits */
  	maxwell_select = uart_num << 5;
}

bool mpu9250_detect_sensor(uint8_t uart_num, uint8_t* whoami) {
	int status = kStatus_DSPI_Timeout;

	*whoami = 0;

  	update_maxwell_cs(uart_num);

	uint8_t data[MPU9250_REG_COUNT];

	sensor_read_regs(MPU9250_REG_WHO_AM_I, 1, data);
	*whoami = data[0];

	return true;
}

bool sat_enable_sensor(uint8_t sensor_num, uint8_t uart_num) {
  	update_maxwell_cs(uart_num);

	set_mpu9250_sensor_num(sensor_num);

	return true;
}

bool mpu9250_init_all_pins(void)
{
	int resulting_compass_rate;

	for (uint8_t qq = 0; qq < MAX_SENSORS_IN_CS; qq++) {
		running[qq] = false;
	}
	self_test = false;
	data_avail = false;

#if DO_TEMP_COMP
	for (int axis = X; axis <= Z; axis++)
	{
		// Getting suitable temperature compensation
		// scale factors will require calibrating the
		// device at two temperatures and using
		// linear regression.  It would be desirable
		// to calibrate and the minimum and maximum
		// expected operating temperature. It would be
		// even better to calibrate at more than two
		// temperatures and do a least-squares curve
		// fit, requiring more coefficients than this
		// code currently uses.
		gyro_temp_comp[axis].offset = 0.0;
		gyro_temp_comp[axis].scale = 0.0;
		accel_temp_comp[axis].offset = 0.0;
		accel_temp_comp[axis].scale = 0.0;
	}
#endif

#if PIVOT_3_0
	// PTC5 - SPI0_SCLK
	PORT_HAL_SetSlewRateMode(PORTC,5u,kPortSlowSlewRate);
	PORT_HAL_SetMuxMode(PORTC,5u,kPortMuxAlt2);

	// PTC6 - SPI0_SOUT
	PORT_HAL_SetSlewRateMode(PORTC,6u,kPortSlowSlewRate);
	PORT_HAL_SetMuxMode(PORTC,6u,kPortMuxAlt2);

	// PTC7 - SPI0_SIN
	PORT_HAL_SetSlewRateMode(PORTC,7u,kPortSlowSlewRate);
	PORT_HAL_SetMuxMode(PORTC,7u,kPortMuxAlt2);
#endif /* PIVOT_3_0 */

	if (! mpu9250_configure_spi())
	{
		//printf ("error configuring SPI\r\n");
		return false;
	}

	return true;
}

bool mpu9250_init_regs(void) {
	sensor_write_reg_no_verify(MPU9250_REG_SIGNAL_PATH_RESET,  0x07);  // signal path reset = gyro_rst + accel_rst + tmp_rst
	sensor_write_reg_no_verify(MPU9250_REG_USER_CTRL,          0x07);  // fifo_rst + i2c_mst_rst + sig_cond_rst
								 	 	 	 	 	  // fifo_rst and i2c_mst_rst auto-clear after one clock cycle
													  // sig_cond_rst might also auto-clear; doc is unclear
	OSA_TimeDelay(10);
	sensor_write_reg_no_verify(MPU9250_REG_SIGNAL_PATH_RESET,  0x00);  // clear signal path reset

	sensor_write_reg(MPU9250_REG_I2C_SLV0_CTRL,      0x00);  // disable
	sensor_write_reg(MPU9250_REG_I2C_SLV1_CTRL,      0x00);  // disable
	sensor_write_reg(MPU9250_REG_I2C_SLV2_CTRL,      0x00);  // disable
	sensor_write_reg(MPU9250_REG_I2C_SLV3_CTRL,      0x00);  // disable
	sensor_write_reg(MPU9250_REG_I2C_SLV4_CTRL,      0x00);  // disable
	sensor_write_reg_no_verify(MPU9250_REG_USER_CTRL,          0x20);  // I2C_MST_EN

	sensor_write_reg(MPU9250_REG_SMPLRT_DIV,         SMPLRT_DIV);  // sample rate divisor = 10, so 100 Hz output
	sensor_write_reg(MPU9250_REG_CONFIG,             0x00 | DPLF_CFG);  // FIFO_MODE disabled, EXT_SYNC_SET disabled, DPLF_CFG
	sensor_write_reg(MPU9250_REG_GYRO_CONFIG,        (GYRO_FS_SEL<<3) | FCHOICE_B);  // gyro self-test disabled, all axes
	sensor_write_reg(MPU9250_REG_ACCEL_CONFIG,       ACCEL_FS_SEL<<3);  // accel self-test disabled, all axes
	sensor_write_reg(MPU9250_REG_ACCEL_CONFIG2,      (ACCEL_FCHOICE_B<<3) | A_DPLF_CFG);
	sensor_write_reg(MPU9250_REG_PWR_MGMT_1,         0x00);  // PWR_MGMT_1 = 0

#if MAG_SUBSAMPLE != 1
	sensor_write_reg(MPU9250_REG_I2C_SLV4_CTRL,      (MAG_SUBSAMPLE-1));  // I2C_MST_DLY: only read every nth sample
	sensor_write_reg(MPU9250_REG_I2C_MST_DELAY_CTRL, 0x81);  // DELAY_ES_SHADOW | I2C_SLV0_DLY_EN
#else
	sensor_write_reg(MPU9250_REG_I2C_SLV4_CTRL,      0x00);  // I2C_MST_DLY: read every sample
	sensor_write_reg(MPU9250_REG_I2C_MST_DELAY_CTRL, 0x80);  // DELAY_ES_SHADOW
#endif

	sensor_write_reg(MPU9250_REG_I2C_MST_CTRL,       0x10);  // not MULT_MST_EN, !WAIT_FOR_ES, !SLV_3_FIFO_EN, I2C_MST_P_NSR, I2C_MST_CLK = 348 kHz

#if READ_MAG_DIRECTLY
	///////////////////////cwati mag setup
	sensor_write_reg(MPU9250_REG_INT_PIN_CFG, 0x2);  // bypass mode cwati
#endif /* READ_MAG_DIRECTLY */

	sensor_write_reg(MPU9250_REG_INT_PIN_CFG, 0xc0); /* INT active low, open drain */
	sensor_write_reg(MPU9250_REG_INT_ENABLE, 0x00);  /* DIsable all INT */

	write_mag_reg(AK8963_REG_CNTL1,           0x00);  // power-down, necessary to exit fuse access mode  //cwati Power down MAG
	OSA_TimeDelay(10);	//cwati 10ms delay after powering down

	write_mag_reg(AK8963_REG_CNTL1,           0x0f);  // enter fuse access mode
	OSA_TimeDelay(10);	//cwati

	ak8963_sensitivity_cal[X] = read_mag_reg(AK8963_REG_ASAX);
	ak8963_sensitivity_cal[Y] = read_mag_reg(AK8963_REG_ASAY);
	ak8963_sensitivity_cal[Z] = read_mag_reg(AK8963_REG_ASAZ);
	write_mag_reg(AK8963_REG_CNTL1,           0x00);  // power-down, necessary to exit fuse access mode  //cwati Power down MAG

	OSA_TimeDelay(10);	//cwati 10ms delay after powering down

	/* WARNING: sample rate bigger than 100 Hz is NOT supported!!
	 * If you set TS_MAG_RATE to something bigger than 100 Hz, it should generate compile error.
	 * In the case it doesn't generate compile error, it will set to 100 Hz here.
	 */
	if (mag_rate >= 100) {
		write_mag_reg(AK8963_REG_CNTL1,           0x16);  // BIT (16-bit), continuous mode 2 (100 Hz)
	} else if (mag_rate == 8) {
		write_mag_reg(AK8963_REG_CNTL1,           0x12);  // BIT (16-bit), continuous mode 1 (8 Hz)
	} else {
		/* Result is not guaranteed! */
		mpu9250_set_compass_sample_rate(mag_rate);
	}
	OSA_TimeDelay(10);	//cwati 10ms delay

	sensor_write_reg_no_verify(MPU9250_REG_USER_CTRL,          0x00);  // !I2C_MST_EN

	return true;
}


bool mpu9250_start(void)
{
	if (running[mpu9250_num])
		return false;

	sensor_write_reg_no_verify(MPU9250_REG_USER_CTRL,          0x00);  // !I2C_MST_EN
	sensor_write_reg_no_verify(MPU9250_REG_USER_CTRL,          0x02);  // !I2C_MST_EN | I2C_MST_RST
	sensor_write_reg_no_verify(MPU9250_REG_USER_CTRL,          0x00);  // !I2C_MST_EN
	sensor_write_reg_no_verify(MPU9250_REG_USER_CTRL,          0x20);  // I2C_MST_EN

	// enable reading magnetometer
#if !READ_MAG_DIRECTLY
	uint8_t smplrt_div;
	smplrt_div = (int) (1000/200 /* Freq */ - 1);

	sensor_write_reg(MPU9250_REG_SMPLRT_DIV,         smplrt_div);  // sample rate divisor = 10, so 100 Hz output
//
//	uint8_t tmp = 	sensor_read_reg(MPU9250_REG_SMPLRT_DIV);  // sample rate divisor = 10, so 100 Hz output

	sensor_write_reg(MPU9250_REG_I2C_SLV0_ADDR,      0x80 + AK8963_I2C_ADDR);  // read from 0x0c
	sensor_write_reg(MPU9250_REG_I2C_SLV0_REG,       AK8963_FIRST_REG);
	sensor_write_reg(MPU9250_REG_I2C_SLV0_CTRL,      0x80 + AK8963_REG_COUNT);  // enable auto read, length
#endif /* READ_MAG_DIRECTLY */
	running[mpu9250_num] = true;
	return true;
}


static bool mpu9250_self_test_start(void)
{
	if (running[mpu9250_num])
		return false;

	sensor_write_reg_no_verify(MPU9250_REG_USER_CTRL,          0x00);  // !I2C_MST_EN
	sensor_write_reg(MPU9250_REG_I2C_SLV0_CTRL,      0x00);  // disable reading magnetometer

	sensor_write_reg(MPU9250_REG_SMPLRT_DIV,         SELFTEST_SMPLRT_DIV);  // sample rate divisor = 1, so 1000 Hz output

	running[mpu9250_num] = true;
	return true;
}


bool mpu9250_stop(void)
{
	if (! running[mpu9250_num])
		return true;
	running[mpu9250_num] = false;
//	sensor_write_reg(MPU9250_REG_INT_ENABLE,         0x00);  // disable data ready interrupt
	sensor_write_reg(MPU9250_REG_SMPLRT_DIV,         SMPLRT_DIV);  // sample rate divisor = 10, so 100 Hz output
	sensor_write_reg(MPU9250_REG_I2C_SLV0_CTRL,      0x00);  // disable reading magnetometer
	return true;
}

//
//bool mpu9250_data_avail(void)
//{
//	uint8_t data;
//
//	sensor_read_regs(MPU9250_REG_INT_STATUS, 1, &data);
//
//	if (! data & 0x01) {
//		return false;
//	}
//
//	return true;
//
//	//cwati test... instead of interrupt, use timer...
////	return data_avail;
//
////	static uint32_t last_data_avail;
////	static bool first_time = true;
////	uint32_t time_now;
////
////	if (first_time) {
////		last_data_avail = k22f_get_rtc();
////		first_time = false;
////	}
////
////	time_now = k22f_get_rtc();
////	if (time_now - last_data_avail > DATA_AVAIL_PERIOD /* TODO change based on freq */) {
////		last_data_avail = time_now;
////		return true;
////	}
////
////	return false;
////
//
//}
//
//void mpu9250_clear_data_avail(void)
//{
//		data_avail = false;
//}


// AK8963 compass data sensitivity calibration
static int16_t ak8963_sensitivity_compensate(int16_t raw, uint8_t sens_cal)
{
	return (((int32_t) raw) * ((sens_cal-128)+256))/256;
}


#if DO_TEMP_COMP
// general temperature compensation of a 16-bit data
// value with a floating point temperature in degrees C, and
// linear compensation offset and scale values
static int16_t temp_compensate(int16_t raw, float temp, temp_comp_t *comp)
{
	float v = raw + comp->offset;
	v *= (1.0 + comp->scale * temp);
	return v;
}
#endif

bool mpu9250_read(int sensors,  // bit field, SENSOR_x above
				  sensor_data_t *sensor_data)
{
	uint8_t data[MPU9250_REG_COUNT];
#if READ_MAG_DIRECTLY
	uint8_t mag_data[10];
#endif

	sensor_data->valid_sensors = 0;
	sensor_data->timestamp = k22f_get_rtc();

	if (self_test) {
		sensor_read_regs(MPU9250_FIRST_REG, MPU9250_SELF_TEST_REG_COUNT, data);
	} else {
		sensor_read_regs(MPU9250_FIRST_REG, MPU9250_REG_COUNT, data);
	}

//	if (! data[MPU9250_REG_OFFSET(MPU9250_REG_INT_STATUS)] & 0x01)
//		return false;

	if (sensors & SENSOR_ACCEL)
	{
		int i = MPU9250_REG_OFFSET(MPU9250_REG_ACCEL_XOUT_H);
		for (int axis = X; axis <= Z; axis++)
		{
			sensor_data->accel[axis] = (data[i]<<8) | data[i+1];
			i += 2;
		}
		sensor_data->valid_sensors |= SENSOR_ACCEL;
	}

	if (sensors & SENSOR_GYRO)
	{
		int i = MPU9250_REG_OFFSET(MPU9250_REG_GYRO_XOUT_H);
		for (int axis = X; axis <= Z; axis++)
		{
			sensor_data->gyro[axis] = (data[i]<<8) | data[i+1];
			i += 2;
		}
		sensor_data->valid_sensors |= SENSOR_GYRO;

	}

	// MAG X, Y, Z - NOTE: mag axes in AK8963 not oriented same as accel, gyro (MPU-9250 data sheet section 9.1)
	// mag X and Y are interchanged relative to accel, gyro
	// mag Z is inverted relative to accel, gyro

#if READ_MAG_DIRECTLY
	if (sensors & SENSOR_MAG)
	{
		//read manually from MAG
		uint8_t tmp, cnt = 0;
		tmp = read_mag_reg_fast(AK8963_REG_WAI);
		while (tmp != 0x48) {
			cnt++;
			debug_printf("\n\rSensor %u, cs %u, WAI = 0x%x cnt = %u", mpu9250_num, valid_cs[mpu9250_num], tmp, cnt);
			OSA_TimeDelay(5);
			tmp = read_mag_reg_fast(AK8963_REG_WAI);
			if (cnt < 10) {
				break;
			}
		}

		// Read 10 bytes from mag
		for (uint8_t qq = 0; qq < 10; qq++) {
			mag_data[qq] = read_mag_reg_fast(AK8963_FIRST_REG + qq);
		}

		if (mag_data[AK8963_REG_STATUS_1] & 0x01) {
			int16_t raw[3];
			int i = 0;
			for (int mag_axis = X; mag_axis <= Z; mag_axis++)
			{
				raw[mag_axis] = mag_data[i] | (mag_data[i+1]<<8);
				i += 2;
			}
			sensor_data->mag[Y] = ak8963_sensitivity_compensate(raw[X], ak8963_sensitivity_cal[X]);
			sensor_data->mag[X] = ak8963_sensitivity_compensate(raw[Y], ak8963_sensitivity_cal[Y]);
			sensor_data->mag[Z] = - ak8963_sensitivity_compensate(raw[Z], ak8963_sensitivity_cal[Z]);
			sensor_data->valid_sensors |= SENSOR_MAG;
		} else {
			sensor_data->mag[Y] = 0;
			sensor_data->mag[X] = 0;
			sensor_data->mag[Z] = 0;
		}
	}
#else
	if ((sensors & SENSOR_MAG) && (data[AK8963_REG_OFFSET(AK8963_REG_STATUS_1)] & 0x01))
	{
		/* If the Who-Am-I register is wrong, then we consider the data invalid */
		if (data[AK8963_REG_OFFSET(AK8963_REG_WAI)] == 0x48 ) {
			int16_t raw[3];
			int i = AK8963_REG_OFFSET(AK8963_REG_HXL);
			for (int mag_axis = X; mag_axis <= Z; mag_axis++)
			{
				raw[mag_axis] = data[i] | (data[i+1]<<8);
				i += 2;
			}
			sensor_data->mag[Y] = ak8963_sensitivity_compensate(raw[X], ak8963_sensitivity_cal[X]);
			sensor_data->mag[X] = ak8963_sensitivity_compensate(raw[Y], ak8963_sensitivity_cal[Y]);
			sensor_data->mag[Z] = - ak8963_sensitivity_compensate(raw[Z], ak8963_sensitivity_cal[Z]);
			sensor_data->valid_sensors |= SENSOR_MAG;
		}
	}
#endif /* READ_MAG_DIRECTLY */

#define MPU9250_TEMP_OFFSET 0.0  /* deg C */
#define MPU9250_TEMP_SENS 334.0  /* LSB per deg C */
	if (sensors & SENSOR_TEMP)
	{
		int i = MPU9250_REG_OFFSET(MPU9250_REG_TEMP_OUT_H);
		uint16_t raw = (data[i]<<8) | data[i+1];
		sensor_data->temp =  (raw - MPU9250_TEMP_OFFSET) / MPU9250_TEMP_SENS;
		sensor_data->valid_sensors |= SENSOR_TEMP;

#if DO_TEMP_COMP
		if (sensor_data->valid_sensors & SENSOR_ACCEL)
			for (int axis = X; i <= Z; axis++)
				sensor_data->accel[axis] = temp_compensate(sensor_data->accel[axis], sensor_data->temp, & accel_temp_comp[axis]);

		if (sensor_data->valid_sensors & SENSOR_GYRO)
			for (int axis = X; i <= Z; axis++)
				sensor_data->gyro[axis] = temp_compensate(sensor_data->gyro[axis], sensor_data->temp, & gyro_temp_comp[axis]);
#endif
	}

	return true;
}


bool mpu9250_get_factory_self_test_results(mpu9250_self_test_results_t *factory_st)
{
	if (running[mpu9250_num])
		return false;
	sensor_read_regs(MPU9250_REG_SELF_TEST_X_GYRO, 3, factory_st->gyro);
	sensor_read_regs(MPU9250_REG_SELF_TEST_X_ACCEL, 3, factory_st->accel);
	return true;
}

static bool average_test_samples(int flags, int count, mpu9250_average_data_t *results)
{
	bool status = true;
	bool accel_out_of_range = false;
	bool gyro_out_of_range = false;
	sensor_data_t sensor;
	int32_t gyro_accum[3];
	int32_t accel_accum[3];

	memset(results, 0, sizeof(mpu9250_average_data_t));
	memset(accel_accum, 0, sizeof(accel_accum));
	memset(gyro_accum, 0, sizeof(gyro_accum));

	mpu9250_self_test_start();
	for (int i = 0; i < count; i++)
	{
//		while (! mpu9250_data_avail())
//			;
//		mpu9250_clear_data_avail();
		mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO, & sensor);
		if (flags & ST_FLAG_ACCEL)
			for (int axis = X; axis <= Z; axis++)
			{
				if (abs(sensor.accel[axis]) >= 32000)
			    {
					accel_out_of_range = true;
					status = false;
			    }
			accel_accum[axis] += sensor.accel[axis];
			}
		if (flags & ST_FLAG_GYRO)
			for (int axis = X; axis <= Z; axis++)
			{
				if (abs(sensor.gyro[axis]) >= 32000)
			    	{
					gyro_out_of_range = true;
					status = false;
			    	}
			gyro_accum[axis] += sensor.gyro[axis];
			}

	}
	mpu9250_stop();

	if (accel_out_of_range)
	{
		if (flags & ST_FLAG_DEBUG_PRINTF)
			debug_printf("accel data out of range\r\n");
	}

	if (gyro_out_of_range)
	{
		if (flags & ST_FLAG_DEBUG_PRINTF)
			debug_printf("gyro data out of range\r\n");
	}

	for (int axis = X; axis <= Z; axis++)
	{
		if (flags & ST_FLAG_ACCEL)
		{
			if (flags & ST_FLAG_DEBUG_PRINTF)
				debug_printf("accel %c accum: %d\r\n", 'X'+axis, accel_accum[axis]);
			accel_accum[axis] /= count;
			if ((accel_accum[axis] < -32767) || (accel_accum[axis] > 32767))
				status = false;
			results->accel[axis] = accel_accum[axis];
		}
		if (flags & ST_FLAG_GYRO)
		{
			if (flags & ST_FLAG_DEBUG_PRINTF)
				debug_printf("gyro %c accum: %d\r\n", 'X'+axis, gyro_accum[axis]);
			gyro_accum[axis] /= count;
			if ((gyro_accum[axis] < -32767) || (gyro_accum[axis] > 32767))
				status = false;
			results->gyro[axis] = gyro_accum[axis];
		}
	}
	return status;
}

#if SELFTEST_GYRO_BANDWIDTH == 5
#  define SELFTEST_FCHOICE_B 0
#  define SELFTEST_DPLF_CFG 6
#elif SELFTEST_GYRO_BANDWIDTH == 10
#  define SELFTEST_FCHOICE_B 0
#  define SELFTEST_DPLF_CFG 5
#elif SELFTEST_GYRO_BANDWIDTH == 20
#  define SELFTEST_FCHOICE_B 0
#  define SELFTEST_DPLF_CFG 4
#elif SELFTEST_GYRO_BANDWIDTH == 41
#  define SELFTEST_FCHOICE_B 0
#  define SELFTEST_DPLF_CFG 3
#elif SELFTEST_GYRO_BANDWIDTH == 92
#  define SELFTEST_FCHOICE_B 0
#  define SELFTEST_DPLF_CFG 2
#elif GYRO_BANDWIDTH == 184
#  define SELFTEST_FCHOICE_B 0
#  define SELFTEST_DPLF_CFG 1
#else
#  error "invalid SELFTEST_GYRO_BANDWIDTH"
#endif

#if SELFTEST_GYRO_RANGE_DPS == 250
#  define SELFTEST_GYRO_FS_SEL 0
#elif SELFTEST_GYRO_RANGE_DPS == 500
#  define SELFTEST_GYRO_FS_SEL 1
#elif SELFTEST_GYRO_RANGE_DPS == 1000
#  define SELFTEST_GYRO_FS_SEL 2
#elif SELFTEST_GYRO_RANGE_DPS == 2000
#  define SELFTEST_GYRO_FS_SEL 3
#else
#  error "invalid SELFTEST_GYRO_RANGE_DPS"
#endif

#if SELFTEST_GYRO_RANGE_DPS == 250
#  define SELFTEST_MPU9250_COUNTSPERDEGPERSEC (131.0F)
#elif SELFTEST_GYRO_RANGE_DPS == 500
#  define SELFTEST_MPU9250_COUNTSPERDEGPERSEC (65.5F)
#elif SELFTEST_GYRO_RANGE_DPS == 1000
#  define SELFTEST_MPU9250_COUNTSPERDEGPERSEC (32.8F)
#elif SELFTEST_GYRO_RANGE_DPS == 2000
#  define SELFTEST_MPU9250_COUNTSPERDEGPERSEC (16.4F)
#else
#  error "invalid SELFTEST_GYRO_RANGE_DPS"
#endif

#if SELFTEST_ACCEL_BANDWIDTH == 5
#  define SELFTEST_ACCEL_FCHOICE_B 0
#  define SELFTEST_A_DPLF_CFG 6
#elif SELFTEST_ACCEL_BANDWIDTH == 10
#  define SELFTEST_ACCEL_FCHOICE_B 0
#  define SELFTEST_A_DPLF_CFG 5
#elif SELFTEST_ACCEL_BANDWIDTH == 20
#  define SELFTEST_ACCEL_FCHOICE_B 0
#  define SELFTEST_A_DPLF_CFG 4
#elif SELFTEST_ACCEL_BANDWIDTH == 41
#  define SELFTEST_ACCEL_FCHOICE_B 0
#  define SELFTEST_A_DPLF_CFG 3
#elif SELFTEST_ACCEL_BANDWIDTH == 92
#  define SELFTEST_ACCEL_FCHOICE_B 0
#  define SELFTEST_A_DPLF_CFG 2
#elif SELFTEST_ACCEL_BANDWIDTH == 184
#  define SELFTEST_ACCEL_FCHOICE_B 0
#  define SELFTEST_A_DPLF_CFG 1
#else
#  error "invalid SELFTEST_ACCEL_BANDWIDTH"
#endif

#if SELFTEST_ACCEL_RANGE_G == 2
#  define SELFTEST_ACCEL_FS_SEL 0
#elif SELFTEST_ACCEL_RANGE_G == 4
#  define SELFTEST_ACCEL_FS_SEL 1
#elif SELFTEST_ACCEL_RANGE_G == 8
#  define SELFTEST_ACCEL_FS_SEL 2
#elif SELFTEST_ACCEL_RANGE_G == 16
#  define SELFTEST_ACCEL_FS_SEL 3
#else
#  error "invalid SELFTEST_ACCEL_RANGE_G"
#endif

#if SELFTEST_ACCEL_RANGE_G == 2
#  define SELFTEST_MPU9250_COUNTSPERG (16384.0F)
#elif SELFTEST_ACCEL_RANGE_G == 4
#  define SELFTEST_MPU9250_COUNTSPERG (8192.0F)
#elif SELFTEST_ACCEL_RANGE_G == 8
#  define SELFTEST_MPU9250_COUNTSPERG (4096.0F)
#elif SELFTEST_ACCEL_RANGE_G == 16
#  define SELFTEST_MPU9250_COUNTSPERG (2048.0F)
#else
#  error "invalid SELFTEST_ACCEL_RANGE_G"
#endif

bool mpu9250_self_test(int flags,
					   mpu9250_average_data_t *normal,
		               mpu9250_average_data_t *selftest)
{
	bool status = true;
	if (running[mpu9250_num])
		return false;
	self_test = true;

#if 0
	printf("SELFTEST_DPLF_CFG: %d\r\n", SELFTEST_DPLF_CFG);
	printf("SELFTEST_GYRO_FS_SEL: %d\r\n", SELFTEST_GYRO_FS_SEL);
	printf("SELFTEST_FCHOICE_B: %d\r\n", SELFTEST_FCHOICE_B);

	printf("SELFTEST_A_DPLF_CFG: %d\r\n", SELFTEST_A_DPLF_CFG);
	printf("SELFTEST_ACCEL_FS_SEL: %d\r\n", SELFTEST_ACCEL_FS_SEL);
	printf("SELFTEST_ACCEL_FCHOICE_B: %d\r\n", SELFTEST_ACCEL_FCHOICE_B);
#endif

	// change sample rate and sensitivity to settings required for self-test
	sensor_write_reg(MPU9250_REG_SMPLRT_DIV,         SELFTEST_SMPLRT_DIV);
	sensor_write_reg(MPU9250_REG_CONFIG,             0x00 | SELFTEST_DPLF_CFG);  // FIFO_MODE disabled, EXT_SYNC_SET disabled, DPLF_CFG
	sensor_write_reg(MPU9250_REG_GYRO_CONFIG,        (SELFTEST_GYRO_FS_SEL<<3) | SELFTEST_FCHOICE_B);  // gyro self-test disabled, all axes
	sensor_write_reg(MPU9250_REG_ACCEL_CONFIG,       (SELFTEST_ACCEL_FS_SEL<<3));  // accel self-test disabled, all axes
	sensor_write_reg(MPU9250_REG_ACCEL_CONFIG2,      (SELFTEST_ACCEL_FCHOICE_B<<3) | SELFTEST_A_DPLF_CFG);

	status &= average_test_samples(flags, 200, normal);

	// XXX set self-test bits
	sensor_write_reg(MPU9250_REG_GYRO_CONFIG,        0xe0 | (SELFTEST_GYRO_FS_SEL<<3) | SELFTEST_FCHOICE_B);  // gyro self-test enabled, all axes
	sensor_write_reg(MPU9250_REG_ACCEL_CONFIG,       0xe0 | (SELFTEST_ACCEL_FS_SEL<<3));  // accel self-test enabled, all axes

	OSA_TimeDelay(20); // wait 20ms for oscillations from enabling self-test to stabilize

	status &= average_test_samples(flags, 200, selftest);

	// change rate and sensitivity back to normal settings, and clear self-test bits
	sensor_write_reg(MPU9250_REG_SMPLRT_DIV,         SELFTEST_SMPLRT_DIV);
	sensor_write_reg(MPU9250_REG_CONFIG,             0x00 | DPLF_CFG);  // FIFO_MODE disabled, EXT_SYNC_SET disabled, DPLF_CFG
	sensor_write_reg(MPU9250_REG_GYRO_CONFIG,        (GYRO_FS_SEL<<3) | FCHOICE_B);  // gyro self-test disabled, all axes
	sensor_write_reg(MPU9250_REG_ACCEL_CONFIG,       (ACCEL_FS_SEL<<3));  // accel self-test disabled, all axes
	sensor_write_reg(MPU9250_REG_ACCEL_CONFIG2,      (ACCEL_FCHOICE_B<<3) | A_DPLF_CFG);

	OSA_TimeDelay(20); // wait 20ms for oscillations from disabling self-test to stabilize

	self_test = false;
	return status;
}

#define ACCEL_ST_ABS_LIMIT   (0.675 * SELFTEST_MPU9250_COUNTSPERG)       /* 675 mgee */
#define GYRO_ST_ABS_LIMIT    (60 * SELFTEST_MPU9250_COUNTSPERDEGPERSEC)  /* 60 dps */
#define GYRO_MAX_OFFSET      (20 * SELFTEST_MPU9250_COUNTSPERDEGPERSEC)  /* 20 dps */

static float factory_st_decode(int fs, uint8_t code)
{
	return (2620.0 / (1 << fs)) * powf(1.01, code - 1);
}

uint32_t mpu9250_check_selftest_results(int flags,
								    mpu9250_self_test_results_t *factory_st,
		                          	mpu9250_average_data_t *normal,
		                          	mpu9250_average_data_t *selftest)
{
	uint32_t status = 0;  // assume OK

	if (flags & ST_FLAG_ACCEL)
		for (int axis=X; axis <= Z; axis++)
		{
			int accel_response = selftest->accel[axis] - normal->accel[axis];

			if (factory_st->accel[axis] == 0)
			{
				// no factory calibration, check against absolute limits
				if (abs(accel_response) > ACCEL_ST_ABS_LIMIT)
				{
					if (flags & ST_FLAG_DEBUG_PRINTF)
						printf("accel %c above absolute limit, %d > %d\r\n", 'X'+axis, accel_response, ACCEL_ST_ABS_LIMIT);
					status++;
				}
			}
			else
			{
				float factory = factory_st_decode(SELFTEST_ACCEL_FS_SEL, factory_st->accel[axis]);
				float ratio = accel_response / factory;
	            if ((ratio < 0.5) || (ratio > 1.5))
	            {
	    			if (flags & ST_FLAG_DEBUG_PRINTF)
	    				printf("accel %c response too far from factory\r\n", 'X'+axis, accel_response, (int)factory);
	            	status++;
	            }
			}
		}

	if (flags & ST_FLAG_GYRO)
		for (int axis=X; axis <= Z; axis++)
		{
			int gyro_response = selftest->gyro[axis] - normal->gyro[axis];

			if (factory_st->gyro[axis] == 0)
			{
				// no factory calibration, check against absolute limits
				if (abs(gyro_response) < GYRO_ST_ABS_LIMIT)
				{
					if (flags & ST_FLAG_DEBUG_PRINTF)
						printf("gyro %c below absolute limit, %d < %d\r\n", 'X'+axis, gyro_response, GYRO_ST_ABS_LIMIT);
					status++;
				}
			}
			else
			{
				float factory = factory_st_decode(SELFTEST_GYRO_FS_SEL, factory_st->gyro[axis]);
				float response = gyro_response / factory;
	            if (response < 0.5)
	            {
	    			if (flags & ST_FLAG_DEBUG_PRINTF)
	    				printf("gyro %c response less than half of factory, %d, %d\r\n", 'X'+axis, gyro_response, (int)factory);
	            	status++;
	            }
			}

			// also have to check gyro offset
			if (abs(normal->gyro[axis]) > GYRO_MAX_OFFSET)
			{
				if (flags & ST_FLAG_DEBUG_PRINTF)
					printf("gyro %c offset too large, %d > %d\r\n", 'X'+axis, normal->gyro[axis], GYRO_MAX_OFFSET);
				status++;
			}
		}

	return status;
}

void mpu9250_sleep(void)
{
	sensor_write_reg(MPU9250_REG_PWR_MGMT_1, 0x4F); // SLEEP=1, PD_PTAT=1 (power down volt gen), stop clock
//	sensor_write_reg(MPU9250_REG_PWR_MGMT_2, 0x3F); // disable accel and gyro
}

void mpu9250_wakeup(void)
{
	sensor_write_reg(MPU9250_REG_PWR_MGMT_1, 0x00); // enable voltage generator and clock
//	sensor_write_reg(MPU9250_REG_PWR_MGMT_2, 0x00); // enable accel and gyro
}
