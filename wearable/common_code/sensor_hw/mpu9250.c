/*
 * mpu9250.c
 *
 *  This file contains the SPI interface to the sensor.
 *  Initially the sensor was only MPU9250, but now the satellite might interface
 *  also to STM LSM6DSL.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fsl_dspi_master_driver.h"
#include "fsl_dspi_shared_function.h"
#include "fsl_gpio_common.h"  /* not needed with KSDK 1.1.0 ? */
#include "fsl_interrupt_manager.h"
#include "fsl_os_abstraction.h"
#include "fsl_uart_driver.h"

#define DSPI_DRV_MasterTransferBlocking DSPI_DRV_MasterTransferDataBlocking  /* fn name changed in KSDK 1.1.0 */

#include "board.h"
#include "sat_module.h"

/* MPU9250 header files are separated into 2 files,
 * one for firmware-related and the other one for
 * purely OSSF functionality.
 */
#include "lsm6dsl_firmware.h"
#include "mpu9250_firmware.h"
#include "mpu9250_ossf.h"
#include "ts_fusion.h"
#include "tasks.h"

unsigned short	mag_rate = TS_MAG_RATE;	/* Compass rate */

uint8_t maxwell_select = 0;
uint8_t invalid_maxwell_num = 0;

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

enum _more_pins
{
#if PROTO_BOARD
	kSensorSsPin   = GPIO_MAKE_PIN(HW_GPIOD,  4U),
	kSensorMosiPin = GPIO_MAKE_PIN(HW_GPIOD,  6U),
	kSensorIntPin  = GPIO_MAKE_PIN(HW_GPIOD,  3U), /* Proto Sensor module uses PTD3 for interrupt */
#else
	/* PRODUCTION1 or Eval Board */
	kSensorSsPin   = GPIO_MAKE_PIN(HW_GPIOC,  4U),
	kSensorMosiPin = GPIO_MAKE_PIN(HW_GPIOC,  6U),
	kSensorIntPin  = GPIO_MAKE_PIN(HW_GPIOC,  8U), /* Production Sensor module uses PTC8 for interrupt */

#if PIVOT_3_0
	kSensorCs1Pin  = GPIO_MAKE_PIN(HW_GPIOD,  4U),
	kSensorCs2Pin  = GPIO_MAKE_PIN(HW_GPIOD,  5U),
	kSensorCs3Pin  = GPIO_MAKE_PIN(HW_GPIOD,  6U),
#endif /* PIVOT_3_0 */
#endif
};

static const gpio_input_pin_user_config_t sensor_int_pin =
{
		.pinName = kSensorIntPin,
		.config =
		{
				.isPullEnable = false,
				.interrupt = kPortIntFallingEdge,
		}
};

#if !PIVOT_3_0
static const gpio_output_pin_user_config_t mpu9250_ss_pin =
{
		.pinName = kSensorSsPin,
		.config =
		{
				.outputLogic = 0,
				.slewRate = kPortFastSlewRate,
				.driveStrength = kPortHighDriveStrength,
				.isOpenDrainEnabled = 0
		}
};

static const gpio_output_pin_user_config_t mpu9250_mosi_pin =
{
		.pinName = kSensorMosiPin,
		.config =
		{
				.outputLogic = 0,
				.slewRate = kPortFastSlewRate,
				.driveStrength = kPortHighDriveStrength,
				.isOpenDrainEnabled = 0
		}
};
#endif

static const dspi_device_t sensor_dspi_device =
{
	.bitsPerSec = 1000000L,
	.dataBusConfig.bitsPerFrame = 8,
	.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh,
	.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge,
	.dataBusConfig.direction = kDspiMsbFirst,
};

dspi_master_state_t sensor_dspi_master_state;

#if PIVOT_3_0
const dspi_master_user_config_t sensor_dspi_master_user_config =
{
	.whichCtar = kDspiCtar0,
#if (PROTO_BOARD || PRODUCTION1)
	.whichPcs = kDspiPcs0,
#else
	/* Eval board */
	.whichPcs = kDspiPcs1,
#endif
	.pcsPolarity = kDspiPcs_ActiveLow,
	.isSckContinuous = false,
	.isChipSelectContinuous = true,
};
const dspi_master_user_config_t sensor_dspi_master_user_config_cs1 =
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
	GPIO_DRV_ClearPinIntFlag(sensor_int_pin.pinName);
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
	GPIO_DRV_ClearPinIntFlag(sensor_int_pin.pinName);
	data_avail = true;
}

void SPI0_IRQHandler(void)	/* sensor module uses SPI1 for invensense */
{
    DSPI_DRV_IRQHandler(HW_SPI0);
}
#endif

void mpu9250_set_irq_enable(bool v)
{
	if (v)
		INT_SYS_EnableIRQ(g_portIrqId[GPIO_EXTRACT_PORT(sensor_int_pin.pinName)]);
	else
		INT_SYS_DisableIRQ(g_portIrqId[GPIO_EXTRACT_PORT(sensor_int_pin.pinName)]);
}


void sensor_write_reg(uint8_t reg_addr, uint8_t data)
{
	dspi_status_t Error;
	mts_buffer[0] = reg_addr;
	mts_buffer[1] = data;

	/* cw TODO.  It seems like we need to toggle CS to make it work.
	 * Toggling is by selecting unused CS so that CS goes high.
	 */
	UART_DRV_SendDataBlocking(REV_F_UART_INSTANCE, &invalid_maxwell_num, 1, 1000u); /* Toggle CS up */

	UART_DRV_SendDataBlocking(REV_F_UART_INSTANCE, &maxwell_select, 1, 1000u); /* Toggle CS down */

	OSA_TimeDelay(1); //actually needs only 15us cwati TODO test

	Error = DSPI_DRV_MasterTransferBlocking(kSensorSpiInstance,
											& sensor_dspi_device,
											mts_buffer,
											stm_buffer,
											2,
											1000);
}


/* From the oscilloscope we estimate that we get about 1 us delay with this call */
void ts_22f_delay_us(uint16_t num) {
//	// TPR = 1 equals to ~30.5us = 1000000us / 32768Hz = (30.5/1000)ms = (1000/32768)ms
//	uint32_t tpr_start = HW_RTC_TPR_RD(RTC_BASE);
//	uint32_t tpr_now = HW_RTC_TPR_RD(RTC_BASE);

	for (uint16_t qq = 0; qq < num; qq++) {
		for (uint32_t cnt = 0; cnt < 11; cnt++) {
			;//do nothing
		}
	}
}

void sensor_read_regs(uint8_t reg_addr, size_t count, uint8_t *data)
{
	dspi_status_t Error;
	mts_buffer[0] = 0x80 | reg_addr;
	memset(& mts_buffer[1], 0, count);  // not really necessary
	memset(stm_buffer, 0, count+1); // also not really necessary

	/* cw TODO.  It seems like we need to toggle CS to make it work.
	 * Toggling is by selecting unused CS so that CS goes high.
	 */
	UART_DRV_SendDataBlocking(REV_F_UART_INSTANCE, &invalid_maxwell_num, 1, 1000u); /* Toggle CS up */

	UART_DRV_SendDataBlocking(REV_F_UART_INSTANCE, &maxwell_select, 1, 1000u); /* Toggle CS down */

	ts_22f_delay_us(15);

	Error = DSPI_DRV_MasterTransferBlocking(kSensorSpiInstance,
											& sensor_dspi_device,
											mts_buffer,
											stm_buffer,
											count + 1,
											1000);
	memcpy(data, & stm_buffer[1], count);
}

static void write_mpu9250_mag_reg(uint8_t reg_addr, uint8_t data)
{
	sensor_write_reg(MPU9250_REG_I2C_SLV4_DO,   data);
	sensor_write_reg(MPU9250_REG_I2C_SLV4_ADDR, 0x0c);      // write, magnetometer address
	sensor_write_reg(MPU9250_REG_I2C_SLV4_REG,  reg_addr);
	sensor_write_reg(MPU9250_REG_I2C_SLV4_CTRL, 0x80+(MAG_SUBSAMPLE-1));      // enable

	OSA_TimeDelay(30);			// delay for write to complete, should happen every 10 ms

	sensor_write_reg(MPU9250_REG_I2C_SLV4_CTRL, (MAG_SUBSAMPLE-1));      // disable

	OSA_TimeDelay(30);          // delay for write to stop happening
}

static uint8_t read_mpu9250_mag_reg(uint8_t reg_addr)
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

#define MPU9250_SPI_FREQUENCY 1000000L

static bool mpu9250_configure_spi(void)
{
	dspi_status_t status;
	uint32_t calculatedBaudRate;
	uint32_t calculatedPcsToSck, calculatedLastSckToPcs, calculatedAfterTransfer;

	/* We don't use CS0 now */
	status = DSPI_DRV_MasterInit(kSensorSpiInstance, & sensor_dspi_master_state, & sensor_dspi_master_user_config_cs1);
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
    if (!rate | rate > MAX_COMPASS_SAMPLE_RATE)
        return -1;
    div = TS_GYRO_FREQ / rate - 1;
    write_mpu9250_mag_reg(MPU9250_REG_I2C_SLV4_CTRL, div);
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
			PORT_HAL_SetMuxMode(g_portBaseAddr[2],4u,kPortMuxAsGpio);
			GPIO_DRV_SetPinOutput(pin.pinName);
		} else {
			PORT_HAL_SetMuxMode(g_portBaseAddr[2],4u,kPortMuxAlt2);
		}
		break;
	case 1:
		pin = cs1;
		if (val) {
			PORT_HAL_SetMuxMode(g_portBaseAddr[3],4u,kPortMuxAsGpio);
			GPIO_DRV_SetPinOutput(pin.pinName);
		} else {
			PORT_HAL_SetMuxMode(g_portBaseAddr[3],4u,kPortMuxAlt2);
		}
		break;
	case 2:
		pin = cs2;
		if (val) {
			PORT_HAL_SetMuxMode(g_portBaseAddr[3],5u,kPortMuxAsGpio);
			GPIO_DRV_SetPinOutput(pin.pinName);
		} else {
			PORT_HAL_SetMuxMode(g_portBaseAddr[3],5u,kPortMuxAlt2);
		}
		break;
	case 3:
		pin = cs3;
		if (val) {
			PORT_HAL_SetMuxMode(g_portBaseAddr[3],6u,kPortMuxAsGpio);
			GPIO_DRV_SetPinOutput(pin.pinName);
		} else {
			PORT_HAL_SetMuxMode(g_portBaseAddr[3],6u,kPortMuxAlt2);
		}
		break;
	case 4:
		pin = cs4;
		if (val) {
			PORT_HAL_SetMuxMode(g_portBaseAddr[2],0u,kPortMuxAsGpio);
			GPIO_DRV_SetPinOutput(pin.pinName);
		} else {
			PORT_HAL_SetMuxMode(g_portBaseAddr[2],0u,kPortMuxAlt2);
		}
		break;
	case 5:
		pin = cs5;
		if (val) {
			PORT_HAL_SetMuxMode(g_portBaseAddr[1],23u,kPortMuxAsGpio);
			GPIO_DRV_SetPinOutput(pin.pinName);
		} else {
			PORT_HAL_SetMuxMode(g_portBaseAddr[1],23u,kPortMuxAlt3);
		}
		break;
	default:
		break;
	}
}

/* Set everything to 1 */
void mpu9250_init_cs_gpio(void) {
	//based on nordicSwdioNrst

	cs1.config.outputLogic = 1;
	cs1.config.slewRate = kPortSlowSlewRate;
//	cs1.config.driveStrength = kPortHighDriveStrength;
	cs1.pinName = GPIO_MAKE_PIN(HW_GPIOD, 4);
	GPIO_DRV_OutputPinInit(&cs1);
}

#if MAXWELL_UART
void update_maxwell_cs(uint8_t uart_num) {
	/* Maxwell only take highest 3 bits */
  	maxwell_select = uart_num << 5;
}
#endif

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
	PORT_HAL_SetSlewRateMode(PORTC_BASE,5u,kPortSlowSlewRate);
	PORT_HAL_SetMuxMode(PORTC_BASE,5u,kPortMuxAlt2);

	// PTC6 - SPI0_SOUT
	PORT_HAL_SetSlewRateMode(PORTC_BASE,6u,kPortSlowSlewRate);
	PORT_HAL_SetMuxMode(PORTC_BASE,6u,kPortMuxAlt2);

	// PTC7 - SPI0_SIN
	PORT_HAL_SetSlewRateMode(PORTC_BASE,7u,kPortSlowSlewRate);
	PORT_HAL_SetMuxMode(PORTC_BASE,7u,kPortMuxAlt2);
#endif /* PIVOT_3_0 */

	if (! mpu9250_configure_spi())
	{
		//printf ("error configuring SPI\r\n");
		return false;
	}

	return true;
}

bool mpu9250_init_regs(void) {
	sensor_write_reg(MPU9250_REG_SIGNAL_PATH_RESET,  0x07);  // signal path reset = gyro_rst + accel_rst + tmp_rst
	sensor_write_reg(MPU9250_REG_USER_CTRL,          0x07);  // fifo_rst + i2c_mst_rst + sig_cond_rst
								 	 	 	 	 	  // fifo_rst and i2c_mst_rst auto-clear after one clock cycle
													  // sig_cond_rst might also auto-clear; doc is unclear
	OSA_TimeDelay(10);
	sensor_write_reg(MPU9250_REG_SIGNAL_PATH_RESET,  0x00);  // clear signal path reset

	sensor_write_reg(MPU9250_REG_I2C_SLV0_CTRL,      0x00);  // disable
	sensor_write_reg(MPU9250_REG_I2C_SLV1_CTRL,      0x00);  // disable
	sensor_write_reg(MPU9250_REG_I2C_SLV2_CTRL,      0x00);  // disable
	sensor_write_reg(MPU9250_REG_I2C_SLV3_CTRL,      0x00);  // disable
	sensor_write_reg(MPU9250_REG_I2C_SLV4_CTRL,      0x00);  // disable
	sensor_write_reg(MPU9250_REG_USER_CTRL,          0x20);  // I2C_MST_EN

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

	write_mpu9250_mag_reg(AK8963_REG_CNTL1,           0x0f);  // enter fuse access mode
	ak8963_sensitivity_cal[X] = read_mpu9250_mag_reg(AK8963_REG_ASAX);
	ak8963_sensitivity_cal[Y] = read_mpu9250_mag_reg(AK8963_REG_ASAY);
	ak8963_sensitivity_cal[Z] = read_mpu9250_mag_reg(AK8963_REG_ASAZ);
	write_mpu9250_mag_reg(AK8963_REG_CNTL1,           0x00);  // power-down, necessary to exit fuse access mode

	/* WARNING: sample rate bigger than 100 Hz is NOT supported!!
	 * If you set TS_MAG_RATE to something bigger than 100 Hz, it should generate compile error.
	 * In the case it doesn't generate compile error, it will set to 100 Hz here.
	 */
	if (mag_rate >= 100) {
		write_mpu9250_mag_reg(AK8963_REG_CNTL1,           0x16);  // BIT (16-bit), continuous mode 2 (100 Hz)
	} else if (mag_rate == 8) {
		write_mpu9250_mag_reg(AK8963_REG_CNTL1,           0x12);  // BIT (16-bit), continuous mode 1 (8 Hz)
	} else {
		/* Result is not guaranteed! */
		mpu9250_set_compass_sample_rate(mag_rate);
	}

	sensor_write_reg(MPU9250_REG_USER_CTRL,          0x00);  // !I2C_MST_EN

	return true;
}

bool mpu9250_start(void)
{
	if (running[mpu9250_num])
		return false;

	sensor_write_reg(MPU9250_REG_USER_CTRL,          0x00);  // !I2C_MST_EN
	sensor_write_reg(MPU9250_REG_USER_CTRL,          0x02);  // !I2C_MST_EN | I2C_MST_RST
	sensor_write_reg(MPU9250_REG_USER_CTRL,          0x00);  // !I2C_MST_EN
	sensor_write_reg(MPU9250_REG_USER_CTRL,          0x20);  // I2C_MST_EN

	sensor_write_reg(MPU9250_REG_SMPLRT_DIV,         SMPLRT_DIV);  // sample rate divisor = 10, so 100 Hz output

	// enable reading magnetometer
	sensor_write_reg(MPU9250_REG_I2C_SLV0_ADDR,      0x80 + AK8963_I2C_ADDR);  // read from 0x0c
	sensor_write_reg(MPU9250_REG_I2C_SLV0_REG,       AK8963_FIRST_REG);
	sensor_write_reg(MPU9250_REG_I2C_SLV0_CTRL,      0x80 + AK8963_REG_COUNT);  // enable auto read, length

	running[mpu9250_num] = true;
	return true;
}

static bool mpu9250_self_test_start(void)
{
	if (running[mpu9250_num])
		return false;

	sensor_write_reg(MPU9250_REG_USER_CTRL,          0x00);  // !I2C_MST_EN
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
//
//	if (valid_cs[mpu9250_num] == 4) {
//		sensor_write_reg(MPU9250_REG_INT_ENABLE,         0x00);  // disable data ready interrupt
//	}
	sensor_write_reg(MPU9250_REG_SMPLRT_DIV,         SMPLRT_DIV);  // sample rate divisor = 10, so 100 Hz output
	sensor_write_reg(MPU9250_REG_I2C_SLV0_CTRL,      0x00);  // disable reading magnetometer
	return true;
}


bool mpu9250_data_avail(void)
{
	uint8_t data;

	sensor_read_regs(MPU9250_REG_INT_STATUS, 1, &data);

	if (! data & 0x01) {
		return false;
	}

	return true;
}

void mpu9250_clear_data_avail(void)
{
		data_avail = false;
}


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

	sensor_data->valid_sensors = 0;
	sensor_data->timestamp = OSA_TimeGetMsec();

	if (self_test)
		sensor_read_regs(MPU9250_FIRST_REG, MPU9250_SELF_TEST_REG_COUNT, data);
	else
		sensor_read_regs(MPU9250_FIRST_REG, MPU9250_REG_COUNT, data);

	//cwati TODO PIVOT_3_0_POC.  The following is causing issues with PIVOT 3.0 POC
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
	if ((sensors & SENSOR_MAG) && (data[AK8963_REG_OFFSET(AK8963_REG_STATUS_1)] & 0x01))
	{
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

//	if (accel_out_of_range)
//	{
//		if (flags & ST_FLAG_DEBUG_PRINTF)
//			printf("accel data out of range\r\n");
//	}
//
//	if (gyro_out_of_range)
//	{
//		if (flags & ST_FLAG_DEBUG_PRINTF)
//			printf("gyro data out of range\r\n");
//	}

	for (int axis = X; axis <= Z; axis++)
	{
		if (flags & ST_FLAG_ACCEL)
		{
//			if (flags & ST_FLAG_DEBUG_PRINTF)
//				printf("accel %c accum: %d\r\n", 'X'+axis, accel_accum[axis]);
			accel_accum[axis] /= count;
			if ((accel_accum[axis] < -32767) || (accel_accum[axis] > 32767))
				status = false;
			results->accel[axis] = accel_accum[axis];
		}
		if (flags & ST_FLAG_GYRO)
		{
//			if (flags & ST_FLAG_DEBUG_PRINTF)
//				printf("gyro %c accum: %d\r\n", 'X'+axis, gyro_accum[axis]);
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
//					if (fzlags & ST_FLAG_DEBUG_PRINTF)
//						("accel %c above absolute limit, %d > %d\r\n", 'X'+axis, accel_response, ACCEL_ST_ABS_LIMIT);
					status++;
				}
			}
			else
			{
				float factory = factory_st_decode(SELFTEST_ACCEL_FS_SEL, factory_st->accel[axis]);
				float ratio = accel_response / factory;
	            if ((ratio < 0.5) || (ratio > 1.5))
	            {
//	    			if (flags & ST_FLAG_DEBUG_PRINTF)
//	    				printf("accel %c response too far from factory\r\n", 'X'+axis, accel_response, (int)factory);
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
//					if (flags & ST_FLAG_DEBUG_PRINTF)
//						printf("gyro %c below absolute limit, %d < %d\r\n", 'X'+axis, gyro_response, GYRO_ST_ABS_LIMIT);
					status++;
				}
			}
			else
			{
				float factory = factory_st_decode(SELFTEST_GYRO_FS_SEL, factory_st->gyro[axis]);
				float response = gyro_response / factory;
	            if (response < 0.5)
	            {
//	    			if (flags & ST_FLAG_DEBUG_PRINTF)
//	    				printf("gyro %c response less than half of factory, %d, %d\r\n", 'X'+axis, gyro_response, (int)factory);
	            	status++;
	            }
			}

			// also have to check gyro offset
			if (abs(normal->gyro[axis]) > GYRO_MAX_OFFSET)
			{
//				if (flags & ST_FLAG_DEBUG_PRINTF)
//					printf("gyro %c offset too large, %d > %d\r\n", 'X'+axis, normal->gyro[axis], GYRO_MAX_OFFSET);
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

