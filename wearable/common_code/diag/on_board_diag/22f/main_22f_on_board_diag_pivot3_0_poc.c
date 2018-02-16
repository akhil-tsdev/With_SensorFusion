/*
 * Copyright (c) 2015 TuringSense
 *
 * main_22f_sensor.c
 *
 * TuringSense K22F Sensor Module.
 * Read from sensor is using SPI.  Initializing Magnetometer is using I2C.
 *
 * This is the main file for satellite 22F code.
 * Different HW revision will use the same set of files, only different
 * defines as follows:
 *
 * For sensor module: PROTO_BOARD
 * For 1st production: PRODUCTION1
 */
/*******************************************************************************
 * Standard C Include Files
 ******************************************************************************/
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * SDK Include Files
 ******************************************************************************/
#include "fsl_device_registers.h"
#include "fsl_os_abstraction.h"
#include "fsl_dspi_master_driver.h"
#include "fsl_dspi_slave_driver.h"
#include "board.h"
#include "mpu9250_freq.h"
#include "ts_rtc.h"

/* USB */
void VirtualCom_Deinit(); //virtual_com.h
#include "fsl_debug_console.h"

#include "crc16.h"
#include "dtm_common_types.h"

/* PIVOT 3.0 POC */
#include "main_22f_on_board_diag_pivot30poc.h"
#include "dtmTestUtilities_pivot3_0_poc.h"
#include "mpu9250_pivot30poc.h"

#define MPU9250_VALID_WHO_AM_I	113

/*******************************************************************************
 * Freescale Open Source Sensor Fusion (OSSF) Include Files
 ******************************************************************************/
#include "build.h"
#include "tasks.h"
#include "magnetic.h"
/*******************************************************************************
 * TuringSense Include Files
 ******************************************************************************/
#include "common_types.h"
#include "common_err.h"
#include "sat_module.h"
#include "ts_fusion.h"
#include "cbuf.h"



#define NORDIC_CALLBACK_TODO 0

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * BIAS DETECTION AND BIAS SETTING *** NOTE! VALID FOR SINGOL MEMS
 ******************************************************************************/
#define SAT_BIAS_SENDING 	false
#define SAT_MAGBIAS_X 	 0
#define SAT_MAGBIAS_Y    0
#define SAT_MAGBIAS_Z    0
parameters_t parameters;

#define SAT_BIAS_ALWAYSACTIVECALIBRATION 	false //Even if set the fixed values of BIAS (with SAT_BIAS_SENDING=false), calibration of the magnetometer remains active. In this case the calibration values will be recalculated

#define USE_HWE_DEBUG_CONFIGURATION 1
static bool read_sensors_launch_fusion (sensor_data_t *raw_data, sensor_record_t* data);

#if PACE_SENSOR_OUTPUT
static bool alternateSending = true;
#endif

// The following defines determine what things are output by default to the
// USB CDC. Don't ifdef code based on these, as it will prevent the ability to
// control at runtime by changing the boolean variables (e.g., with a debugger).


#define ENABLE_PRINTF	1 /* Unless you set this to 1, no printf will be enabled.
							 * If you enable this to 1, you must connect the USB to let the printf got out,
							 * otherwise the code will get stuck waiting to print.. */

#define DEBUG_GENERAL   1
#define DEBUG_RAW_DATA  0
#define DEBUG_SENSOR_FUSION_OUTPUT 0
#define DEBUG_NORDIC    0
#define OUTPUT_3D_CUBE  0
#define DEBUG_PRINT_LOG 0

hub_err_t hub_err_status = hub_no_err;

uint32_t reset_rtc_diff = 0; /* Elapsed time before resetting RTC happens.*/

static volatile bool hw_init_done = false;

/* Heartbeat check for Nordic communication */
uint32_t last_nordic_comm;
const uint32_t max_nordic_quiet = 1000; /* in ms */

#if PROTO_BOARD
#define RED_LED_GPIO	4   /* K22F's GPIOA pin number for red LED */
#define GREEN_LED_GPIO	5   /* K22F's GPIOA pin number for green LED */
#define BLUE_LED_GPIO	12  /* K22F's GPIOA pin number for blue LED */
#else
#define RED_LED_GPIO	12   /* K22F's GPIOA pin number for red LED */
#define GREEN_LED_GPIO	13   /* K22F's GPIOA pin number for green LED */
#define BLUE_LED_GPIO	14  /* K22F's GPIOA pin number for blue LED */
#define NORDIC_SWDIO_NRST	19	/* K22F's PTB 19 is for Nordic soft reset */
#endif


extern uint32_t g_dspiBaseAddr[];
static dspi_slave_state_t nrf51822_dspiSlaveState;


static cbuf_t sensorRecordQ = { 0 }; // Queue of sensor data records (see sat_module.h and cbuf.h)
// Power up defaults: don't collect sensor data, but if told to start
// collecting, send it right away.
static volatile bool sensor_active = false;
static volatile bool sensor_sending = false;

//static uint32_t timeAtBootInMillis = 0;
static int currSourceByte = 0;  // SPI data byte position for SPI slave
static gpio_output_pin_user_config_t ledRedPin;    // Red LED pin
static gpio_output_pin_user_config_t ledGreenPin;  // Green LED pin
static gpio_output_pin_user_config_t ledBluePin;   // Blue LED pin
#if PRODUCTION1
static gpio_output_pin_user_config_t nordicSwdioNrst;
#endif
dspi_status_t nrf51822_init();
#if ENABLE_RTC
uint32_t k22f_get_rtc();
#endif
static void sensorFusionInit();


/* PIVOT 3.0 POC */
uint8_t valid_num_sensors_in_cs = 0;
uint8_t valid_cs[MAX_SPI_CS] = {0};

sat_to_nordic_dtm_t spi_tx;
nordic_to_sat_dtm_t spi_rx;

/* EM USB related stuff */
enum _mainState
{
	mainUSBstart,
	mainUSBstartWait,
	mainSensorInitialization,
	mainLoop
};
static enum _mainState mainState = mainUSBstart;

extern void VC_main();
extern void APP_init();
extern void Task_Start(void *arg);
void Main_Start(void *arg);

void Test_task(void);

/*******************************************************************************
 * Control application and/or debug output
 ******************************************************************************/

// The following variables can be changed at runtime (e.g. by a debugger)
// to control what output goes to the USB CDC.

bool debug_output = DEBUG_GENERAL;
bool debug_nordic = DEBUG_NORDIC;
bool debug_raw_data = DEBUG_RAW_DATA;
bool debug_sensor_fusion_output = DEBUG_SENSOR_FUSION_OUTPUT;
bool cube_output = OUTPUT_3D_CUBE;
bool debug_log_output = DEBUG_PRINT_LOG;

void cube_debug_print_str(char *p)
{
	static bool first = true;
	int l = strlen(p);
	if (first)
	{
		puts("");
		first = false;
	}
	while (l)
	{
		int i;
		putchar('$');
		putchar(1);
		putchar(0);
		for (i = 3; i <= 21; i++)
		{
			if (l)
			{
				putchar(*(p++));
				l--;
			}
			else
				putchar(0);
		}
	}
}

int cond_printf(bool cond, const char *fmt, ...)
{
	va_list ap;
	int rv = 0;
	static int reentrancy_count = 0;
	static char pbuf[200];

	if (!cond)
		return 0;

	// Use gcc atomic add intrinsic to increment a semaphore and test whether
	// we're being called reentrantly, which should never happen, but I saw it
	// once when debugging. If called reentrantly, skip doing anything.
	if (__sync_fetch_and_add(& reentrancy_count, 1) == 0)
	    {
		va_start(ap, fmt);
		rv = vsnprintf(pbuf, sizeof(pbuf), fmt, ap);
		va_end(ap);
		if (cube_output)
			cube_debug_print_str(pbuf);
		else
		    puts(pbuf);
    }
    __sync_fetch_and_add(& reentrancy_count, -1);
	return rv;
}

int cond_putchar(bool cond, int c)
{
	if (cond)
		putchar(c);
	return c;
}

static uint32_t sRandomSeed;

void initRandom()
{
	sRandomSeed = 0x02;
}


uint32_t getRandom(uint8_t is_prbs9)
{
	uint8_t a = (is_prbs9 ? 6 : 14);
	uint32_t new_v = (((sRandomSeed >> a) ^ (sRandomSeed >> (a-1))) & 0x01);
	sRandomSeed = ((sRandomSeed << 1) | new_v) & 0x7F;
	return sRandomSeed;
}

#if ENABLE_PRINTF
#define debug_printf(...) cond_printf(debug_output, __VA_ARGS__)

#define debug_raw_data_printf(...) cond_printf(debug_raw_data, __VA_ARGS__)

#define debug_sensor_fusion_output_printf(...) cond_printf(debug_sensor_fusion_output, __VA_ARGS__)

#define debug_nordic_printf(...) cond_printf(debug_nordic, __VA_ARGS__)

#define cube_printf(...) cond_printf(cube_output, __VA_ARGS__)
#define cube_putchar(c) cond_putchar(cube_output, c)

#define debug_log_printf(...) cond_printf(debug_log_output, __VA_ARGS__)

#else

void no_printf(bool cond, const char *fmt, ...) {
	return;
}
#define debug_printf(...) no_printf(debug_output, __VA_ARGS__)
#define debug_raw_data_printf(...) no_printf(debug_raw_data, __VA_ARGS__)
#define debug_sensor_fusion_output_printf(...) no_printf(debug_sensor_fusion_output, __VA_ARGS__)
#define debug_nordic_printf(...) no_printf(debug_nordic, __VA_ARGS__)
#define cube_printf(...) no_printf(cube_output, __VA_ARGS__)
#define cube_putchar(c) no_printf(cube_output, c)
#define debug_log_printf(...) no_printf(debug_log_output, __VA_ARGS__)

#endif /* ENABLE_PRINTF */

// macros to prevent the direct use of the underlying functions
// Note that redefining putchar() will cause a warning, which can be ignored.


/*******************************************************************************
 * Turn on LED of requested color
 ******************************************************************************/
void turnOnLED(sat_led_t color) {
	// LED is active low, clear pin output will turn on
	switch (color) {
	case offLed:
		GPIO_DRV_SetPinOutput(ledGreenPin.pinName);
		GPIO_DRV_SetPinOutput(ledBluePin.pinName);
		GPIO_DRV_SetPinOutput(ledRedPin.pinName);
		break;
	case redLed:
		GPIO_DRV_SetPinOutput(ledGreenPin.pinName);
		GPIO_DRV_SetPinOutput(ledBluePin.pinName);
		GPIO_DRV_ClearPinOutput(ledRedPin.pinName);
		break;
	case greenLed:
		GPIO_DRV_ClearPinOutput(ledGreenPin.pinName);
		GPIO_DRV_SetPinOutput(ledBluePin.pinName);
		GPIO_DRV_SetPinOutput(ledRedPin.pinName);
		break;
	case blueLed:
		GPIO_DRV_SetPinOutput(ledGreenPin.pinName);
		GPIO_DRV_ClearPinOutput(ledBluePin.pinName);
		GPIO_DRV_SetPinOutput(ledRedPin.pinName);
		break;
	case yellowLed:
		GPIO_DRV_ClearPinOutput(ledGreenPin.pinName);
		GPIO_DRV_SetPinOutput(ledBluePin.pinName);
		GPIO_DRV_ClearPinOutput(ledRedPin.pinName);
		break;
	case purpleLed:
		GPIO_DRV_SetPinOutput(ledGreenPin.pinName);
		GPIO_DRV_ClearPinOutput(ledBluePin.pinName);
		GPIO_DRV_ClearPinOutput(ledRedPin.pinName);
		break;
	case turqoiseLed:
	default:
		GPIO_DRV_ClearPinOutput(ledGreenPin.pinName);
		GPIO_DRV_ClearPinOutput(ledBluePin.pinName);
		GPIO_DRV_SetPinOutput(ledRedPin.pinName);
		break;
	}
}

/*******************************************************************************
 * Nordic soft reset
 * Using SWDIO_nRST_K22F_nRF51
 * Minimum 100ms-150ms delay in between.
 ******************************************************************************/
void nrf51822_swdio_reset() {
	uint32_t green, blue, red;
	green = GPIO_DRV_ReadPinInput(ledGreenPin.pinName);
	blue = GPIO_DRV_ReadPinInput(ledBluePin.pinName);
	red = GPIO_DRV_ReadPinInput(ledRedPin.pinName);

	GPIO_DRV_ClearPinOutput(nordicSwdioNrst.pinName);

	turnOnLED(offLed);
	OSA_TimeDelay(200); /* 200ms */

	GPIO_DRV_SetPinOutput(nordicSwdioNrst.pinName);

	/* Turning LEDs back on to its original state.
	 * 0 means LED is on. */
	if (!green) {
		GPIO_DRV_ClearPinOutput(ledGreenPin.pinName);
	}
	if (!blue) {
		GPIO_DRV_ClearPinOutput(ledBluePin.pinName);
	}
	if (!red) {
		GPIO_DRV_ClearPinOutput(ledRedPin.pinName);
	}
}

/*
 * Calling this will turn off and turn back on to whatever initial color was.
 */
void blinkLED() {
	uint32_t green, blue, red;

	green = GPIO_DRV_ReadPinInput(ledGreenPin.pinName);
	blue = GPIO_DRV_ReadPinInput(ledBluePin.pinName);
	red = GPIO_DRV_ReadPinInput(ledRedPin.pinName);

	turnOnLED(offLed);

	/* 0 means LED is on */
	if (!green) {
		GPIO_DRV_ClearPinOutput(ledGreenPin.pinName);
	}
	if (!blue) {
		GPIO_DRV_ClearPinOutput(ledBluePin.pinName);
	}
	if (!red) {
		GPIO_DRV_ClearPinOutput(ledRedPin.pinName);
	}
}
/* Upon unrecoverable error, we call this function and
 * it will blink the LED.
 *
 * WARNING: This function does NOT return!!!
 * It's an infinite loop!!
 */
void blinkLEDerror(sat_led_t color, uint32_t multiplier) {
	 debug_printf("Unrecoverable error happened!");

	while (1) {
		turnOnLED(color);
		OSA_TimeDelay(300 * multiplier);
		turnOnLED(offLed);
		OSA_TimeDelay(300 * multiplier);
	}
}

void onlyTest(bool isCommand){
	OSA_TimeDelay(500);
	if(isCommand)
		turnOnLED(redLed);
	else
		turnOnLED(yellowLed);

	OSA_TimeDelay(500);
	turnOnLED(offLed);
}

void testCommand0(sat_mosi_packet_t packet){ //use command=0
	turnOnLED(turqoiseLed);
	OSA_TimeDelay(500);
	onlyTest(packet.sat_command[SAT_COMMSTRUCT_VALX1] == 1);
	onlyTest(packet.sat_command[SAT_COMMSTRUCT_VALY2] == 2);
	onlyTest(packet.sat_command[SAT_COMMSTRUCT_VALZ3] == 3);
}

void testCommand1(sat_mosi_packet_t packet){ //use command=1
	turnOnLED(purpleLed);
	OSA_TimeDelay(500);
	onlyTest(packet.sat_command[SAT_COMMSTRUCT_VALX1] == 1);
	onlyTest(packet.sat_command[SAT_COMMSTRUCT_VALY2] == 1);
	onlyTest(packet.sat_command[SAT_COMMSTRUCT_VALZ3] == 1);
}

int WAgiveIntoFromUint(uint32_t value){ //use command=1
	int result;
	uint32_t negativeCK=1000000000;

	if(value >= negativeCK)
		result = -((int)((value-negativeCK)));
	else
		result = (int)value;

	return result;
}




/*******************************************************************************
 * Initialize Nordic nRF51822
 ******************************************************************************/
dspi_status_t nrf51822_init()
{
	dspi_status_t dspiResult;

	// Setting up K22F as SPI Slave of Nordic nRF51822
	static const dspi_slave_user_config_t nrf51822_dspiSlaveUserConfig =
	{
		.dataConfig.clkPhase = kDspiClockPhase_FirstEdge,
		.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh,
		.dataConfig.bitsPerFrame = 8,
		.dummyPattern = 0,
	};

	configure_spi_pins(knRF51822SpiInstance);

	DSPI_DRV_SlaveDeinit(knRF51822SpiInstance);
	dspiResult = DSPI_DRV_SlaveInit(knRF51822SpiInstance, &nrf51822_dspiSlaveState,
			&nrf51822_dspiSlaveUserConfig);

	return dspiResult;
}


#if OUTPUT_3D_CUBE
/*******************************************************************************
 * Output float number in char
 ******************************************************************************/
static void put_float(float f)
{
	int32_t q;

	q = (int32_t) (f * (1<<30));  // Python demo uses fixed point +-1.30 bits

	putchar((q >> 24) & 0xff);
	putchar((q >> 16) & 0xff);
	putchar((q >> 8)  & 0xff);
	putchar(q & 0xff);
}

/*******************************************************************************
 * Display Quarternion numbers
 ******************************************************************************/
static void output_quaternion_packet(void)
{
#if 1
	printf("$1,%f,%f,%f,%f;\r\n", thisSV_9DOF_GBY_KALMAN.fqPl.q0,
			thisSV_9DOF_GBY_KALMAN.fqPl.q1,
			thisSV_9DOF_GBY_KALMAN.fqPl.q2,
			thisSV_9DOF_GBY_KALMAN.fqPl.q3);
#else
	putchar('$');
	putchar(0x02);
	putchar(0x00);
	put_float(thisSV_9DOF_GBY_KALMAN.fqPl.q0);
	put_float(thisSV_9DOF_GBY_KALMAN.fqPl.q1);
	put_float(thisSV_9DOF_GBY_KALMAN.fqPl.q2);
	put_float(thisSV_9DOF_GBY_KALMAN.fqPl.q3);
	putchar(0x00);
	putchar(0x00);
	putchar('\r');
	putchar('\n');
#endif /* 1 */
}

void main_loop_output_3d_cube(void)
{
	sensor_data_t 		raw_data;				// Raw output of MPU9250 (accel, gyro, mag)
	sat_cbuf_packet_t 	sensor_record;	// Timestamp + sensor data (raw + sensor-fusion output)

	while (true)
	{
		if (! get_raw_data_sample(&raw_data, &sensor_record.data))
			continue;
		output_quaternion_packet();
	}
}
#endif /* OUTPUT_3D_CUBE */
/*******************************************************************************
 * Read MPU9250 raw output (Accelerometer, Gyrometer, Magnetometer, Temperature)
 ******************************************************************************/
static bool get_raw_data_sample_ts(sensor_data_t *raw_data, sensor_record_t *data, uint32_t *timestamp) {
	bool ret;
//	;
//
//	while (!mpu9250_data_avail())
//		;
//	mpu9250_clear_data_avail();
	ret = mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP,
			raw_data);

	if (ret) {
		process_sensor_fusion(raw_data, data);
		*timestamp = raw_data->timestamp;
	}

	return ret;
}

/*******************************************************************************
 * Read MPU9250 raw output (Accelerometer, Gyrometer, Magnetometer, Temperature)
 ******************************************************************************/
static bool get_raw_data_sample(sensor_data_t *raw_data, sensor_record_t *data) {
	bool ret;
//	;
//
//	while (!mpu9250_data_avail())
//		;
//	mpu9250_clear_data_avail();
	ret = mpu9250_read(SENSOR_ACCEL | SENSOR_GYRO | SENSOR_MAG | SENSOR_TEMP,
			raw_data);

	if (ret) {
		process_sensor_fusion(raw_data, data);
	}

	return ret;
}

//#endif

static void sensorFusionInit() {
	/* Initializing Turing Sense sensor fusion */
	ts_fusion_init();
	mpu9250_start();

}

extern uint32_t g_xtal0ClkFreq;

/**
 * HardFaultHandler_C:
 * This is called from the HardFault_HandlerAsm with a pointer the Fault stack
 * as the parameter. We can then read the values from the stack and place them
 * into local variables for ease of reading.
 * We then read the various Fault Status and Address Registers to help decode
 * cause of the fault.
 * The function ends with a BKPT instruction to force control back into the debugger
 */
void GenericFault_HandlerC(unsigned long *hardfault_args){
  volatile unsigned long stacked_r0 ;
  volatile unsigned long stacked_r1 ;
  volatile unsigned long stacked_r2 ;
  volatile unsigned long stacked_r3 ;
  volatile unsigned long stacked_r12 ;
  volatile unsigned long stacked_lr ;
  volatile unsigned long stacked_pc ;
  volatile unsigned long stacked_psr ;
  volatile unsigned long _CFSR ;
  volatile unsigned long _HFSR ;
  volatile unsigned long _DFSR ;
  volatile unsigned long _AFSR ;
  volatile unsigned long _BFAR ;
  volatile unsigned long _MMAR ;

  stacked_r0 = ((unsigned long)hardfault_args[0]) ;
  stacked_r1 = ((unsigned long)hardfault_args[1]) ;
  stacked_r2 = ((unsigned long)hardfault_args[2]) ;
  stacked_r3 = ((unsigned long)hardfault_args[3]) ;
  stacked_r12 = ((unsigned long)hardfault_args[4]) ;
  stacked_lr = ((unsigned long)hardfault_args[5]) ;
  stacked_pc = ((unsigned long)hardfault_args[6]) ;
  stacked_psr = ((unsigned long)hardfault_args[7]) ;

  // Configurable Fault Status Register
  // Consists of MMSR, BFSR and UFSR
  _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;

  // Hard Fault Status Register
  _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

  // Debug Fault Status Register
  _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

  // Auxiliary Fault Status Register
  _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

  // Read the Fault Address Registers. These may not contain valid values.
  // Check BFARVALID/MMARVALID to see if they are valid values
  // MemManage Fault Address Register
  _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
  // Bus Fault Address Register
  _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

  __asm("BKPT #0\n") ; // Break into the debugger
}

__attribute__((naked))
void Default_HandlerC(void)
{
  __asm volatile (
    " movs r0,#4       \n"
    " movs r1, lr      \n"
    " tst r0, r1       \n"
    " beq _MSP         \n"
    " mrs r0, psp      \n"
    " b _HALT          \n"
  "_MSP:               \n"
    " mrs r0, msp      \n"
  "_HALT:              \n"
    " ldr r1,[r0,#20]  \n"
    " b GenericFault_HandlerC \n"
    " bkpt #0          \n"
  );
}

uint32_t calculateSATID() {
            //uint32_t MOD=1073741824;
            //return SIM_UIDH%MOD+SIM_UIDMH%MOD+SIM_UIDML%MOD+SIM_UIDL%MOD;
	return (( SIM_UIDML >> 16) & 0x007F) | ((( SIM_UIDMH >> 0) & 0x007F) << 7) | ((( SIM_UIDMH >> 16) & 0x000F) << 14);
}

uint8_t get_sensor_num_from_cs(uint8_t cs) {
	uint8_t sensor_num;

	for(sensor_num = 0; sensor_num < MAX_SPI_CS; sensor_num++) {
		if (valid_cs[sensor_num] == cs) {
			return sensor_num;
		}
	}

	return UINT8_MAX;
}

/*******************************************************************************
 * Main routine
 ******************************************************************************/
int main(void) {
	sensor_data_t raw_data;			// Raw output of MPU9250 (accel, gyro, mag)
	sat_cbuf_packet_t sensor_record;// Timestamp + sensor data (raw + sensor-fusion output)
	uint32_t current_ts10, time_now;
	bool is_sleeping = false, retbool;
    dspi_status_t dspiResult;
    uint32_t 				wordsTransfer = 0;
    static uint16_t			last_crc16_sent = 0;
    uint16_t				generated_crc16, temp_crc16;
    bool					need_spi_resend = false;
    uint16_t				user_input;
    uint16_t 				chip_select = 0xff;
	uint8_t 				whoami;
	char 					yesno;
	bool					pass, acc_pass[MAX_SENSORS_IN_CS], gyr_pass[MAX_SENSORS_IN_CS], mag_pass[MAX_SENSORS_IN_CS];
	uint8_t					sensor_num;
#if DO_SELFTEST
	mpu9250_self_test_results_t factory_st;
	mpu9250_average_data_t avg_normal, avg_selftest;
#endif /* DO_SELFTEST */

	g_xtal0ClkFreq = 8000000U;

	VirtualCom_Deinit();

	// Initialize K22F
	hardware_init();
	if (OSA_Init() != kStatus_OSA_Success) {
		debug_printf("error initializing OSA\r\n");
		blinkLEDerror(redLed, 1);
		hub_err_status = hub_hw_err1;
	}
	configure_uart_pins(BOARD_DEBUG_UART_INSTANCE);
	dbg_uart_init();
	k22f_enable_rtc();

	// Initialize sensor record queue
	if (cbufInit(&sensorRecordQ) != E_OK) {
		debug_printf("error initializing sensorRecordQ\r\n");
		blinkLEDerror(redLed, 2);
		hub_err_status = hub_hw_err2;
	}

	// Initialize R/G/B LED pins
	// Set RED_LED_GPIO (4)
	ledRedPin.config.outputLogic = 0;
	ledRedPin.config.slewRate = kPortSlowSlewRate;
	ledRedPin.config.driveStrength = kPortHighDriveStrength;
	ledRedPin.pinName = GPIO_MAKE_PIN(GPIOA_IDX, RED_LED_GPIO);
	GPIO_DRV_OutputPinInit(&ledRedPin);

	// Set GREEN_LED_GPIO (5)
	ledGreenPin.config.outputLogic = 0;
	ledGreenPin.config.slewRate = kPortSlowSlewRate;
	ledGreenPin.config.driveStrength = kPortHighDriveStrength;
	ledGreenPin.pinName = GPIO_MAKE_PIN(GPIOA_IDX, GREEN_LED_GPIO);
	GPIO_DRV_OutputPinInit(&ledGreenPin);
	// Set BLUE_LED_GPIO (12)
	ledBluePin.config.outputLogic = 0;
	ledBluePin.config.slewRate = kPortSlowSlewRate;
	ledBluePin.config.driveStrength = kPortHighDriveStrength;
	ledBluePin.pinName = GPIO_MAKE_PIN(GPIOA_IDX, BLUE_LED_GPIO);
	GPIO_DRV_OutputPinInit(&ledBluePin);

#if PRODUCTION1
	// Set NORDIC soft reset (PTB19)
	nordicSwdioNrst.config.outputLogic = 1;
	nordicSwdioNrst.config.slewRate = kPortSlowSlewRate;
	nordicSwdioNrst.config.driveStrength = kPortHighDriveStrength;
	nordicSwdioNrst.pinName = GPIO_MAKE_PIN(GPIOB_IDX, NORDIC_SWDIO_NRST);
	GPIO_DRV_OutputPinInit(&nordicSwdioNrst);
#endif

#if OUTPUT_3D_CUBE
	turnOnLED(blueLed);
#endif /* OUTPUT_3D_CUBE */
#if DEBUG
	long int i;
	float test;
	for(i=0; i<10; i++)
		debug_printf("HELLO TS CDC, %d \n\r", i);
	for(i=0; i<10; i++){
		test += 101.25;
		debug_printf("FLOAT VALUE = %f \n\r", test);
	}
#endif /* DEBUG */

	hw_init_done = true;

	// Initialize nRF51822 (configure SPI0 slave, data structures, etc.)
//	debug_printf("\r\nInitializing Nordic nRF51822\r\n");
//	turnOnLED(offLed);
	nrf51822_swdio_reset();

	dspiResult = nrf51822_init();
	if (dspiResult != kStatus_DSPI_Success)
    {
		debug_printf("\r\n ERROR: Can not initialize DSPI slave driver \r\n");
		blinkLEDerror(redLed, 4);
    };

	// Done with all initializations, turn on GREEN LED, entering Main loop
	turnOnLED(greenLed);
//	debug_printf("GOT HERE \n\r");
	debug_printf("***********************************************\r\n");
	debug_printf("\r\nSensor Rev E Bringup Version %s Date %s", TMP_VERSION, TMP_VERSION_DATE);
	debug_printf("\r\nTuringsense DTM Utility Version %s\r\n", VERSION_RELEASE);

	debug_printf("[SATDTM=" VERSION_RELEASE "]\r\n");
	debug_printf("[SATIDX=%d]\r\n", calculateSATID());

	// Main loop
#if OUTPUT_3D_CUBE
	main_loop_output_3d_cube();
#else

	bool alternateSending = true;
	last_nordic_comm = k22f_get_rtc();
	initRandom();


	parameters.spi_attemptLimit =100;
	parameters.spi_numberOfTests =1000;
	parameters.spi_max_crc_fail = 10;
	parameters.spi_max_attempt_for_packet = 10;
	parameters.spiSensors_numberOfTests =100;
	parameters.log_enableDetail = 0;
	parameters.spiSensor_max_WR_fail = 1;
	while (true) {
		debug_printf("\n\r***********************************************\r");
		debug_printf("M| Sensor Rev E Bringup Version %s Date %s\r", TMP_VERSION, TMP_VERSION_DATE);
		debug_printf("M| DTM TEST ENVIRONMENT version %s. Choose an optionX:\r",VERSION_RELEASE);
		debug_printf("M|  0 - Options\r");
		debug_printf("M|  1 - Set tolerances\r");
		debug_printf("M|  2 - Execute test 1.A: debug SPI\r");
		debug_printf("M|  3 - Execute test 1.B: debug SPI: write/read sensors registers\r");
		debug_printf("M|  4 - Execute test 1.C: sensors self test\r");
		debug_printf("M|  5 - Get FW version\r");
		debug_printf("M|  7 - Generate SAT_ID\r");
		debug_printf("M|  8 - RF test\r");
		debug_printf("M|  9 - RF reset parameters\r");
		debug_printf("M|  10- Magnetometer Output Test\r");
		debug_printf("M|  11- Garment Test\r");
		debug_printf("E|\r\n");

#if CWATI_NO_PRINTF
		user_input = 4;
#else
		debug_scanf("%u", &user_input);
#endif

		if(user_input == 0){
			debug_printf("S| OPTIONS *****************************************\r\n");

			debug_printf("S|    1 - SPI TEST: set number of attempts packet forwarding. Actual value %u\r\n", parameters.spi_attemptLimit);
			debug_printf("S|    2 - SPI TEST: set number of packages to be shipped. Actual value %d\r\n", parameters.spi_numberOfTests);
			debug_printf("S|    3 - Set Nordic parameters\r\n");
			debug_printf("S|    4 - SPIsensor register W/R:set number of write/read cycles. Actual value %u\r\n", parameters.spiSensors_numberOfTests);
			debug_printf("S|    5 - Set flag enable detailed log. Actual value %u\r\n", parameters.log_enableDetail);
			debug_printf("E|\r\n");
			debug_scanf("%u", &user_input);

			if(user_input==1){
				debug_printf("S|       1) enter number of attempts packet forwarding:\r\n");
				debug_printf("E|\r\n");
				debug_scanf("%u", &user_input);
				parameters.spi_attemptLimit = user_input;
			}else if(user_input==2){
				debug_printf("S|       2)enter number of packages to be shipped :\r\n");//through SPI
				debug_printf("E|\r\n");

				debug_scanf("%u", &(parameters.spi_numberOfTests));
			}else if(user_input==3){
				setNordicParameters(2);
			}else if(user_input==4){
				debug_printf("S|       4)enter number of sensor register write/read cycle:\r\n");
				debug_printf("E|\r\n");
				debug_scanf("%u", &(parameters.spiSensors_numberOfTests));
			}else if(user_input==5){
				debug_printf("S|       5) enter detailed log (0 off - 1 on - 2 all on):\r\n");
				debug_printf("E|\r\n");
				debug_scanf("%u", &user_input);
				parameters.log_enableDetail = user_input;
			}
		}else if(user_input == 1){
			debug_printf("S| SET TOLERANCES **********************************\r\n");

			debug_printf("S|    0 - Show result codes\r\n");
			debug_printf("S|    1 - SPI TEST: number of FAILED CRD to FAIL test. Actual value %u\r\n", parameters.spi_max_crc_fail);
			debug_printf("S|    2 - SPIsensor TEST: number of FAILED W/R registers. Actual value %u\r\n", parameters.spiSensor_max_WR_fail);
			debug_printf("E|\r\n");
			debug_scanf("%u", &user_input);

			if(user_input==0){
				showResultTag();
			}else if(user_input==1){
				debug_printf("S|       1) SPI TEST: set number of FAILED CRD to FAIL test:\r\n");
				debug_printf("E|\r\n");
				debug_scanf("%u", &(parameters.spi_max_crc_fail));
			}else if(user_input==2){
				debug_printf("S|       2) SPIsensor TEST: set number of FAILED W/R to FAIL test:\r\n");
				debug_printf("E|\r\n");
				debug_scanf("%u", &(parameters.spiSensor_max_WR_fail));
			}
		} else if(user_input == 2) {
			spiTest(parameters);
		} else if(user_input == 3) {
			mpu9250_init_cs_gpio();
			mpu9250_init_all_pins(); //cwati all except for CS

			chip_select = 0xff;
			while (chip_select >= MAX_SPI_CS) {
				debug_printf("\r\nWhich sensor (chip select)? [0-%u]: ", MAX_SPI_CS-1);
				debug_scanf("%u", &chip_select);
			}

			debug_printf("\r\nEnabling chip select %u...", chip_select);
			mpu9250_detect_sensor(chip_select, &whoami);

			if (whoami == 113) {
				debug_printf("\r\nRunning SPI Sensor Register Write Test..");
				spiSensorRegisterWriteTest(parameters, &pass);
			} else {
				debug_printf("\r\nCan't reach sensor in chip select %u", chip_select);
			}

		} else if (user_input == 4)
		{
			bool accpass, gyrpass, magpass;

			mpu9250_init_cs_gpio();
			mpu9250_init_all_pins(); //cwati all except for CS

			chip_select = 0xff;
			while (chip_select >= MAX_SPI_CS) {
				debug_printf("\r\nWhich sensor (chip select)? [0-%u]: ", MAX_SPI_CS-1);
				debug_scanf("%u", &chip_select);
			}

			mpu9250_detect_sensor(chip_select, &whoami);

			if (whoami == 113) {
				debug_printf("\r\nDo you want to check Initial Sensor Data? (y/n):");
				debug_scanf("%c", &yesno);

				if (yesno == 'y' || yesno == 'Y') {
					mpu9250_init_regs();
					checkInitialSensorData(parameters, false/* print result */, &accpass, &gyrpass, &magpass);
				}
			} else {
				debug_printf("\r\nSensor in CS %u is not properly read!", chip_select);
			}
		}
		else if (user_input == 5)
			debug_printf("[SATDTM=" VERSION_RELEASE "]\r\n");

		else if (user_input == 7){
			debug_printf("[SATIDX=%d]\r\n", calculateSATID());
			debug_printf("[SATIDX_UIDH=%d]\r\n", SIM_UIDH);
			debug_printf("[SATIDX_UIDMH=%d]\r\n", SIM_UIDMH);
			debug_printf("[SATIDX_UIDML=%d]\r\n", SIM_UIDML);
			debug_printf("[SATIDX_UIDL=%d]\r\n", SIM_UIDL);
		}
		else if(user_input == 8)
		{
			int freqTmp = spi_tx.dtm_tx_freq;
			int powerTmp = spi_tx.dtm_power;

			// ask input params to the user
		    setNordicParameters(2);

		    // send the new params to the nrd device
		    spiSendUntilAcked();

			spi_tx.dtm_tx_freq = freqTmp;
			spi_tx.dtm_power = powerTmp;

		}else if(user_input == 9)
		{
			// set and send the reset params to the nrd device
			spiResetParams();
			for (int i = 0; i < 10; i++)
				spiSendReceive();

			debug_printf("RF parameters successfully defaulted \r\n ");
			debug_printf("E|\r\n");
		} else if (user_input == 10)
		{
			mpu9250_init_cs_gpio();
			mpu9250_init_all_pins(); //cwati all except for CS
			uint8_t cs;

			valid_num_sensors_in_cs = 0;
			for(cs = 0; cs < MAX_SPI_CS; cs++) {

				mpu9250_detect_sensor(cs, &whoami);

				/* Valid sensor detected */
				if (whoami == MPU9250_VALID_WHO_AM_I) {
					debug_printf("\r\nSensor %u cs %u: found\n", valid_num_sensors_in_cs, cs);
					valid_cs[valid_num_sensors_in_cs] = cs;

					mpu9250_enable_sensor(valid_num_sensors_in_cs);

					/* Turn on LED for indicator of a sensor getting found */
					turnOnLED(greenLed);

					mpu9250_init_regs();

					valid_num_sensors_in_cs++;
					if (valid_num_sensors_in_cs == MAX_SENSORS_IN_CS) {
						/* Can't have more sensors, break... */
						break;
					}
				}
			}

			checkMagOutput(parameters);

		}  else if (user_input == 11)
		{
			mpu9250_init_cs_gpio();
			mpu9250_init_all_pins(); //cwati all except for CS
			uint8_t cs;
			bool spi_integrity_result[MAX_SPI_CS] = {0};
			bool acc_result [MAX_SPI_CS] = {0};
			bool gyr_result [MAX_SPI_CS] = {0};
			bool mag_result [MAX_SPI_CS] = {0};

			debug_printf("\r\nIndividual Sensor Test");
			valid_num_sensors_in_cs = 0;
			for(cs = 0; cs < MAX_SPI_CS; cs++) {

				mpu9250_detect_sensor(cs, &whoami);

				/* Valid sensor detected */
				if (whoami == MPU9250_VALID_WHO_AM_I) {
					if (parameters.log_enableDetail > 1) {
						debug_printf("\r\nSensor %u cs %u: found\n", valid_num_sensors_in_cs, cs);
					}
					valid_cs[valid_num_sensors_in_cs] = cs;

					mpu9250_enable_sensor(valid_num_sensors_in_cs);

					/* Turn on LED for indicator of a sensor getting found */
					turnOnLED(greenLed);

					mpu9250_init_regs();

					/* SPI read/write register test */
					sensorDataIntegrityTest_spi(parameters, &spi_integrity_result[valid_num_sensors_in_cs]);

					checkInitialSensorData(parameters, true, &acc_pass[valid_num_sensors_in_cs], &gyr_pass[valid_num_sensors_in_cs],
							&mag_pass[valid_num_sensors_in_cs]);

					valid_num_sensors_in_cs++;
					if (valid_num_sensors_in_cs == MAX_SENSORS_IN_CS) {
						/* Can't have more sensors, break... */
						break;
					}
				}

			}
			/* Print results from Individual sensor test */
			debug_printf("\n\rSENSOR(cs)\t|SPI     |Accel   |Gyro    |Mag     |PASS/FAIL");
			debug_printf("\r----------------------------------------------------------------------");

			for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++) {
				debug_printf("\r%u(%u)\t\t|%s|%s|%s|%s|%s", sensor_num, valid_cs[sensor_num],
						spi_integrity_result[sensor_num] ? "PASS    " : "FAIL    ",
						acc_pass[sensor_num] ? "PASS    " : "FAIL    ",
						gyr_pass[sensor_num] ? "PASS    " : "FAIL    ",
						mag_pass[sensor_num] ? "PASS    " : "FAIL    ",
						spi_integrity_result[sensor_num] && acc_pass[sensor_num] && gyr_pass[sensor_num] && mag_pass[sensor_num] ? \
								"PASS    " : "FAIL    ");
			}

			debug_printf("\rAll-Sensor Magnetometer Test");
			checkMagOutput(parameters);
		}

		OSA_TimeDelay(1000); /* ms */

	}
#endif /* OUTPUT_3D_CUBE */
	return 0;  // will never reach this

} // end main()

