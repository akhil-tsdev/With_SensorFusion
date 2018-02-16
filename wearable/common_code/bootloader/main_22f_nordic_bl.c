/*
 * Copyright (c) 2015 TuringSense
 *
 * main_22f_nordic_bl.c - KSDK 1.3.0
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
//#include "mpu9250_firmware.h"
//#include "mpu9250_freq.h"
#include "ts_rtc.h"

/* USB */
void VirtualCom_Deinit(); //virtual_com.h
#include "fsl_debug_console.h"

#include "crc16.h"

/*******************************************************************************
 * Freescale Open Source Sensor Fusion (OSSF) Include Files
 ******************************************************************************/
//#include "build.h"
//#include "tasks.h"
//#include "magnetic.h"
/*******************************************************************************
 * TuringSense Include Files
 ******************************************************************************/
#include "common_types.h"
#include "common_err.h"
#include "sat_module.h"
#include "ts_fusion.h"
#include "cbuf.h"
#include "main_22f_nordic_bl.h"
#include "nordic_bl_22f.h"

#define NORDIC_CALLBACK_TODO 0

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SAT_ID			56 /* TODO later will read from memory region */

/*******************************************************************************
 * BIAS DETECTION AND BIAS SETTING *** NOTE! VALID FOR SINGOL MEMS
 ******************************************************************************/
#define SAT_BIAS_SENDING 	false
#define SAT_MAGBIAS_X 	 0
#define SAT_MAGBIAS_Y    0
#define SAT_MAGBIAS_Z    0

#define SAT_BIAS_ALWAYSACTIVECALIBRATION 	false //Even if set the fixed values of BIAS (with SAT_BIAS_SENDING=false), calibration of the magnetometer remains active. In this case the calibration values will be recalculated

#define USE_HWE_DEBUG_CONFIGURATION 1

// The following defines determine what things are output by default to the
// USB CDC. Don't ifdef code based on these, as it will prevent the ability to
// control at runtime by changing the boolean variables (e.g., with a debugger).


#define ENABLE_PRINTF	1	/* Unless you set this to 1, no printf will be enabled.
							 * If you enable this to 1, you must connect the USB to let the printf got out,
							 * otherwise the code will get stuck waiting to print.. */

#define DEBUG_GENERAL   1
#define DEBUG_RAW_DATA  0
#define DEBUG_SENSOR_FUSION_OUTPUT 0
#define DEBUG_NORDIC    0
#define OUTPUT_3D_CUBE  0
#define DEBUG_PRINT_LOG 0

static uint32_t cwati_fail_spi = 0, cwati_ok_spi = 0;

hub_err_t hub_err_status = hub_no_err;

uint32_t reset_rtc_diff = 0; /* Elapsed time before resetting RTC happens.*/

static volatile bool hw_init_done = false;

/* Heartbeat check for Nordic communication */
uint32_t last_nordic_comm;
const uint32_t max_nordic_quiet = 2000; /* in ms */
static uint32_t spi_crc_error = 0;
const uint32_t max_spi_crc_error = 1000;
static uint32_t reinit_nordic = 0;

#if HUB_22F
#define RED_LED_GPIO	10   /* K22F's GPIOA pin number for red LED */
#define GREEN_LED_GPIO	11  /* K22F's GPIOA pin number for green LED */
#define BLUE_LED_GPIO	12  /* K22F's GPIOA pin number for blue LED */
#else /* SAT_22F */
#define RED_LED_GPIO	12   /* K22F's GPIOA pin number for red LED */
#define GREEN_LED_GPIO	13   /* K22F's GPIOA pin number for green LED */
#define BLUE_LED_GPIO	14  /* K22F's GPIOA pin number for blue LED */
#endif
#define NORDIC_SWDIO_NRST	19	/* K22F's PTB 19 is for Nordic soft reset */

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

static uint8_t spi_tx_arr[SPI_BOOT_LENGTH] = {0};
static uint8_t spi_rx_arr[SPI_BOOT_LENGTH] = {0};

// variable to control bootloader mode
static uint8_t nordic_bl_mode = NORDIC_BOOTLOADER_MODE_INVALID;
static sat_to_nordic_boot_t spi_boot_tx;
static padded_nordic_to_sat_boot_t padded_spi_boot_rx;

static dspi_slave_state_t nrf51822_dspiSlaveState;


//static uint32_t timeAtBootInMillis = 0;
static int currSourceByte = 0;  // SPI data byte position for SPI slave
static gpio_output_pin_user_config_t ledRedPin;    // Red LED pin
static gpio_output_pin_user_config_t ledGreenPin;  // Green LED pin
static gpio_output_pin_user_config_t ledBluePin;   // Blue LED pin
#if PRODUCTION1
static gpio_output_pin_user_config_t nordicSwdioNrst;
#endif
//static gpio_output_pin_user_config_t nordicBootloaderSignal;
#define BOOT_SIGNAL_PIN 7
dspi_status_t nrf51822_init();
#if ENABLE_RTC
uint32_t k22f_get_rtc();
#endif

/* EM USB related stuff */
enum _mainState
{
	mainUSBstart,
	mainUSBstartWait,
	mainSensorInitialization,
	mainLoop
};
static enum _mainState mainState = mainUSBstart;

/* SPI to Nordic.  We act as the master */
enum _more_pins
{
	/* PRODUCTION1 or Eval Board */
	kNordicSsPin   = GPIO_MAKE_PIN(GPIOB_IDX,  10U),
	kNordicMosiPin = GPIO_MAKE_PIN(GPIOB_IDX,  17U),
};
static const gpio_output_pin_user_config_t nordic_ss_pin =
{
		.pinName = kNordicSsPin,
		.config =
		{
				.outputLogic = 0,
				.slewRate = kPortFastSlewRate,
				.driveStrength = kPortHighDriveStrength,
				.isOpenDrainEnabled = 0
		}
};

static const gpio_output_pin_user_config_t nordic_mosi_pin =
{
		.pinName = kNordicMosiPin,
		.config =
		{
				.outputLogic = 0,
				.slewRate = kPortFastSlewRate,
				.driveStrength = kPortHighDriveStrength,
				.isOpenDrainEnabled = 0
		}
};

static const dspi_device_t nordic_dspi_device =
{
	.bitsPerSec = 4000000L,
	.dataBusConfig.bitsPerFrame = 8,
	.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh,
	.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge,
	.dataBusConfig.direction = kDspiMsbFirst,
};
static dspi_master_state_t nordic_dspi_master_state;

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
	case whiteLed:
		GPIO_DRV_ClearPinOutput(ledGreenPin.pinName);
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
	OSA_TimeDelay(500); /* in ms */

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



int WAgiveIntoFromUint(uint32_t value){ //use command=1
	int result;
	uint32_t negativeCK=1000000000;

	if(value >= negativeCK)
		result = -((int)((value-negativeCK)));
	else
		result = (int)value;

	return result;
}



#define spi_time_size 1000
static uint32_t spi_time[spi_time_size] = {0};
static uint32_t spi_time_cnt = 0;


/* transmit_to_nordic
 * This is a function to transmit data between 22F and nordic
 * It uses the global spi_tx for transmit and spi_rx for receiving
 */
dspi_status_t transmit_to_nordic(void)
{
	dspi_status_t dspiResult;
	uint32_t temp_crc16, generated_crc16;
	uint32_t wordsTransfer;
	uint32_t time_start, time_now;

	time_start = k22f_get_rtc();

	/* Generate CRC16 */
	spi_boot_tx.crc16 = 0;
	spi_boot_tx.crc16 = crc16_compute((uint8_t*) &spi_boot_tx, sizeof(spi_boot_tx), 0);

	memcpy((uint8_t*)&spi_tx_arr, &spi_boot_tx, sizeof(spi_boot_tx));

	memset((uint8_t*)&spi_rx_arr, 0, sizeof(spi_rx_arr)); //cwati test
	dspiResult = DSPI_DRV_MasterTransferBlocking(knRF51822SpiInstance,
											& nordic_dspi_device,
											(uint8_t*)&spi_tx_arr,
											(uint8_t*)&spi_rx_arr,
											SPI_BOOT_LENGTH,
											1000);

	/* Give time for Nordic to process its main loop */
	OSA_TimeDelay(7);
	if (dspiResult != kStatus_DSPI_Success)
	{
		debug_printf("\r\n ERROR: Can not start SPI to Nordic \r\n");
		blinkLEDerror(redLed, 4);
	};

	memcpy(&padded_spi_boot_rx, &spi_rx_arr, sizeof(padded_spi_boot_rx));

	temp_crc16 = padded_spi_boot_rx.crc16;
	padded_spi_boot_rx.crc16 = 0;

	generated_crc16 = crc16_compute ((uint8_t*) &padded_spi_boot_rx, sizeof(padded_spi_boot_rx), 0);

	//cwati todo bug
	generated_crc16 |= 0x80;

	if (generated_crc16 != temp_crc16) {
		//debug_printf("\r\nWRONG CRC: 0x%x vs 0x%x\r\n", generated_crc16, temp_crc16);
		cwati_fail_spi++;
		if (cwati_fail_spi % 500 == 0) {
			dspiResult = nrf51822_init();
			if (dspiResult != kStatus_DSPI_Success)
		    {
				debug_printf("\r\n ERROR: Can not initialize DSPI master driver \r\n");
				blinkLEDerror(redLed, 4);
		    };
			//nrf51822_swdio_reset();
		}
		return kStatus_DSPI_Error;
	}
	cwati_ok_spi++;

	time_now = k22f_get_rtc();

	spi_time[spi_time_cnt++ % spi_time_size] = time_now - time_start;

	return kStatus_DSPI_Success;
}



/**************************************************************************
 * initiate_nordic_bl_upload
 * This will send a signal to Nordic to initiate HEX transfer for
 * Nordic bootloader
 **************************************************************************/
dspi_status_t initiate_nordic_bl_upload() {
	dspi_status_t dspiResult;

	// send command to Nordic to start the boot loader mode
	spi_boot_tx.boot_cmd = NORDIC_BOOTLOADER_START;

	for (uint8_t repeat = 0; repeat < 10; repeat++) {
		dspiResult = transmit_to_nordic();
		/* Give time for Nordic to process command and enter into its while main loop*/
		if (dspiResult == kStatus_DSPI_Success) {
			// only return "Success" if we receive an acknowledgement from nordic
			// that the BOOTLOADER mode is ready
			if (padded_spi_boot_rx.packet.reply_cmd == NORDIC_BOOTLOADER_READY) {
				return kStatus_DSPI_Success;
			}
		}
	}

    return kStatus_DSPI_Error;
}


uint32_t nordic_bl_mode_time[bl_num] = {0};
uint32_t nordic_bl_mode_cnt[bl_num] = {0};

/**************************************************************************
 * nordic_bootloader_run
 * This is the main code for Nordic's bootloader
 **************************************************************************/
void nordic_bootloader_run(void)
{
	if (nordic_bl_mode == NORDIC_BOOTLOADER_MODE_INVALID) {
		return;
	}
	dspi_status_t dspiResult;
	static uint32_t bl_num_lines = 0;
	intel_hex_t hex;
	intel_hex_bin_t hex_bin;
	static uint32_t bl_total_lines = 0;
	bool first_time = false;
	uint16_t last_crc16 = 0;
	int count = 0;
	int i;
	static uint32_t time_start, time_diff, time_now;
	static uint32_t cnt = 0;
	const uint16_t max_length = 350;
	char buf[max_length];

	uint32_t mem_address = 2000;
	int error_code = 0;
	while (1) {
		if (nordic_bl_mode == NORDIC_BOOTLOADER_MODE_START) {
			time_start = k22f_get_rtc();
			//debug_printf("Initializing NORDIC\n\r");
			dspiResult = initiate_nordic_bl_upload();
			if (dspiResult == kStatus_DSPI_Success) {
				// receive acknowledgement from nordic
				// so we can start the bootloader hex loading process
				nordic_bl_mode = NORDIC_BOOTLOADER_MODE_INIT;
				first_time = true;
				//debug_printf("RECEIVED ACKNOWLEDGEMENT FROM NORDIC\n\r");
				// Check Nordic's bootloader version to make sure it's major version
				// is the same
				if (padded_spi_boot_rx.packet.major_version != BOOTLOADER_MAJOR_VERSION) {
					error_code = 1;	// invalid version
					break;
				}
			} else {
				if (cnt++ % 10 == 0) {
					nrf51822_swdio_reset();
				}
			}

			time_diff = k22f_get_rtc() - time_start;
			nordic_bl_mode_time[bl_start] += time_diff;
			nordic_bl_mode_cnt[bl_start]++;

		} else if (nordic_bl_mode == NORDIC_BOOTLOADER_MODE_INIT) {
			// get the number of lines
			if (first_time) {
				debug_printf("abracadabra1\n\r"); /* Need some string first ... */
				debug_printf("abracadabra2\n\r"); /* Need some string first ... */
				debug_printf("abracadabra3\n\r"); /* Need some string first ... */
				debug_printf("abracadabra4\n\r"); /* Need some string first ... */
				debug_printf("abracadabra5\n\r"); /* Need some string first ... */
				debug_printf("abracadabra6\n\r"); /* Need some string first ... */
				debug_printf("abracadabra7\n\r"); /* Need some string first ... */
				debug_printf("abracadabra8\n\r"); /* Need some string first ... */
				debug_printf("abracadabra9\n\r"); /* Need some string first ... */
				debug_printf("abracadabra10\n\r"); /* Need some string first ... */
				debug_printf("Turingsense RF Firmware Upgrade Version %u.%u\n\r",BOOTLOADER_MAJOR_VERSION, BOOTLOADER_MINOR_VERSION);
				bl_total_lines = init_hex_from_usb();
#if DEBUG_BL
				debug_printf("NUM LINE %d\n\r", bl_total_lines);
#endif /* DEBUG_BL */
				first_time = false;
			}
			time_start = k22f_get_rtc();

			spi_boot_tx.boot_cmd = NORDIC_BOOTLOADER_ERASE;
			spi_boot_tx.line_num = bl_total_lines;
			dspiResult = transmit_to_nordic();
			if (dspiResult == kStatus_DSPI_Success
					&& padded_spi_boot_rx.packet.reply_cmd == NORDIC_BOOTLOADER_ACK) {
				nordic_bl_mode = NORDIC_BOOTLOADER_MODE_ERASE;
				bl_num_lines = 0;
			}
			time_now = k22f_get_rtc();
			time_diff = time_now - time_start;
			nordic_bl_mode_time[bl_init2] += time_diff;
			nordic_bl_mode_cnt[bl_init2]++;

		} else if (nordic_bl_mode == NORDIC_BOOTLOADER_MODE_ERASE
					|| nordic_bl_mode == NORDIC_BOOTLOADER_MODE_LOAD) {
			time_start = k22f_get_rtc();

			spi_boot_tx.boot_cmd = NORDIC_BOOTLOADER_ACK;
			dspiResult = transmit_to_nordic();
			//if (dspiResult == kStatus_DSPI_Success) {
				//debug_printf("Receive from nordic (after erase/load): %d %x %x\n\r", spi_boot_rx.reply_cmd, spi_boot_rx.back[0], spi_boot_rx.back[1]);
				//debug_printf("  BACK: 0x%x 0x%x 0x%x 0x%x\n\r", spi_boot_rx.back[0], spi_boot_rx.back[1], spi_boot_rx.back[2], spi_boot_rx.back[3]);
			//}
			if (dspiResult == kStatus_DSPI_Success
					&& padded_spi_boot_rx.packet.reply_cmd == NORDIC_BOOTLOADER_READY) {
				nordic_bl_mode = NORDIC_BOOTLOADER_MODE_READY;
				//debug_printf("GET READY FROM NORDIC\n\r");
				first_time = true;
			}
			time_diff = k22f_get_rtc() - time_start;
			nordic_bl_mode_time[bl_erase_load] += time_diff;
			nordic_bl_mode_cnt[bl_erase_load]++;

		} else if (nordic_bl_mode == NORDIC_BOOTLOADER_MODE_READY) {
			//OSA_TimeDelay(10); /* ms */
			if (first_time) {
				time_start = k22f_get_rtc();
#if DEBUG_BL
				debug_printf("%d of %d\n\r", bl_num_lines, bl_total_lines);
#endif
				first_time = false;
				spi_boot_tx.boot_cmd = NORDIC_BOOTLOADER_TX;
				spi_boot_tx.line_num = bl_num_lines;
				if (bl_num_lines < bl_total_lines) {
					for (i = 0; i < NUM_HEX_PER_SPI_PACKET; i++) {
						if (bl_num_lines >= bl_total_lines) {
							memset(&spi_boot_tx.hex[i], 0, sizeof(hex_record_t));
						} else {
							uint16_t size = get_hex_string_from_usb(buf, max_length);
							uint16_t count = convert_string_to_hex_record(buf, size, &(spi_boot_tx.hex[i]));
							bl_num_lines += count;
							//debug_printf("SUCCESSFULLY RECEIVE %d HEX\n\r", count);
							i += count - 1;
							continue;
/*
							bl_num_lines++;
							get_hex_from_usb(&hex);

							if (convert_intel_hex_to_hex_record(hex, &spi_boot_tx.hex[i])) {
								//print_hex_record2(spi_boot_tx.hex[i]);
							} else {
#if DEBUG_BL
								debug_printf("FAIL TO CONVERT\n\r");
								hex.data[hex.size] = '\0';
								debug_printf("GET: %d[%s]\n\r", hex.size, hex.data);
#endif
							}
*/
						}
					}
				} else {
					nordic_bl_mode = NORDIC_BOOTLOADER_MODE_STOP;
					end_hex_from_usb();
					spi_boot_tx.boot_cmd = NORDIC_BOOTLOADER_DONE;
					count = 100;
				}
				spi_boot_tx.crc16 = 0;
				last_crc16 = spi_boot_tx.crc16 = crc16_compute((uint8_t*) &spi_boot_tx, sizeof(spi_boot_tx), 0);
				time_diff = k22f_get_rtc() - time_start;
				nordic_bl_mode_time[bl_ready1] += time_diff;
				nordic_bl_mode_cnt[bl_ready1]++;
			}
			time_start = k22f_get_rtc();
			dspiResult = transmit_to_nordic();
			if (dspiResult == kStatus_DSPI_Success && nordic_bl_mode == NORDIC_BOOTLOADER_MODE_READY
					&& padded_spi_boot_rx.packet.reply_cmd == NORDIC_BOOTLOADER_ACK
					&& padded_spi_boot_rx.packet.last_crc16_recvd == spi_boot_tx.crc16) {
						/*&& spi_boot_rx.line_num == bl_num_lines*/
				nordic_bl_mode = NORDIC_BOOTLOADER_MODE_LOAD;
				//debug_printf("GOT ACKNOWLEDGEMENT\n\r");
			}
			time_diff = k22f_get_rtc() - time_start;
			nordic_bl_mode_time[bl_ready2] += time_diff;
			nordic_bl_mode_cnt[bl_ready2]++;

		} else if (nordic_bl_mode == NORDIC_BOOTLOADER_MODE_STOP) {
			time_start = k22f_get_rtc();

			// send the stop signal multiple times to make sure Nordic gets it
			count--;
			if (count > 0) {
				spi_boot_tx.boot_cmd = NORDIC_BOOTLOADER_DONE;
				spi_boot_tx.crc16 = 0;
				spi_boot_tx.crc16 = crc16_compute((uint8_t*) &spi_boot_tx, sizeof(spi_boot_tx), 0);
				dspiResult = transmit_to_nordic();
			} else {
				nordic_bl_mode = NORDIC_BOOTLOADER_MODE_FINISH;
			}
			time_diff = k22f_get_rtc() - time_start;
			nordic_bl_mode_time[bl_stop] += time_diff;
			nordic_bl_mode_cnt[bl_stop]++;
		} else if (nordic_bl_mode == NORDIC_BOOTLOADER_MODE_READ) {
			time_start = k22f_get_rtc();

			spi_boot_tx.boot_cmd = 0;
			spi_boot_tx.line_num = 0;
			dspiResult = transmit_to_nordic();
//			OSA_TimeDelay(5);
#if DEBUG_BL
			if (dspiResult == kStatus_DSPI_Success) {
				debug_printf("Receive from nordic: %d %x %x %x %x %d\n\r",
						padded_spi_boot_rx.packet.reply_cmd, padded_spi_boot_rx.packet.back[0], \
						padded_spi_boot_rx.packet.back[1], padded_spi_boot_rx.packet.back[2], \
						padded_spi_boot_rx.packet.back[3], padded_spi_boot_rx.packet.line_num);
			}
#endif
			time_diff = k22f_get_rtc() - time_start;
			nordic_bl_mode_time[bl_read] += time_diff;
#if 0
		} else if (0 && nordic_bl_mode == NORDIC_BOOTLOADER_MODE_READ) {
			if (first_time) {
				spi_boot_tx.boot_cmd = NORDIC_BOOTLOADER_READ;
				spi_boot_tx.line_num = mem_address;
				mem_address += 16;
				first_time = false;
				if (mem_address > 6000) {
					nordic_bl_mode = NORDIC_BOOTLOADER_MODE_INVALID;
				}
			}
			dspiResult = transmit_to_nordic();
			OSA_TimeDelay(20);
			if (dspiResult == kStatus_DSPI_Success && spi_boot_rx.reply_cmd == NORDIC_BOOTLOADER_ACK) {
				debug_printf("Receive from nordic: %d %x %x %x %x %d\n\r",
						padded_spi_boot_rx.packet.reply_cmd, padded_spi_boot_rx.packet.back[0], padded_spi_boot_rx.packet.back[1],
						padded_spi_boot_rx.packet.back[2], padded_spi_boot_rx.packet.back[3], padded_spi_boot_rx.packet.line_num);
				spi_boot_tx.boot_cmd = NORDIC_BOOTLOADER_ACK;
			} else if (dspiResult == kStatus_DSPI_Success && padded_spi_boot_rx.packet.reply_cmd == NORDIC_BOOTLOADER_READY) {
				first_time = true;
			}
#endif
		} else if (nordic_bl_mode == NORDIC_BOOTLOADER_MODE_FINISH) {
		    NVIC_SystemReset();
		}
		//OSA_TimeDelay(5); /* ms */
	}
	// we will come to here if there is any error
	// handling error code
	while(1) {
		switch (error_code) {
		case 1:
			debug_printf("\n\rERROR: Incompatible Nordic's bootloader version\n\r");
			break;
		default:
			debug_printf("\n\rERROR: Unknown error\n\r");
		}
		OSA_TimeDelay(5);
	}
}

/*******************************************************************************
 * Initialize Nordic nRF51822
 * For Nordic tunneling code, 22F is the SPI master.
 ******************************************************************************/
dspi_status_t nrf51822_init()
{
	dspi_status_t status;
	uint32_t calculatedBaudRate;
	uint32_t calculatedPcsToSck, calculatedLastSckToPcs, calculatedAfterTransfer;

	static const dspi_master_user_config_t nrf51822_dspiMasterUserConfig =
	{
		.whichCtar = kDspiCtar0,
		.whichPcs = kDspiPcs0,
		.pcsPolarity = kDspiPcs_ActiveLow,
		.isSckContinuous = false,
		.isChipSelectContinuous = true,
	};

	GPIO_DRV_OutputPinInit(& nordic_ss_pin);
	GPIO_DRV_OutputPinInit(& nordic_mosi_pin);

	configure_spi_pins(knRF51822SpiInstance);

	status = DSPI_DRV_MasterInit(knRF51822SpiInstance, & nordic_dspi_master_state, &nrf51822_dspiMasterUserConfig);
	if (status != kStatus_DSPI_Success)
		return status;

    status = DSPI_DRV_MasterConfigureBus(knRF51822SpiInstance, & nordic_dspi_device, & calculatedBaudRate);
	if (status != kStatus_DSPI_Success)
		return status;

    status = DSPI_DRV_MasterSetDelay(knRF51822SpiInstance,
    								 kDspiPcsToSck,
    								 500, // delayInNanoSec
    								 & calculatedPcsToSck);
	if (status != kStatus_DSPI_Success)
		return status;

    status = DSPI_DRV_MasterSetDelay(knRF51822SpiInstance,
    								 kDspiLastSckToPcs,
    								 500, // delayInNanoSec
    								 & calculatedLastSckToPcs);
	if (status != kStatus_DSPI_Success)
		return status;

	status = DSPI_DRV_MasterSetDelay(knRF51822SpiInstance,
    								 kDspiAfterTransfer,
    								 500, // delayInNanoSec
    								 & calculatedAfterTransfer);
	if (status != kStatus_DSPI_Success)
		return status;

	return status;
}

extern uint32_t g_xtal0ClkFreq;  /* For USB purposes.  Do not remove me if you want to use USB! */
uint32_t tmp_nordic;


//cwati test
void PORTC_IRQHandler(void)
{
	//interrupt from SPI0/invensense
	GPIO_DRV_ClearPinIntFlag(GPIO_MAKE_PIN(GPIOC_IDX,  8U));
}

void SPI0_IRQHandler(void)	/* sensor module uses SPI1 for invensense */
{
    DSPI_DRV_IRQHandler(0); /* SPI0 */
}

/*******************************************************************************
 * Main routine
 ******************************************************************************/
int main(void) {
	uint32_t current_ts10, time_now;
	bool is_sleeping = false, retbool;
    dspi_status_t dspiResult;
    uint32_t wordsTransfer = 0;
    static uint16_t			last_crc16_sent = 0;
    uint16_t				generated_crc16, temp_crc16;

	g_xtal0ClkFreq = 8000000U;   /* For USB purposes.  Do not remove me if you want to use USB! */

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
	k22f_set_rtc(0);
	uint32_t timetmp = k22f_get_rtc();

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
//
//	// Set BOOTLOADER signal pin (PTB7)
//	nordicBootloaderSignal.config.outputLogic = 0;
//	nordicBootloaderSignal.config.slewRate = kPortSlowSlewRate;
//	nordicBootloaderSignal.config.driveStrength = kPortHighDriveStrength;
//	nordicBootloaderSignal.pinName = GPIO_MAKE_PIN(GPIOB_IDX, BOOT_SIGNAL_PIN);
//	GPIO_DRV_OutputPinInit(&nordicBootloaderSignal);

	/* Initializing satellite id */

	// Initialize nRF51822 (configure SPI0 slave, data structures, etc.)
	//debug_printf("\r\nInitializing Nordic nRF51822\r\n");

	turnOnLED(blueLed);

	dspiResult = nrf51822_init();
	if (dspiResult != kStatus_DSPI_Success)
    {
		debug_printf("\r\n ERROR: Can not initialize DSPI master driver \r\n");
		blinkLEDerror(redLed, 4);
    };

	//debug_printf("GOT HERE \n\r");
//cwati todo
//	// send signal to nordic to start bootloader mode
//	int i;
//	GPIO_DRV_ClearPinOutput(nordicBootloaderSignal.pinName);
//	OSA_TimeDelay(10);
//	for (i = 0; i < 100; i++) {
//		GPIO_DRV_TogglePinOutput(nordicBootloaderSignal.pinName);
//		OSA_TimeDelay(10);
//	}

	// Main loop

	nordic_bl_mode = NORDIC_BOOTLOADER_MODE_START;
	//nordic_bl_mode = NORDIC_BOOTLOADER_MODE_READ;

//	nrf51822_swdio_reset();

	while (1) {
		nordic_bootloader_run();
//		OSA_TimeDelay(5); /* ms */
	} /* End of while(1) */

	return 0;  // will never reach this

} // end main()

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

