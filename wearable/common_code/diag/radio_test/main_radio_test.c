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
#include "ts_rtc.h"

/* USB */
void VirtualCom_Deinit(); //virtual_com.h
#include "fsl_debug_console.h"
#include "crc16.h"

/*******************************************************************************
 * TuringSense Include Files
 ******************************************************************************/
#include "main_radio_test.h"
#include "radio_test_common_types.h"

#define NORDIC_CALLBACK_TODO 0

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
static uint16_t last_crc16_sent = UINT16_MAX;

uint8_t mode_          = UINT8_MAX;
uint8_t txpower_       = UINT8_MAX;
uint8_t channel_start_ = UINT8_MAX;
uint8_t channel_end_   = UINT8_MAX;
uint8_t delayms_       = UINT8_MAX;

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

static uint8_t spi_tx_arr[SPI_COMM_LENGTH] = {0};
static uint8_t spi_rx_arr[SPI_COMM_LENGTH] = {0};
static mk22f_to_nordic_radio_test_t spi_tx;
static nordic_to_mk22f_radio_test_t spi_rx;

static dspi_slave_state_t nrf51822_dspiSlaveState;

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
	.bitsPerSec = 250000L,
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
#define printf(...) cond_printf(debug_output, __VA_ARGS__)
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
	OSA_TimeDelay(200); /* in ms */

	GPIO_DRV_SetPinOutput(nordicSwdioNrst.pinName);

	turnOnLED(offLed);

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

/* transmit_to_nordic
 * This is a function to transmit data between 22F and nordic
 * It uses the global spi_tx for transmit and spi_rx for receiving
 */
dspi_status_t transmit_to_nordic(void)
{
	dspi_status_t dspiResult;
	uint32_t temp_crc16, generated_crc16;
	uint32_t wordsTransfer;

	/* Generate CRC16 */
	spi_tx.crc16 = 0;
	spi_tx.crc16 = crc16_compute((uint8_t*) &spi_tx, sizeof(spi_tx), 0);

	memcpy((uint8_t*)&spi_tx_arr, &spi_tx, sizeof(spi_tx));

	last_crc16_sent = spi_tx.crc16;

	memset((uint8_t*)&spi_rx_arr, 0, sizeof(spi_rx_arr)); //cwati test
	dspiResult = DSPI_DRV_MasterTransferBlocking(knRF51822SpiInstance,
											& nordic_dspi_device,
											(uint8_t*)&spi_tx_arr,
											(uint8_t*)&spi_rx_arr,
											SPI_COMM_LENGTH,
											1000);

	/* Give time for Nordic to process its main loop */
	OSA_TimeDelay(10);

	if (dspiResult != kStatus_DSPI_Success)
	{
		debug_printf("\r ERROR: Can not start SPI to Nordic \r");
		blinkLEDerror(redLed, 4);
	};

	memcpy(&spi_rx, &spi_rx_arr, sizeof(spi_rx));

	temp_crc16 = spi_rx.crc16;
	spi_rx.crc16 = 0;

	generated_crc16 = crc16_compute ((uint8_t*) &spi_rx, sizeof(spi_rx), 0);

	if (generated_crc16 != temp_crc16) {
		//debug_printf("\rWRONG CRC: 0x%x vs 0x%x\r", generated_crc16, temp_crc16);
		cwati_fail_spi++;
		if (cwati_fail_spi % 500 == 0) {
			dspiResult = nrf51822_init();
			if (dspiResult != kStatus_DSPI_Success)
		    {
				debug_printf("\r ERROR: Can not initialize DSPI master driver \r");
				blinkLEDerror(redLed, 4);
		    };
		}
		return kStatus_DSPI_Error;
	}
	cwati_ok_spi++;

    return kStatus_DSPI_Success;
}


void spiSendUntilAcked() {
	dspi_status_t 		dspiResult, initResult;
	static uint32_t	  	failed_to_get_ack = 0, max_wait_to_get_ack = 0;;
	const uint32_t		max_fail = 500;

	while (1) {
		dspiResult = transmit_to_nordic();

		if (dspiResult == kStatus_DSPI_Success) {
			debug_printf("Command has been SUCCESSFULLY transmitted to Nordic!\r");
			debug_printf("[SPI_TRANSM:OK]\r");
			mode_          = spi_rx.mode;
			txpower_       = spi_rx.tx_power;
			channel_start_ = spi_rx.channel_start;
			channel_end_   = spi_rx.channel_end;
			delayms_       = spi_rx.delayms;
			break;
		} else {
			failed_to_get_ack++;
		}

		if (failed_to_get_ack > max_wait_to_get_ack) {
			max_wait_to_get_ack = failed_to_get_ack;
		}

		if (failed_to_get_ack == max_fail) {
			initResult = nrf51822_init();

			if (initResult != kStatus_DSPI_Success) {
				//blinkLEDerror(redLed, 2);
				debug_printf("[SPI_TRANSM:FAILED]\r");
			} else {
				failed_to_get_ack = 0;
			}
		}
	} /* end of while(1) */
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

static void print_parameters() {
    printf("Parameters:\r");

    switch(mode_)
    {
        case 0:
            printf("Data rate...........: 250 Kbit/s\r");
            break;

        case 1:
            printf("Data rate...........: 1 Mbit/s\r");
            break;

        case 2:
            printf("Data rate...........: 2 Mbit/s\r");
            break;
        default:
            printf("Data rate UNKNOWN...: 0x%x Mbit/s\r", mode_);
            break;
    }

    switch(txpower_)
    {
    	case 0:
            printf("TX Power............: +4 dBm\r");
            break;

        case 1:
            printf("TX Power............: 0 dBm\r");
            break;

        case 2:
            printf("TX Power............: -4 dBm\r");
            break;

        case 3:
            printf("TX Power............: -8 dBm\r");
            break;

        case 4:
            printf("TX Power............: -12 dBm\r");
            break;

        case 5:
            printf("TX Power............: -16 dBm\r");
            break;

        case 6:
            printf("TX Power............: -20 dBm\r");
            break;

        case 7:
            printf("TX Power............: -30 dBm\r");
            break;

        default:
            printf("TX Power UNKNOWN....: 0x%x dBm\r", txpower_);
            break;

    }
    printf("Start Channel.......: %d\r",channel_start_);
    printf("End Channel.........: %d\r",channel_end_);
    printf("Time on each channel: %d ms\r",delayms_);
}

static void print_help() {
	debug_printf("M|	Usage:\r");
	debug_printf("M|	a: Enter start channel for sweep/channel for constant carrier\r");
	debug_printf("M|	b: Enter end channel for sweep\r");
	debug_printf("M|	c: Start TX carrier\r");
	debug_printf("M|	d: Enter time on each channel (1ms-99ms)\r");
	debug_printf("M|	e: Cancel sweep/carrier\r");
	debug_printf("M|	m: Enter data rate\r");
	debug_printf("M|	o: Start modulated TX carrier\r");
	debug_printf("M|	p: Enter output power\r");
	debug_printf("M|	s: Print current delay, channels and so on\r");
	debug_printf("M|	r: Start RX sweep\r");
	debug_printf("M|	t: Start TX sweep\r");
	debug_printf("M|	x: Start RX carrier\r");
	debug_printf("M|	5: Print FW version\r");
	debug_printf("M|	7: Print SAT ID\r");
	debug_printf("E|\r");
}

uint32_t calculateSATID() {
            //uint32_t MOD=1073741824;
            //return SIM_UIDH%MOD+SIM_UIDMH%MOD+SIM_UIDML%MOD+SIM_UIDL%MOD;
	return (SIM_UIDL & 0x003F) | (((SIM_UIDMH >> 8) & 0x003F) << 6) | (((SIM_UIDH >> 16) & 0x003F) << 12);
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
    uint8_t					c;
    uint32_t				test;
    char					command;

	g_xtal0ClkFreq = 8000000U;   /* For USB purposes.  Do not remove me if you want to use USB! */

	VirtualCom_Deinit();

	// Initialize K22F
	hardware_init();
	if (OSA_Init() != kStatus_OSA_Success) {
		debug_printf("error initializing OSA\r");
		blinkLEDerror(redLed, 1);
	}
	configure_uart_pins(BOARD_DEBUG_UART_INSTANCE);
	dbg_uart_init();
	k22f_enable_rtc();

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

	nrf51822_swdio_reset();
	dspiResult = nrf51822_init();
	if (dspiResult != kStatus_DSPI_Success)
    {
		debug_printf("\r ERROR: Can not initialize DSPI master driver \r");
		blinkLEDerror(redLed, 4);
    };

	spi_tx.command = 'z';

	// Done with all initializations, turn on GREEN LED, entering Main loop
	turnOnLED(greenLed);
	debug_printf("***********************************************\r");
	debug_printf("Turingsense Radio Test Utility Version %s\r", VERSION_RELEASE);
#if HUB_22F
	debug_printf("[HUBRADIOTEST=" VERSION_RELEASE "]\r");
	debug_printf("[HUBIDX=%d]\r", calculateSATID());
#else
	debug_printf("[SATRADIOTEST=" VERSION_RELEASE "]\r");
	debug_printf("[SATIDX=%d]\r", calculateSATID());
#endif
	debug_printf("***********************************************\r");

	while (1) {
		print_help();

		debug_scanf("%c", &command);
		printf("%c\n", command);

		switch (command)
		{
			case 'a':
				spi_tx.command = 'a';
				while (true)
				{
					printf("S|	Enter start channel (two decimal digits, 00 to 80):\r");
					printf("E|\r");
					debug_scanf("%d",&c);
					printf("%d\n", c);
					if ((c <= 80)&&(c >= 0))
					{
						spi_tx.start_channel = c;
						break;
					}

					printf("S|	Channel must be between 0 and 80\r");
					printf("E|\r");
				}
				break;

			case 'b':
				spi_tx.command = 'b';
				while (true)
				{
					printf("S|	Enter end channel (two decimal digits, 00 to 80):\r");
					printf("E|\r");
					debug_scanf("%d",&c);
					printf("%d\n", c);
					if ((c <= 80)&&(c >= 0))
					{
						spi_tx.end_channel = c;
						break;
					}
					printf("S|	Channel must be between 0 and 80\r");
					printf("E|\r");
				}
				break;

			case 'c':
				spi_tx.command = 'c';
				break;

			case 'd':
				spi_tx.command = 'd';
				while (true)
				{
					printf("S|	Enter delay in ms (two decimal digits, 01 to 99):\r");
					printf("E|\r");
					debug_scanf("%d",&c);
					printf("%d\n", c);
					if ((c > 0) && (c < 100))
					{
						spi_tx.delayms = c;
						break;
					}
					printf("S|	Delay must be between 1 and 99\r");
					printf("E|\r");
				}
				break;

			case 'e':
				spi_tx.command = 'e';
				break;

			case 'm':
				spi_tx.command = 'm';
				do {
					printf("S|	Enter data rate ('0'=250 Kbit/s, '1'=1 Mbit/s and '2'=2 Mbit/s):\r");
					printf("E|\r");
					debug_scanf("%u",&c);
					printf("%u\n", c);
					if ((c >= 0) && (c <= 2))
					{
						spi_tx.mode = c;
						break;
					}
				} while (1);
				break;

			case 'o':
				spi_tx.command = 'o';
				printf("TX modulated carrier\r");
				break;

			case 'p':
			    spi_tx.command = 'p';
			    do {
			    	printf("S|	Enter output power (0=+4 dBm, 1=0 dBm,...,7=-30 dBm):\r");
			    	printf("E|\r");
			        debug_scanf("%u",&c);
					printf("%u\n", c);
			        if ((c >= 0) && (c <= 7))
			        {
			            spi_tx.power = c;
			            break;
			        }
			    } while (1);
				break;

			case 'r':
				spi_tx.command = 'r';
				printf("RX Sweep\r");
				break;

			case 's':
				spiSendUntilAcked();
				print_parameters();
				continue;

			case 't':
				spi_tx.command = 't';
				printf("TX Sweep\r");
				break;

			case 'x':
				spi_tx.command = 'h';
				printf("RX constant carrier\r");
				break;
			case '5':
				// Print version
#if HUB_22F
				printf("[HUBRADIOTEST=" VERSION_RELEASE "]\r");
#else
				printf("[SATRADIOTEST=" VERSION_RELEASE "]\r");
#endif
				break;
			case '7':
#if HUB_22F
				printf("[HUBIDX=%d]\r", calculateSATID());
#else
				printf("[SATIDX=%d]\r", calculateSATID());
#endif
				break;
			case 'h':
				// Fall through.

			default:
				print_help();
				break;
		}

		spiSendUntilAcked();
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

