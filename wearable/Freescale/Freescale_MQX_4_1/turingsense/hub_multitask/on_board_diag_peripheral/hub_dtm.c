/*
 * Copyright TuringSense, Inc © 2016
 * hub_dtm.c
 *
 *  Created on: June 23, 2016
 *      Author: cwati
 *
 *
 */

#include <mqx.h>
#include <mutex.h>

#include "hub_main_loop_dtm.h"
#include "hub_to_nordic_dtm.h"
#include "main.h"
#include "throughput.h"
#include "wmiconfig_ts.h"
#include "virtual_com.h"
#include "common_types.h"
#include "dtm_common_types.h" /* Turing Sense specific */



/*******************************************************************************
 * Global Variables
 ******************************************************************************/
static hub_to_nordic_dtm_t spi_tx;
static nordic_to_hub_dtm_t spi_rx;
/*************************** GLOBAL ************************/


const uint8_t enable_stats = 1;

cloud_to_hub_t cloud_to_hub;
hub_to_nordic_t hub_to_nordic;
nordic_to_hub_t nordic_to_hub;
hub_to_cloud_t hub_to_cloud;

uint32_t current_state;
cbuf_t cbuf[MAX_SENSORS];

#define TMP_NUM_OF_SAT			5
uint32_t cur_num_of_sat = TMP_NUM_OF_SAT;

uint32_t cur_sat_ids[] = {4,10,12,16,21};

uint32_t cur_hub_id = 0xabcd1;

/***********************************************************/
uint32_t hub_active (uint32_t c) {
	return 1;//(c & WIFI_START);
}
uint32_t hub_not_active (uint32_t c) {
	return 1;//(!(c & WIFI_START));
}
uint32_t hub_can_send (uint32_t c) {
	return 1;//(!(c & WIFI_WAIT));
}
uint32_t hub_cannot_send (uint32_t c) {
	return 1;//(c & WIFI_WAIT);
}
void set_hub_can_send(uint32_t* c) {
//	*c &= ~WIFI_WAIT;
//	hub_to_nordic.command &= ~NORDIC_WAIT;
}
void set_hub_cannot_send(uint32_t* c)	{
//	*c |= WIFI_WAIT;
//	hub_to_nordic.command |= NORDIC_WAIT;
}

void set_hub_active(uint32_t* c) {
//	*c |= WIFI_START;
//	hub_to_nordic.command |= NORDIC_START;
}

void set_hub_not_active(uint32_t* c) {
//	*c &= ~WIFI_START;
//	hub_to_nordic.command &= ~NORDIC_START;
}




/* Function: clr_atheros_GPIO
 * Parameters: none
 *
 * This function will turn on Green LED
 * */
void clr_ath_GPIO(MQX_FILE_PTR pins_fd) {
	const GPIO_PIN_STRUCT pins[] = {
	GPIO_GREEN_LED,
	GPIO_LIST_END };

	pins_fd = fopen("gpio:output", (char *) &pins);

	if (NULL == pins_fd) {
		printf("ERROR: Failed to open GPIO char device to turn on green LED!\n");
	}

	if (ioctl(pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
//		printf("OK write log0\n");
	}
	fclose(pins_fd);
}

/* Function: clr_atheros_GPIO
 * Parameters: none
 *
 * This function will clear GPIO pin.
 * Will turn off Green LED.
 */
void set_ath_GPIO(MQX_FILE_PTR pins_fd) {
	const GPIO_PIN_STRUCT pins[] = {
	GPIO_GREEN_LED,
	GPIO_LIST_END };

	pins_fd = fopen("gpio:output", (char *) &pins);

	if (NULL == pins_fd) {
		printf(
				"ERROR: Failed to open GPIO char device to turn off green LED!\n");
	}

	if (ioctl(pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
//		printf("OK write log1\n");
	}
	fclose(pins_fd);
}

/* Function: assert_nordic_reset
 * Parameters: none
 *
 * This function will assert nordic soft reset pin.
 *
 */
void assert_nordic_reset() {
  MQX_FILE_PTR pins_fd;
  const GPIO_PIN_STRUCT pins[] = {NORDIC_RST, GPIO_LIST_END };
  uint8_t rgb[3] = {0};
  
  pins_fd = fopen("gpio:output", (char *) &pins);
  if (NULL == pins_fd) {
#if EVAL_BOARD
    printf("ERROR: Failed to open GPIO char device to reset Nordic!\n");
#endif
  }

  if (ioctl(pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
  //		printf("OK write log1\n");
  }
  /* Turn off the LEDs */
  rgb[redLed] = ts_get_red_LED();
  rgb[greenLed] = ts_get_green_LED();
  rgb[blueLed] = ts_get_blue_LED();
  ts_turn_on_LED(offLed);

  app_time_delay(100); /* Wait in millisecond.  Minimum 100-150ms. */

  if (ioctl(pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
  //		printf("OK write log1\n");
  }
  /* Turn on the LEDs.  Why do we flip the value you ask?
   * Because ts_get_<color>_LED returns the GPIO pin value, and
   * ts_turnOnOneLED accepts "on" or "off" state according to human eye.
   * These two are opposite of each other, ie if GPIO pin value is 0, then it turns on. */
  ts_turnOnOneLED(redLed,!rgb[redLed]);
  ts_turnOnOneLED(greenLed,!rgb[greenLed]);
  ts_turnOnOneLED(blueLed,!rgb[blueLed]);

  fclose(pins_fd);

}

/* Returns
 * 1	: if any cbuf has fill
 * 0	: if no cbuf has fill.  All cbufs are empty.
 */
uint8_t cbufs_not_empty(void) {
	uint8_t has_fill = 0;

	for (uint8_t i = 0; i < cur_num_of_sat; i++) {
		if (!cbufIsEmpty(&(cbuf[i]))) {
			has_fill = 1;
			return 1;
		}
	}

	return 0;
}

/* Returns
 * 1	: if all cbufs have entry
 * 0	: if any cbuf doesn't have entry
 */
uint8_t cbufs_all_has_entry(void) {
	uint8_t has_entry = 1;

	for (uint8_t i = 0; i < cur_num_of_sat; i++) {
		if (cbufIsEmpty(&(cbuf[i]))) {
			has_entry = 0;
			return 0;
		}
	}

	return 1;
}

/* Returns:
 * 	1: if all cbuf has at least space to read one more nordic spi
 * 	0: if any cbuf is still kind of empty
 */
static uint32_t all_cbuf_fill_pct(void) {
	uint32_t 		fill_pct, max_fill;
	uint8_t			empty_cbuf = 0;

	for (uint8_t i = 0; i < cur_num_of_sat; i++) {
		if (cbufIsEmpty(&cbuf[i])) {
			return 0;
		}
		fill_pct = cbufNum(&(cbuf[i]));
                max_fill = CB_SIZE - MAX_SENSOR_RECORDS_PER_PACKET;
		if (fill_pct < max_fill) {
			return 0;
		}
	}

	return 1;
}

/* Returns:
 * 	cbuf with the highest fill, in percentage, rounded to integer.
 */
uint32_t max_cbuf_fill_pct() {
	uint32_t 		max_pct = 0;
	uint32_t 		fill_pct;

	for (uint8_t i = 0; i < cur_num_of_sat; i++) {
		if (cbufIsEmpty(&cbuf[i])) {
			continue;
		}
		fill_pct = cbufNum(&(cbuf[i])) * 100 / CB_SIZE;
		if (fill_pct > max_pct) {
			max_pct = fill_pct;
		}
	}
	return (max_pct);
}

/*TASK*-------------------------------------------------------------------
 *
 * Task Name : hub_quit_loop
 * Comments  : Called when user types in "quittx"
 *
 *END*----------------------------------------------------------------------*/
static A_INT32 hub_quit_loop(void) {
	deinit_nordic();

	return A_OK;
}

//***************  DTM FUNCTIONS *********************************//
static parameters_t parameters;

//calculate HUB_ID
static uint32_t calculateSATID() {
            //uint32_t MOD=1073741824;
            //return SIM_UIDH%MOD+SIM_UIDMH%MOD+SIM_UIDML%MOD+SIM_UIDL%MOD;
	return (SIM_UIDL & 0x003F) | (((SIM_UIDMH >> 8) & 0x003F) << 6) | (((SIM_UIDH >> 16) & 0x003F) << 12);
}



/*TASK*-------------------------------------------------------------------
 *
 * Task Name : hub_main_loop
 *
 *END*----------------------------------------------------------------------*/
void hub_dtm_main_loop(uint32_t dummy) {
	A_INT32 		return_code, timeout;
	A_UINT8 		i;
	MQX_FILE_PTR 	pins_fd;
	GPIO_PIN_STRUCT pins[2];
	err_t			ret;
	uint32_t		ret32;
        
        uint16_t		user_input;
        static bool             first_time = true;
        
	_int_install_unexpected_isr();

	assert_nordic_reset();
	init_nordic();
        TestApp_Init(); /* Virtual Com */  

	ts_init_LED();
	ts_turn_on_LED(greenLed);
        
        debug_printf("\r\n***********************************************\r\n");
        debug_printf("Turingsense Direct Test Mode tool (c) 2016\r\n");
	debug_printf("[HUBDTM="VERSION_RELEASE"]\r\n");
	debug_printf("[HUBIDX=%d]\r\n", calculateSATID());
        
	parameters.spi_attemptLimit =100;
	parameters.spi_numberOfTests =1000;
	parameters.spi_max_crc_fail = 40;
	parameters.spi_max_attempt_for_packet = 10;
	parameters.spiSensors_numberOfTests =1000;
	parameters.log_enableDetail = 0;
	parameters.spiSensor_max_WR_fail = 1;
 	bool alternateSending = true;
	initRandom();
        
        while (1) {          

          // Main loop

		debug_printf("\r\n***********************************************\r\n");
		debug_printf("M| DTM TEST ENVIRONMENT Choose an option:\r\n");
		debug_printf("M| 0 - Options\r\n");
		debug_printf("M| 1 - Set tolerances\r\n");
		debug_printf("M| 2 - Execute test 1.A: debug SPI\r\n");
                debug_printf("M| 3 - Execute test X.X: debug SPI Atheros\r\n");
                debug_printf("M| 4 - WiFi direct setting\r\n");
		debug_printf("M| 5 - Get FW version\r\n");
                debug_printf("M| 7 - Generate HUB_ID\r\n");
		debug_printf("M| 8 - RF test\r\n");
		debug_printf("M| 9 - RF reset parameters\r\n");
                debug_printf("E|\r\n");
		debug_scanf("%u", &user_input);

		switch (user_input)
                {  
                    case 0: 
			debug_printf("S| OPTIONS *****************************************\r\n");

			debug_printf("S|    1 - SPI TEST: set number of attempts packet forwarding. Actual value %u\r\n", parameters.spi_attemptLimit);
			debug_printf("S|    2 - SPI TEST: set number of packages to be shipped. Actual value %d\r\n", parameters.spi_numberOfTests);
			debug_printf("S|    3 - Set Nordic parameters\r\n");
			debug_printf("S|    4 - Set flag enable detailed log. Actual value %u\r\n", parameters.log_enableDetail);
			debug_printf("S|    5 - Set number of transmissions for the Atheros SPI test. Current value %u \r\n", parameters.spiSensors_numberOfTests);
                        debug_printf("S|    6 - Set WIFI test parameters\r\n");
                        debug_printf("E|\r\n");
                        debug_scanf("%u", &user_input);

                        switch (user_input)
                        {
                          case 1:
				debug_printf("S|       1) enter number of attempts packet forwarding:\r\n");
				debug_printf("E|\r\n");
                                debug_scanf("%u", &user_input);
				parameters.spi_attemptLimit = user_input;
                                break;
                          case 2:
				debug_printf("S|       2)enter number of packages to be shipped through SPI:\r\n");
                                debug_printf("E|\r\n");
				debug_scanf("%u", &(parameters.spi_numberOfTests));
                                break;
                          case 3:
                                setNordicParameters();
                                break;
                          case 4:
				debug_printf("S|       4) enter detailed log (0 off - 1 on - 2 all on):\r\n");
                                debug_printf("E|\r\n");
				debug_scanf("%u", &user_input);
				parameters.log_enableDetail = user_input;
                                break;
                          case 5:
                                debug_printf("S|       5)enter number of transmissions for Atheros SPI test:\r\n");
                                debug_printf("E|\r\n");
                                debug_scanf("%u", &(parameters.spiSensors_numberOfTests));
                                break;
                          case 6:
                                debug_printf("6) set WIFI parameters");
                                setWiFiParameters();
                                debug_printf("E|\r\n");
                                break;
                        }
                        break;
                          
                    case 1:
			debug_printf("S| SET TOLERANCES **********************************\r\n");
			debug_printf("S|    0 - Show result codes\r\n");
			debug_printf("S|    1 - SPI TEST: number of FAILED CRD to FAIL test. Actual value %u\r\n", parameters.spi_max_crc_fail);
			debug_printf("S|    2 - SPIsensor TEST: number of FAILED W/R registers. Actual value %u\r\n", parameters.spiSensor_max_WR_fail);
			debug_printf("E|\r\n");
                        debug_scanf("%u", &user_input);

			switch (user_input)
                        {
                          case 0:
                            showResultTag();
                            break;
                          case 1: 
                            debug_printf("S|       1) SPI TEST: set number of FAILED CRD to FAIL test:\r\n");
                            debug_printf("E|\r\n");
                            debug_scanf("%u", &(parameters.spi_max_crc_fail));
                            break;
                          case 2:
                            debug_printf("S|       2) SPIsensor TEST: set number of FAILED W/R to FAIL test:\r\n");
                            debug_printf("E|\r\n");
                            debug_scanf("%u", &(parameters.spiSensor_max_WR_fail));
                            break;
                        }
                        break;
                    case 2:
			spiTest(parameters);
                        break;
                    case 3:
                        atherosSpiTest(parameters);
                        break;
                    case 4:
                        // Setting Wifi 
                        _lwevent_set(&atheros_task_event, 0x01);
                        _sched_yield();
                        break;
                    case 5:
			debug_printf("[HUBDTM="VERSION_RELEASE"]\r\n");
                        break;
                    case 7:
			debug_printf("[HUBIDX=%d]\r\n", calculateSATID());
                        break;
                    case 8:
                    {
                        parameters_t tmpParameters=parameters;
                        setNordicParameters();
                        tmpParameters.spi_numberOfTests = 50;
                        spiTest(tmpParameters);
                        break;
                    }
                    case 9:
                    {
                        spi_tx.dtm_tx_freq = DTM_FREQ_DEFAULT;
                        spi_tx.dtm_power = DTM_ENUM_POWER_DEFAULT;
                        parameters_t tmpParameters=parameters;
                        tmpParameters.spi_numberOfTests = 50;
                        spiTest(tmpParameters);
                        break;
                    }
                }



          
          _time_delay(2);

          
	}

	SHUTDOWN: hub_quit_loop();

	return;
}

