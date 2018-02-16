/*
 * Copyright TuringSense, Inc © 2016
 * hub_usb.c
 *
 *  Created on: June 21, 2016
 *      Author: cwati
 *
 *
 */

#include <mqx.h>
#include <mutex.h>

#include "atheros_main_loop.h"
#include "hub_main_loop.h"
#include "hub_to_nordic.h"
#include "main.h"
#include "throughput.h"
#include "wmiconfig_ts.h"

//cw debug todo remove me when done
extern uint32_t ts_diff_arr[];

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
	return (c & WIFI_START);
}
uint32_t hub_not_active (uint32_t c) {
	return (!(c & WIFI_START));
}
uint32_t hub_can_send (uint32_t c) {
	return (!(c & WIFI_WAIT));
}
uint32_t hub_cannot_send (uint32_t c) {
	return (c & WIFI_WAIT);
}
void set_hub_can_send(uint32_t* c) {
	*c &= ~WIFI_WAIT;
	hub_to_nordic.command &= ~NORDIC_WAIT;
}
void set_hub_cannot_send(uint32_t* c)	{
	*c |= WIFI_WAIT;
	hub_to_nordic.command |= NORDIC_WAIT;
}

void set_hub_active(uint32_t* c) {
	*c |= WIFI_START;
	hub_to_nordic.command |= NORDIC_START;
}

void set_hub_not_active(uint32_t* c) {
	*c &= ~WIFI_START;
	hub_to_nordic.command &= ~NORDIC_START;
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

/*TASK*-------------------------------------------------------------------
 *
 * Task Name : hub_main_loop
 *
 *END*----------------------------------------------------------------------*/
void hub_main_loop(uint32_t dummy) {
	A_INT32 		return_code, timeout;
	A_UINT8 		i;
	MQX_FILE_PTR 	pins_fd;
	GPIO_PIN_STRUCT pins[2];
	err_t			ret;
	uint32_t		ret32;
	uint8_t			cbuf_full = FALSE;
	uint8_t			sat_idx;
	static uint16_t cnt_print = 1;//todo remove me

	printf("Hi! Welcome to hub main loop%s\n", ENABLE_MAG ? "! (MAG enabled)" : "!");

	_int_install_unexpected_isr();

	hub_to_nordic.command = 0;
	assert_nordic_reset();
	init_nordic();

	for (sat_idx = 0; sat_idx < MAX_SENSORS; sat_idx++) {
		cbufInit(&cbuf[sat_idx]);
	}

	ts_init_LED();
	ts_turn_on_LED(greenLed);
	/* Other init */
#if ENABLE_WIFI
	set_hub_cannot_send(&current_state);
	set_hub_not_active(&current_state);
#else
	set_hub_can_send(&current_state);
	set_hub_active(&current_state);
#endif

#if !TMPDEBUG
	ts_turn_on_LED(turqoiseLed);
#endif
        
	/* Initialize state to Nordic */
	hub_to_nordic.payload.cmd_field2 = cur_num_of_sat;
	hub_to_nordic.command |= NORDIC_SET_SATELLITES;
	hub_to_nordic.command |= NORDIC_SET_RTC;
	hub_to_nordic.payload.cmd_field1 = 0;
	printf("\n\rCurrent num of sat: %d\n", hub_to_nordic.payload.cmd_field2);
	for (uint8_t cnt1 = 0; cnt1 < cur_num_of_sat; cnt1++) {
		hub_to_nordic.payload.satellite_ids[cnt1] = cur_sat_ids[cnt1];
		printf("Sat[%d]: 0x%x\n", cnt1, hub_to_nordic.payload.satellite_ids[cnt1]);
	}

    talk_to_nordic(&nordic_to_hub);

	while (1) {
		talk_to_nordic(&nordic_to_hub);
		_time_delay(2);

		_lwevent_set(&atheros_task_event, 0x01);
		_sched_yield();

#if DEBUG_DELAY
		app_time_delay(3000); /* Wait in millisecond */
#endif
	}

	SHUTDOWN: hub_quit_loop();

	return;
}

