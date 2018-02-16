/*
 * Copyright TuringSense, Inc © 2015
 * hub_22f.c
 *
 *  Created on: May 15, 2015
 *      Author: cwati
 *
 *
 */

#include <mqx.h>
#include <mutex.h>

#include "hub_main_loop.h"
#include "main.h"
#include "throughput.h"
#include "virtual_com.h"
#include "wmiconfig_ts.h"

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

uint32_t cur_sat_ids[MAX_SENSORS] = {0};

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

  app_time_delay(300); /* Wait in millisecond.  Minimum 100-150ms. */

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

void initialize_ids(void) {
#if USE_FLASH
    err_t           flash_ret;
    static char buffer[TS_BUFFER_SIZE] = {0};
    _mqx_int        len = 0;
        
    flash_ret = hub_flash_init();
    if (flash_ret != E_OK) {
      ts_blinkLEDerror(redLed, 4);
    }
    
    // Example to read flash size.  Disable for now.
    //uint32_t flash_size = hub_flash_size();

    /* Read hub ID */
    flash_ret = hub_flash_read32(&cur_hub_id, HUB_ID_OFF); 
    if (flash_ret != E_OK) {
        printf("\nERROR! Could not read from flash. Exiting...");
        ts_blinkLEDerror(redLed, 4);
    }

    for (uint8_t qq = 0; qq < TS_FLASH_READ_SIZE; qq++) {
        buffer[qq]++;
    }
    
    //not needed for now
    //buffer[TS_FLASH_READ_SIZE] = '\0';
    
    flash_ret = hub_flash_write32(buffer, HUB_ID_OFF);
    if (flash_ret != E_OK) {
        printf("\nError writing to the file. Error code: %d", flash_ret);
        ts_blinkLEDerror(redLed, 4);
    }
#endif /* USE_FLASH */
}

/*TASK*-------------------------------------------------------------------
 *
 * Task Name : hub_main_loop
 *
 *END*----------------------------------------------------------------------*/
void hub_main_loop(uint32_t dummy) {
	uint32_t		ret32;

	printf("Hi! Welcome to hub main loop\n");

	_int_install_unexpected_isr();
        
	ts_init_LED();
	ts_turn_on_LED(whiteLed);
        TestApp_Init(); /* Virtual Com */  

        debug_printf("\r\n***********************************************\r\n");
        debug_printf("Turingsense Direct Test Mode tool (c) 2016\r\n");

        ts_atheros_init();

	while (1) {
          app_time_delay(3); /* Wait in millisecond */
	}

	SHUTDOWN: hub_quit_loop();

	return;
}

