/*
 * Copyright TuringSense, Inc © 2015
 * ts_io_led.c
 *
 *  Created on: Nov 25, 2015
 *      Author: cwati
 *
 * This file contains LED functions for Turingsense boards.
 *
 */
#include "ts_io_led.h"
#include "ts_utils.h"
   
//#include <lwevent.h>
//#include <mutex.h>
#include <stdint.h>
//#include <stdio.h>
#include "io_gpio.h"
/*************************** GLOBAL ************************/
/* cwati I'm not sure how to get gpio output pin's values, thus this global
 * This is not a graceful solution, I must admit. #indianwells2016 */

static uint8_t rgb[3] = {0};

/* Upon unrecoverable error, we call this function and
 * it will blink the LED.
 *
 * WARNING: This function does NOT return!!!
 * It's an infinite loop!!
 */
void ts_blinkLEDerror(hub_led_t color, uint32_t multiplier) {
	printf("Unrecoverable error happened! %d", multiplier);
//	_task_stop_preemption();
	while (1) {
		ts_turn_on_LED(color);
		app_time_delay(300*multiplier);
		ts_turn_on_LED(offLed);
		app_time_delay(300*multiplier);
	}
        
	/* Will never get out of the blink and other tasks shouldn't run */
//	_task_start_preemption();
}

void ts_init_LED() {
	MQX_FILE_PTR green_pins_fd, red_pins_fd, blue_pins_fd;

	const GPIO_PIN_STRUCT pins_red[2] = { GPIO_RED_LED, GPIO_LIST_END };
	const GPIO_PIN_STRUCT pins_blue[2] = { GPIO_BLUE_LED, GPIO_LIST_END };
	const GPIO_PIN_STRUCT pins_green[2] = { GPIO_GREEN_LED, GPIO_LIST_END };
	uint32_t read_pin_table[2];

	green_pins_fd = fopen("gpio:output", (char *) &pins_green);
	if (NULL == green_pins_fd) {
		printf("ERROR: Failed to open GPIO char device for GPIO!\n");
	}

	red_pins_fd = fopen("gpio:output", (char *) &pins_red);
	if (NULL == red_pins_fd) {
		printf("ERROR: Failed to open GPIO char device for GPIO!\n");
	}

	blue_pins_fd = fopen("gpio:output", (char *) &pins_blue);
	if (NULL == blue_pins_fd) {
		printf("ERROR: Failed to open GPIO char device for GPIO!\n");
	}
	ioctl(green_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL);
	rgb[greenLed] = 1;
	ioctl(red_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL);
	rgb[redLed] = 1;
	ioctl(blue_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL);
	rgb[blueLed] = 1;

	fclose(red_pins_fd);
	fclose(green_pins_fd);
	fclose(blue_pins_fd);
}

/* This would turn on/off just one LED color and leave the rest unchanged. */
void ts_turnOnOneLED(hub_led_t led_color, uint8_t on) {
	MQX_FILE_PTR green_pins_fd, red_pins_fd, blue_pins_fd;
	MQX_FILE_PTR tmp;
	uint8_t dummy_index;

	/* Green LED on PTA2 */
	const GPIO_PIN_STRUCT pins_red[2] = { GPIO_RED_LED, GPIO_LIST_END };
	const GPIO_PIN_STRUCT pins_blue[2] = { GPIO_BLUE_LED, GPIO_LIST_END };
	const GPIO_PIN_STRUCT pins_green[2] = { GPIO_GREEN_LED, GPIO_LIST_END };
	uint32_t read_pin_table[2];

	green_pins_fd = fopen("gpio:output", (char *) &pins_green);
	if (NULL == green_pins_fd) {
		printf("ERROR: Failed to open GPIO char device for GPIO!\n");
	}

	red_pins_fd = fopen("gpio:output", (char *) &pins_red);
	if (NULL == red_pins_fd) {
		printf("ERROR: Failed to open GPIO char device for GPIO!\n");
	}

	blue_pins_fd = fopen("gpio:output", (char *) &pins_blue);
	if (NULL == blue_pins_fd) {
		printf("ERROR: Failed to open GPIO char device for GPIO!\n");
	}

	switch (led_color) {
	case redLed:
		tmp = red_pins_fd;
		dummy_index = redLed;
		break;
	case greenLed:
		tmp = green_pins_fd;
		dummy_index = greenLed;
		break;
	case blueLed:
		tmp = blue_pins_fd;
		dummy_index = blueLed;
		break;
	}

	if (on) {
		if (ioctl(tmp, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
			rgb[dummy_index] = 0;
	//		printf("OK write log1\n");
		}
	} else {
		if (ioctl(tmp, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
			rgb[dummy_index] = 1;
	//		printf("OK write log1\n");
		}
	}

	fclose(red_pins_fd);
	fclose(green_pins_fd);
	fclose(blue_pins_fd);
}

/*
 * This is more for diagnostics and to verify that the program has been loaded successfully.
 */
void ts_turn_on_LED(hub_led_t led_color) {

	MQX_FILE_PTR green_pins_fd, red_pins_fd, blue_pins_fd;

	/* Green LED on PTA2 */
	const GPIO_PIN_STRUCT pins_red[2] = { GPIO_RED_LED, GPIO_LIST_END };
	const GPIO_PIN_STRUCT pins_blue[2] = { GPIO_BLUE_LED, GPIO_LIST_END };
	const GPIO_PIN_STRUCT pins_green[2] = { GPIO_GREEN_LED, GPIO_LIST_END };
	uint32_t read_pin_table[2];

	green_pins_fd = fopen("gpio:output", (char *) &pins_green);
	if (NULL == green_pins_fd) {
		printf("ERROR: Failed to open GPIO char device for GPIO!\n");
	}

	red_pins_fd = fopen("gpio:output", (char *) &pins_red);
	if (NULL == red_pins_fd) {
		printf("ERROR: Failed to open GPIO char device for GPIO!\n");
	}

	blue_pins_fd = fopen("gpio:output", (char *) &pins_blue);
	if (NULL == blue_pins_fd) {
		printf("ERROR: Failed to open GPIO char device for GPIO!\n");
	}

	switch (led_color) {
	case offLed:
		if (ioctl(red_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
			rgb[redLed] = 1;
	//		printf("OK write log1\n");
		}
		if (ioctl(green_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
			rgb[greenLed] = 1;
	//		printf("OK write log1\n");
		}
		if (ioctl(blue_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
			rgb[blueLed] = 1;
	//		printf("OK write log1\n");
		}
		break;
	case redLed:
		if (ioctl(red_pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
			rgb[redLed] = 0;
	//		printf("OK write log1\n");
		}
		if (ioctl(green_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
			rgb[greenLed] = 1;
	//		printf("OK write log1\n");
		}
		if (ioctl(blue_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
			rgb[blueLed] = 1;
	//		printf("OK write log1\n");
		}
		break;
	case greenLed:
		if (ioctl(red_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
			rgb[redLed] = 1;
	//		printf("OK write log1\n");
		}
		if (ioctl(green_pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
			rgb[greenLed] = 0;
	//		printf("OK write log1\n");
		}
		if (ioctl(blue_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
			rgb[blueLed] = 1;
	//		printf("OK write log1\n");
		}
		break;
	case blueLed:
		if (ioctl(red_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
			rgb[redLed] = 1;
	//		printf("OK write log1\n");
		}
		if (ioctl(green_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
			rgb[greenLed] = 1;
	//		printf("OK write log1\n");
		}
		if (ioctl(blue_pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
			rgb[blueLed] = 0;
	//		printf("OK write log1\n");
		}
		break;
	case yellowLed:
		if (ioctl(red_pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
			rgb[redLed] = 0;
	//		printf("OK write log1\n");
		}
		if (ioctl(green_pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
			rgb[greenLed] = 0;
	//		printf("OK write log1\n");
		}
		if (ioctl(blue_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
			rgb[blueLed] = 1;
	//		printf("OK write log1\n");
		}
		break;
	case purpleLed:
		if (ioctl(red_pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
			rgb[redLed] = 0;
	//		printf("OK write log1\n");
		}
		if (ioctl(green_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
			rgb[greenLed] = 1;
	//		printf("OK write log1\n");
		}
		if (ioctl(blue_pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
			rgb[blueLed] = 0;
	//		printf("OK write log1\n");
		}
		break;
	case whiteLed:
		if (ioctl(red_pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
			rgb[redLed] = 0;
	//		printf("OK write log1\n");
		}
		if (ioctl(green_pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
			rgb[greenLed] = 0;
	//		printf("OK write log1\n");
		}
		if (ioctl(blue_pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
			rgb[blueLed] = 0;
	//		printf("OK write log0\n");
		}
		break;                
	case turqoiseLed:
	default:
		if (ioctl(red_pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
			rgb[redLed] = 1;
	//		printf("OK write log1\n");
		}
		if (ioctl(green_pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
			rgb[greenLed] = 0;
	//		printf("OK write log1\n");
		}
		if (ioctl(blue_pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
			rgb[blueLed] = 0;
	//		printf("OK write log1\n");
		}
		break;
	}

	fclose(red_pins_fd);
	fclose(green_pins_fd);
	fclose(blue_pins_fd);
}

uint8_t ts_get_red_LED() {
	return rgb[redLed];
}

uint8_t ts_get_green_LED() {
	return rgb[greenLed];
}

uint8_t ts_get_blue_LED() {
	return rgb[blueLed];
}

void ts_doubleBlinkLED (hub_led_t color) {
    ts_turn_on_LED(color);
    app_time_delay(100); /* Wait in millisecond. */
    ts_turn_on_LED(offLed);
    app_time_delay(100); /* Wait in millisecond. */ 
    ts_turn_on_LED(color);
    app_time_delay(100); /* Wait in millisecond. */
}
