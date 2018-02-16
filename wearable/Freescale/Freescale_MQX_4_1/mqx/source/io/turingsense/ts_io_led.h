/*
 * Copyright TuringSense, Inc © 2015
 * hub_main_loop.h
 *
 *  Created on: May 11, 2015
 *      Author: cwati
 */

#ifndef __TS_IO_LED_H__
#define __TS_IO_LED_H__
   
#include <mqx.h>
#include "fio.h"

#if (EVAL_BOARD)
#define GPIO_RED_LED			(GPIO_PORT_A|GPIO_PIN1)		/* This pin is set to 1 if Atheros is sending via wifi */
#define GPIO_GREEN_LED			(GPIO_PORT_A|GPIO_PIN2)	
#define GPIO_BLUE_LED			(GPIO_PORT_A|GPIO_PIN5)	
#else
#if (PRODUCTION1)
#define GPIO_RED_LED			(GPIO_PORT_A|GPIO_PIN10)
#define GPIO_GREEN_LED			(GPIO_PORT_A|GPIO_PIN11)	
#define GPIO_BLUE_LED			(GPIO_PORT_A|GPIO_PIN12)	
#endif /* PRODUCTION1 */
#endif /* EVAL_BOARD */

typedef enum {
	redLed,
	greenLed,
	blueLed,
	offLed,
	yellowLed,
	turqoiseLed,
	purpleLed,
        whiteLed
} hub_led_t;

void ts_blinkLEDerror(hub_led_t color, uint32_t multiplier);
void ts_init_LED();
void ts_turnOnOneLED(hub_led_t led_color, uint8_t on);
void ts_turn_on_LED(hub_led_t led_color);
void ts_turn_off_LED(MQX_FILE_PTR pins_fd);
void ts_doubleBlinkLED (hub_led_t color);

uint8_t ts_get_red_LED();
uint8_t ts_get_green_LED();
uint8_t ts_get_blue_LED();

#endif /* __TS_IO_LED_H__ */
