/*
 * Copyright TuringSense, Inc © 2015
 * hub_to_atheros.c
 *
 *  Created on: Jun 26, 2015
 *      Author: cwati
 */

#include "main.h"
#include "wmiconfig_ts.h"
#include <lwevent.h>

#if PRODUCTION1
#include <stdbool.h>
#include "io.h"
#include "twrk21f120m.h"
/* For production board, need to pull PWD high before starting.. */
#define BSP_ATHEROS_SPI2_CS0_PIN                   (GPIO_PORT_D|GPIO_PIN11) //TONY
#define BSP_ATHEROS_SPI2_CS0_PIN                   (GPIO_PORT_D|GPIO_PIN11) //TONY

/* This is useless.  HM0 and HM1 pins need to be pulled up or down at boot time, way before
 * we enter this state.
 * Changing the pins at this point, will not change the BOOT mode.
 */
static uint32_t init_hm0hm1_pins() {
	uint8_t					ret = IO_OK;
	MQX_FILE_PTR 			pins_fd_pwd, pins_fd_cs;

	//cw todo
	const GPIO_PIN_STRUCT 	pins_cs[2] = { BSP_ATHEROS_SPI2_CS0_PIN, GPIO_LIST_END };
//	/* Green LED on PTA2 */
//	const GPIO_PIN_STRUCT pins_red[2] = { GPIO_RED_LED, GPIO_LIST_END };
//	const GPIO_PIN_STRUCT pins_blue[2] = { GPIO_BLUE_LED, GPIO_LIST_END };
//	const GPIO_PIN_STRUCT pins_green[2] = { GPIO_GREEN_LED, GPIO_LIST_END };
//	uint32_t read_pin_table[2];
//
//	pins_fd = fopen("gpio:output", (char *) &pins_green);
//
//	if (NULL == pins_fd) {
//		printf("ERROR: Failed to open GPIO char device for GPIO!\n");
//	}
//	if (ioctl(pins_fd, GPIO_IOCTL_READ, &read_pin_table) == IO_OK) {
//		if ((read_pin_table[0] & GPIO_PIN_STATUS) == GPIO_PIN_STATUS_1) {
//			/* first pin in the table is set */
//		}
//	}
        
    /* Pull CS0 / HM1 to 1 for now */
	pins_fd_cs = fopen("gpio:write", (char *) &pins_cs);

	if (NULL == pins_fd_cs) {
		printf("ERROR: Failed to open GPIO SPI2 CS0 char device for GPIO!\n");
		ret = IO_DEVICE_DOES_NOT_EXIST;
		return ret;
	}

	ret = ioctl(pins_fd_cs, GPIO_IOCTL_WRITE_LOG0, NULL);
	app_time_delay(2);		/* high for 2ms*/

	fclose(pins_fd_cs);

	return ret;
}

uint32_t ts_atheros_init() {
	return init_hm0hm1_pins();
}

#endif /* PRODUCTION1 */

