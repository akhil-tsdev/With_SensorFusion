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

#if EXPERIMENT
static uint8_t init_io_pins() {
	uint8_t					ret = IO_OK;
	MQX_FILE_PTR 			pins_fd_pwd, pins_fd_cs;
	const GPIO_PIN_STRUCT 	pins_pwd[2] = { BSP_ATHEROS_WIFI_GPIO_PWD_PIN,
											GPIO_LIST_END };

	//cw todo
	const GPIO_PIN_STRUCT 	pins_cs[2] = { BSP_ATHEROS_SPI2_CS0_PIN,
											GPIO_LIST_END };

	/* Initialize pin states */
    GPIO_PDDR_REG(PTD_BASE_PTR) &= ~0x4140;	/* Clear for input pins 6, 8, 14 */
    GPIO_PDDR_REG(PTD_BASE_PTR) |= 0x3A80;  /* Set output pins 7,9,11,12,13 */

    /* Port data direction for output pins*/
    GPIO_PDOR_REG(PTD_BASE_PTR) |= 0x3A00;


    /* Pull CS0 / HM1 to 1 for now */
	pins_fd_cs = fopen("gpio:write", (char *) &pins_cs);

	if (NULL == pins_fd_cs) {
		printf("ERROR: Failed to open GPIO SPI2 CS0 char device for GPIO!\n");
		ret = IO_DEVICE_DOES_NOT_EXIST;
		return ret;
	}

	ioctl(pins_fd_cs, GPIO_IOCTL_WRITE_LOG1, NULL);
	app_time_delay(2);		/* high for 2ms*/

    /* By default, if we open PWD pin, it will be high since we've set the PDOR */
    pins_fd_pwd = fopen(BSP_ATHEROS_WIFI_GPIO_PWD_DEVICE, (char *) &pins_pwd);

	if (NULL == pins_fd_pwd) {
		printf("ERROR: Failed to open GPIO PWD char device for GPIO!\n");
		ret = IO_DEVICE_DOES_NOT_EXIST;
		return ret;
	}
	app_time_delay(30);		/* high for 100 ms */

	fclose(pins_fd_cs);
	fclose(pins_fd_pwd);

	return ret;
}

static uint8_t ts_atheros_init() {
	return init_io_pins();
}

#endif /* EXPERIMENT */
#endif /* PRODUCTION1 */

