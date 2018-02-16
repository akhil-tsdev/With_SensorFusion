/*
 * Copyright TuringSense, Inc © 2015
 * hub_to_nordic.h
 *
 *  Created on: May 5, 2015
 *      Author: cwati
 */

#ifndef ATHEROS_TO_NORDIC_H_
#define ATHEROS_TO_NORDIC_H_

#include <stdbool.h>
//#include <stdio.h>
#include <string.h>

#include <mqx.h>
#include <bsp.h>
#include <spi.h>

#include "fsl_dspi_master_driver.h"

//#include "mpu9250.h"
#include "common_types.h"
#include "hub_main_loop_dtm.h"

//#include "ossf/build.h"
//#include "ossf/tasks.h"
extern uint32_t reset_ts_rtc0[MAX_SENSORS];

void init_nordic (void);
uint32_t talk_to_nordic(nordic_to_hub_t *data);
uint32_t talk_to_nordic_dtm(hub_to_nordic_dtm_t spi_tx, nordic_to_hub_dtm_t *spi_rx);
void reinit_nordic (void);

#endif /* ATHEROS_TO_NORDIC_H_ */
