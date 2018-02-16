#ifndef __HUB_SPI_TYPES_H__
#define __HUB_SPI_TYPES_H__

#include <stdint.h>
#include <stdio.h>
#include "radio_test_common_types.h"

#if EVAL_BOARD
// Nordic Eval nRF51-DK SPI pins
#define TS_SPI_CLK_PIN      29   /**< SPI clock GPIO pin number. */
#define TS_SPI_MOSI_PIN     25   /** Master Out Slave In is not very accurate.  This is more like
																		 Nordic out since Nordic is the slave to 22F. */
#define TS_SPI_MISO_PIN     28   /** Nordic In */
#define TS_SPI_SS_PIN       24   /**< SPI Slave Select GPIO pin number. */
#elif PRODUCTION1 /*  TuringSense Sensor Module SPI pins */
#define TS_SPI_CLK_PIN      10   /**< SPI clock GPIO pin number. */
#define TS_SPI_MOSI_PIN      9   /**< SPI Master Out Slave In GPIO pin number. */
#define TS_SPI_MISO_PIN      8   /**< SPI Master In Slave Out GPIO pin number. */
#define TS_SPI_SS_PIN       11   /**< SPI Slave Select GPIO pin number. */
#endif

#endif /*__HUB_SPI_TYPES_H__*/
