/*
 * Copyright TuringSense, Inc © 2015
 * spi_porting_functions.c
 *
 *  Created on: Apr 30, 2015
 *      Author: cwati
 */

#include <string.h>
#include <mqx.h>
#include <bsp.h>
#include <spi.h>
//#include "spi_memory.h"
#include "spi_porting_functions.h"

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetSpiFreq
 * Description   : Gets the clock frequency for SPI module.
 * This function gets the clock frequency for SPI moudle.
 *
 *END**************************************************************************/
uint32_t    CLOCK_SYS_GetSpiFreq(uint32_t instance)
{
    uint32_t freq = 0;
    BSP_CLOCK_CONFIGURATION clock_config;

    clock_config = _bsp_get_clock_configuration();
    freq = _bsp_get_clock(clock_config,CM_CLOCK_SOURCE_BUS);	/* TODO using BUS CLOCK for now */

    return freq;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_EnableSpiClock
 * Description   : Enable the clock for SPI module
 * This function enables the clock for SPI moudle
 *
 *END**************************************************************************/
void SIM_HAL_EnableSpiClock(uint32_t instance)
{
	/* Enable port clock */
    SIM_MemMapPtr   sim = SIM_BASE_PTR;

    switch (instance)
    {
    case 0:
        /* Enable clock gate to DSPI0 module */
        sim->SCGC6 |= SIM_SCGC6_SPI0_MASK;		/* Enable SPI0 clock */
        break;
    case 1:
        /* Enable clock gate to DSPI1 module */
        sim->SCGC6 |= SIM_SCGC6_SPI1_MASK;		/* Enable SPI1 clock */
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_DisableSpiClock
 * Description   : Disable the clock for SPI module
 * This function disables the clock for SPI moudle
 *
 *END**************************************************************************/
void SIM_HAL_DisableSpiClock(uint32_t instance)
{
    SIM_MemMapPtr   sim = SIM_BASE_PTR;

    switch (instance)
    {
    case 0:
        sim->SCGC6 &= ~SIM_SCGC6_SPI0_MASK;		/* Disable SPI0 clock */
        break;
    case 1:
        sim->SCGC6 &= ~SIM_SCGC6_SPI1_MASK;		/* Disable SPI1 clock */
        break;
    default:
        break;
    }
}
