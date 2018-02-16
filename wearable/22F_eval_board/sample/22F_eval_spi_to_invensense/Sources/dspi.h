/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __DSPI_H__
#define __DSPI_H__
/*******************************************************************************
 * Standard C Included Files
 ******************************************************************************/
#include <stdint.h>
/*******************************************************************************
 * SDK Included Files
 ******************************************************************************/
#include "fsl_dspi_master_driver.h"
#include "fsl_dspi_slave_driver.h"
/*******************************************************************************
 * Constants
 ******************************************************************************/
enum _spi_instances
{
    kSensorInstance = 0,
    kRFInstance
};
extern char spi_mts_buffer[17];
extern char spi_stm_buffer[17];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
dspi_status_t data_source(uint8_t * sourceWord, uint32_t instance); 

dspi_status_t data_sink(uint8_t sinkWord, uint32_t instance);

void on_error(dspi_status_t error, uint32_t instance);

void cwati_dspi_edma_master_setup(dspi_master_state_t *dspiMasterState, uint8_t instance, uint32_t baudRateHz, uint8_t transferSizeBits);
void dspi_edma_master_setup(dspi_master_state_t *dspiMasterState, uint8_t instance, uint32_t baudRateHz, uint8_t transferSizeBits);

void dspi_edma_slave_setup(dspi_slave_state_t *dspiSlaveState, dspi_slave_user_config_t *slaveUserConfig, uint8_t instance, uint8_t transferSizeBits);

void cwajh_dspi_slave_setup(dspi_slave_state_t *dspiSlaveState, uint8_t instance, uint8_t transferSizeBits);

void dspi_edma_deinit(dspi_master_state_t *dspiMasterState, dspi_slave_state_t *dspiSlaveState, dspi_slave_user_config_t *slaveUserConfig);

/*******************************************************************************
 * CWATI
 ******************************************************************************/
uint32_t cwati_isp_send(uint8_t writeAddr);
dspi_master_user_config_t masterSensorConfig;
dspi_device_t masterSpiDevice;

#endif /* __DSPI_H__ */

/******************************************************************************
 * EOF
 ******************************************************************************/
