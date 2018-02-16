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

/*******************************************************************************
 * Application Included Files
 ******************************************************************************/
#include "dspi.h"
/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern uint8_t g_slaveRxBuffer[256];
extern volatile uint16_t g_slaveTxCount;
extern volatile uint32_t g_errorFlag;
extern uint32_t g_dspiBaseAddr[];

/*******************************************************************************
 * CWATI Global Variables
 ******************************************************************************/
dspi_master_user_config_t masterSensorConfig;
dspi_device_t masterSpiDevice;
/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
/*
    @brief Callback function for DSPI slave. Used to transmit data from slave. In this application it generates slave data out, the data out is a count.

    @param sourceWord 8-bit data variable to be passed to slave PUSHR register.

    @param instance Instance of the DSPI module.
 */

dspi_status_t data_source(uint8_t * sourceWord, uint32_t instance)
{
    if(g_slaveTxCount < (sizeof(g_slaveRxBuffer) / 2))
    {
        *sourceWord = (uint8_t)g_slaveTxCount++;

        if(g_slaveTxCount == ((sizeof(g_slaveRxBuffer) / 2) - 1))
        {
            g_errorFlag = 0x00DEAD00; //Set count to termination message
        }
    }

    return kStatus_DSPI_Success;
}

dspi_status_t data_sink(uint8_t sinkWord, uint32_t instance)
{
    return kStatus_DSPI_Success;
}

/*
    @brief Callback function for DSPI slave. Used to handle errors. In this application is sets the error flag to signal the end of the demonstration.

    @param error Uses dspi_status_t value given by driver interrupt handler.

    @param instance Instance of the DSPI module.
 */

void on_error(dspi_status_t error, uint32_t instance)
{
    /* Perform error handling in here. */
}

/*
    @brief CWATI Takes in the DSPI module instance, the desired data rate of DSPI transfers, and the frame size of the data transfer.

    @param instance DSPI module instance

    @param baudRateHz Pass in the desired baud rate of DSPI transfers in Hz.

    @param transferSizeBits Pass data frame size (8 or 16 bit)
 */

void cwati_dspi_edma_master_setup(dspi_master_state_t *dspiMasterState, uint8_t instance, uint32_t baudRateHz, uint8_t transferSizeBits)
{
	/* Setting up Invensense */
    uint32_t baudRate;
    masterSensorConfig.isChipSelectContinuous = false;
    masterSensorConfig.isSckContinuous = false;
    masterSensorConfig.pcsPolarity = kDspiPcs_ActiveLow;///cwati ///Idle HIGH!
    masterSensorConfig.whichCtar = kDspiCtar0;
    masterSensorConfig.whichPcs = kDspiPcs0;   /* Sensor is using SPI1_PCS0 */

    DSPI_DRV_MasterInit(instance, dspiMasterState, &masterSensorConfig);
    //cwati thinks the original code should have used DSPI_DRV_MasterInitDma

    masterSpiDevice.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge; //CPHA=0
    masterSpiDevice.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    masterSpiDevice.dataBusConfig.direction = kDspiMsbFirst;
    masterSpiDevice.bitsPerSec = baudRateHz;
    masterSpiDevice.dataBusConfig.bitsPerFrame = transferSizeBits;
    DSPI_DRV_MasterConfigureBus(instance, &masterSpiDevice, &baudRate);

    //cwati disable for now
//    DSPI_HAL_SetTxFifoFillDmaIntMode(g_dspiBaseAddr[instance], kDspiGenerateDmaReq, true);
//    DSPI_HAL_SetRxFifoDrainDmaIntMode(g_dspiBaseAddr[instance], kDspiGenerateDmaReq, true);

}

/*
    @brief Takes in the DSPI module instance, the desired data rate of DSPI transfers, and the frame size of the data transfer.

    @param instance DSPI module instance

    @param baudRateHz Pass in the desired baud rate of DSPI transfers in Hz.

    @param transferSizeBits Pass data frame size (8 or 16 bit)
 */

void dspi_edma_master_setup(dspi_master_state_t *dspiMasterState, uint8_t instance, uint32_t baudRateHz, uint8_t transferSizeBits)
{
    uint32_t baudRate;

    dspi_master_user_config_t userConfig;
    userConfig.isChipSelectContinuous = false;
    userConfig.isSckContinuous = false;
    userConfig.pcsPolarity = kDspiPcs_ActiveLow;
    userConfig.whichCtar = kDspiCtar0;
    userConfig.whichPcs = kDspiPcs1;

    DSPI_DRV_MasterInit(instance, dspiMasterState, &userConfig);
    //cwati thinks the original code should have used DSPI_DRV_MasterInitDma

    dspi_device_t spiDevice;
    spiDevice.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge;
    spiDevice.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveLow;
    spiDevice.dataBusConfig.direction = kDspiMsbFirst;

    spiDevice.bitsPerSec = baudRateHz;
    spiDevice.dataBusConfig.bitsPerFrame = transferSizeBits;

    DSPI_DRV_MasterConfigureBus(instance, &spiDevice, &baudRate);
//cwati
//    DSPI_HAL_SetTxFifoFillDmaIntMode(g_dspiBaseAddr[instance], kDspiGenerateDmaReq, true);
//    DSPI_HAL_SetRxFifoDrainDmaIntMode(g_dspiBaseAddr[instance], kDspiGenerateDmaReq, true);

}

/*
    @brief Takes in the DSPI module instance, and the frame size of the data transfer. Initializes slave instance of DSPI.

    @param instance DSPI module instance

    @param transferSizeBits Pass data frame size (8 or 16 bit)
 */

void dspi_edma_slave_setup(dspi_slave_state_t *dspiSlaveState, dspi_slave_user_config_t *slaveUserConfig, uint8_t instance, uint8_t transferSizeBits)
{
    g_slaveTxCount = 0;

    slaveUserConfig->callbacks.dataSink = data_sink;
    slaveUserConfig->callbacks.dataSource = data_source;
    slaveUserConfig->callbacks.onError = on_error;
    slaveUserConfig->dataConfig.clkPhase = kDspiClockPhase_FirstEdge;
    slaveUserConfig->dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;

    slaveUserConfig->dataConfig.bitsPerFrame = transferSizeBits;

    DSPI_DRV_SlaveInit(instance, dspiSlaveState, slaveUserConfig);

    DSPI_HAL_SetIntMode(g_dspiBaseAddr[instance], kDspiTxFifoUnderflow, false);
    DSPI_HAL_SetIntMode(g_dspiBaseAddr[instance], kDspiTxFifoFillRequest, false);

    DSPI_HAL_SetTxFifoFillDmaIntMode(g_dspiBaseAddr[instance], kDspiGenerateIntReq, true);
    DSPI_HAL_SetRxFifoDrainDmaIntMode(g_dspiBaseAddr[instance], kDspiGenerateDmaReq, true);

    g_errorFlag = 0;
}

void dspi_edma_deinit(dspi_master_state_t *dspiMasterState, dspi_slave_state_t *dspiSlaveState, dspi_slave_user_config_t *slaveUserConfig)
{
	DSPI_DRV_MasterDeinit(kRFInstance);
//    DSPI_DRV_MasterDeinit(kMasterInstance);
//    DSPI_DRV_SlaveDeinit(kSlaveInstance);

    OSA_MemFree(dspiMasterState);
    OSA_MemFree(dspiSlaveState);
    OSA_MemFree(slaveUserConfig);

}

/******************************************************************************
 * EOF
 ******************************************************************************/
