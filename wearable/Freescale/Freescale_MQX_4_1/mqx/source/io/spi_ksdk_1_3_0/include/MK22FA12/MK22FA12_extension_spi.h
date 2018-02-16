/*
 * Copyright TuringSense, Inc © 2015
 * MK22FA12_extension_spi.h
 *
 *  Created on: July 30, 2015
 *      Author: cwati
 *
 * 	Taken from MK22FA12_extension.h from KSDK 1.3.0
 *	Chopping off the rest of the file, only using the SPI part.
 */

/*
** ###################################################################
**     Compilers:           Keil ARM C/C++ Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          GNU C Compiler - CodeSourcery Sourcery G++
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    K22P64M120SF5V2RM, Rev.5, March 2015
**     Version:             rev. 1.0, 2015-04-07
**     Build:               b150612
**
**     Abstract:
**         Extension to the CMSIS register access layer header.
**
**     Copyright (c) 2015 Freescale Semiconductor, Inc.
**     All rights reserved.
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 1.0 (2015-04-07)
**         Initial version
**
** ###################################################################
*/

#ifndef __MK22FA12_EXTENSION_SPI_H__
#define __MK22FA12_EXTENSION_SPI_H__

//#include "MK22FA12_spi.h"
#include "fsl_bitaccess.h"

/*
 * MK22FA12 SPI
 *
 * Serial Peripheral Interface
 *
 * Registers defined in this header file:
 * - SPI_MCR - Module Configuration Register
 * - SPI_TCR - Transfer Count Register
 * - SPI_CTAR - Clock and Transfer Attributes Register (In Master Mode)
 * - SPI_CTAR_SLAVE - Clock and Transfer Attributes Register (In Slave Mode)
 * - SPI_SR - Status Register
 * - SPI_RSER - DMA/Interrupt Request Select and Enable Register
 * - SPI_PUSHR - PUSH TX FIFO Register In Master Mode
 * - SPI_PUSHR_SLAVE - PUSH TX FIFO Register In Slave Mode
 * - SPI_POPR - POP RX FIFO Register
 * - SPI_TXFR0 - Transmit FIFO Registers
 * - SPI_TXFR1 - Transmit FIFO Registers
 * - SPI_TXFR2 - Transmit FIFO Registers
 * - SPI_TXFR3 - Transmit FIFO Registers
 * - SPI_RXFR0 - Receive FIFO Registers
 * - SPI_RXFR1 - Receive FIFO Registers
 * - SPI_RXFR2 - Receive FIFO Registers
 * - SPI_RXFR3 - Receive FIFO Registers
 */

#define SPI_INSTANCE_COUNT (3U) /*!< Number of instances of the SPI module. */
#define SPI0_IDX (0U) /*!< Instance number for SPI0. */
#define SPI1_IDX (1U) /*!< Instance number for SPI1. */
#define SPI2_IDX (2U) /*!< Instance number for SPI2. */

/*******************************************************************************
 * SPI_MCR - Module Configuration Register
 ******************************************************************************/

/*!
 * @brief SPI_MCR - Module Configuration Register (RW)
 *
 * Reset value: 0x00004001U
 *
 * Contains bits to configure various attributes associated with the module
 * operations. The HALT and MDIS bits can be changed at any time, but the effect
 * takes place only on the next frame boundary. Only the HALT and MDIS bits in the
 * MCR can be changed, while the module is in the Running state.
 */
/*!
 * @name Constants and macros for entire SPI_MCR register
 */
/*@{*/
#define SPI_RD_MCR(base)         (SPI_MCR_REG(base))
#define SPI_WR_MCR(base, value)  (SPI_MCR_REG(base) = (value))
#define SPI_RMW_MCR(base, mask, value) (SPI_WR_MCR(base, (SPI_RD_MCR(base) & ~(mask)) | (value)))
#define SPI_SET_MCR(base, value) (SPI_WR_MCR(base, SPI_RD_MCR(base) |  (value)))
#define SPI_CLR_MCR(base, value) (SPI_WR_MCR(base, SPI_RD_MCR(base) & ~(value)))
#define SPI_TOG_MCR(base, value) (SPI_WR_MCR(base, SPI_RD_MCR(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SPI_MCR bitfields
 */

/*!
 * @name Register SPI_MCR, field HALT[0] (RW)
 *
 * The HALT bit starts and stops frame transfers. See Start and Stop of Module
 * transfers
 *
 * Values:
 * - 0b0 - Start transfers.
 * - 0b1 - Stop transfers.
 */
/*@{*/
/*! @brief Read current value of the SPI_MCR_HALT field. */
#define SPI_RD_MCR_HALT(base) ((SPI_MCR_REG(base) & SPI_MCR_HALT_MASK) >> SPI_MCR_HALT_SHIFT)
#define SPI_BRD_MCR_HALT(base) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_HALT_SHIFT))

/*! @brief Set the HALT field to a new value. */
#define SPI_WR_MCR_HALT(base, value) (SPI_RMW_MCR(base, SPI_MCR_HALT_MASK, SPI_MCR_HALT(value)))
#define SPI_BWR_MCR_HALT(base, value) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_HALT_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_MCR, field SMPL_PT[9:8] (RW)
 *
 * Controls when the module master samples SIN in Modified Transfer Format. This
 * field is valid only when CPHA bit in CTARn[CPHA] is 0.
 *
 * Values:
 * - 0b00 - 0 protocol clock cycles between SCK edge and SIN sample
 * - 0b01 - 1 protocol clock cycle between SCK edge and SIN sample
 * - 0b10 - 2 protocol clock cycles between SCK edge and SIN sample
 * - 0b11 - Reserved
 */
/*@{*/
/*! @brief Read current value of the SPI_MCR_SMPL_PT field. */
#define SPI_RD_MCR_SMPL_PT(base) ((SPI_MCR_REG(base) & SPI_MCR_SMPL_PT_MASK) >> SPI_MCR_SMPL_PT_SHIFT)
#define SPI_BRD_MCR_SMPL_PT(base) (SPI_RD_MCR_SMPL_PT(base))

/*! @brief Set the SMPL_PT field to a new value. */
#define SPI_WR_MCR_SMPL_PT(base, value) (SPI_RMW_MCR(base, SPI_MCR_SMPL_PT_MASK, SPI_MCR_SMPL_PT(value)))
#define SPI_BWR_MCR_SMPL_PT(base, value) (SPI_WR_MCR_SMPL_PT(base, value))
/*@}*/

/*!
 * @name Register SPI_MCR, field CLR_RXF[10] (WORZ)
 *
 * Flushes the RX FIFO. Writing a 1 to CLR_RXF clears the RX Counter. The
 * CLR_RXF bit is always read as zero.
 *
 * Values:
 * - 0b0 - Do not clear the RX FIFO counter.
 * - 0b1 - Clear the RX FIFO counter.
 */
/*@{*/
/*! @brief Set the CLR_RXF field to a new value. */
#define SPI_WR_MCR_CLR_RXF(base, value) (SPI_RMW_MCR(base, SPI_MCR_CLR_RXF_MASK, SPI_MCR_CLR_RXF(value)))
#define SPI_BWR_MCR_CLR_RXF(base, value) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_CLR_RXF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_MCR, field CLR_TXF[11] (WORZ)
 *
 * Flushes the TX FIFO. Writing a 1 to CLR_TXF clears the TX FIFO Counter. The
 * CLR_TXF bit is always read as zero.
 *
 * Values:
 * - 0b0 - Do not clear the TX FIFO counter.
 * - 0b1 - Clear the TX FIFO counter.
 */
/*@{*/
/*! @brief Set the CLR_TXF field to a new value. */
#define SPI_WR_MCR_CLR_TXF(base, value) (SPI_RMW_MCR(base, SPI_MCR_CLR_TXF_MASK, SPI_MCR_CLR_TXF(value)))
#define SPI_BWR_MCR_CLR_TXF(base, value) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_CLR_TXF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_MCR, field DIS_RXF[12] (RW)
 *
 * When the RX FIFO is disabled, the receive part of the module operates as a
 * simplified double-buffered SPI. This bit can only be written when the MDIS bit
 * is cleared.
 *
 * Values:
 * - 0b0 - RX FIFO is enabled.
 * - 0b1 - RX FIFO is disabled.
 */
/*@{*/
/*! @brief Read current value of the SPI_MCR_DIS_RXF field. */
#define SPI_RD_MCR_DIS_RXF(base) ((SPI_MCR_REG(base) & SPI_MCR_DIS_RXF_MASK) >> SPI_MCR_DIS_RXF_SHIFT)
#define SPI_BRD_MCR_DIS_RXF(base) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_DIS_RXF_SHIFT))

/*! @brief Set the DIS_RXF field to a new value. */
#define SPI_WR_MCR_DIS_RXF(base, value) (SPI_RMW_MCR(base, SPI_MCR_DIS_RXF_MASK, SPI_MCR_DIS_RXF(value)))
#define SPI_BWR_MCR_DIS_RXF(base, value) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_DIS_RXF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_MCR, field DIS_TXF[13] (RW)
 *
 * When the TX FIFO is disabled, the transmit part of the module operates as a
 * simplified double-buffered SPI. This bit can be written only when the MDIS bit
 * is cleared.
 *
 * Values:
 * - 0b0 - TX FIFO is enabled.
 * - 0b1 - TX FIFO is disabled.
 */
/*@{*/
/*! @brief Read current value of the SPI_MCR_DIS_TXF field. */
#define SPI_RD_MCR_DIS_TXF(base) ((SPI_MCR_REG(base) & SPI_MCR_DIS_TXF_MASK) >> SPI_MCR_DIS_TXF_SHIFT)
#define SPI_BRD_MCR_DIS_TXF(base) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_DIS_TXF_SHIFT))

/*! @brief Set the DIS_TXF field to a new value. */
#define SPI_WR_MCR_DIS_TXF(base, value) (SPI_RMW_MCR(base, SPI_MCR_DIS_TXF_MASK, SPI_MCR_DIS_TXF(value)))
#define SPI_BWR_MCR_DIS_TXF(base, value) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_DIS_TXF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_MCR, field MDIS[14] (RW)
 *
 * Allows the clock to be stopped to the non-memory mapped logic in the module
 * effectively putting it in a software-controlled power-saving state. The reset
 * value of the MDIS bit is parameterized, with a default reset value of 0. When
 * the module is used in Slave Mode, it is recommended to leave this bit 0,
 * because a slave doesn't have control over master transactions.
 *
 * Values:
 * - 0b0 - Enables the module clocks.
 * - 0b1 - Allows external logic to disable the module clocks.
 */
/*@{*/
/*! @brief Read current value of the SPI_MCR_MDIS field. */
#define SPI_RD_MCR_MDIS(base) ((SPI_MCR_REG(base) & SPI_MCR_MDIS_MASK) >> SPI_MCR_MDIS_SHIFT)
#define SPI_BRD_MCR_MDIS(base) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_MDIS_SHIFT))

/*! @brief Set the MDIS field to a new value. */
#define SPI_WR_MCR_MDIS(base, value) (SPI_RMW_MCR(base, SPI_MCR_MDIS_MASK, SPI_MCR_MDIS(value)))
#define SPI_BWR_MCR_MDIS(base, value) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_MDIS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_MCR, field DOZE[15] (RW)
 *
 * Provides support for an externally controlled Doze mode power-saving
 * mechanism.
 *
 * Values:
 * - 0b0 - Doze mode has no effect on the module.
 * - 0b1 - Doze mode disables the module.
 */
/*@{*/
/*! @brief Read current value of the SPI_MCR_DOZE field. */
#define SPI_RD_MCR_DOZE(base) ((SPI_MCR_REG(base) & SPI_MCR_DOZE_MASK) >> SPI_MCR_DOZE_SHIFT)
#define SPI_BRD_MCR_DOZE(base) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_DOZE_SHIFT))

/*! @brief Set the DOZE field to a new value. */
#define SPI_WR_MCR_DOZE(base, value) (SPI_RMW_MCR(base, SPI_MCR_DOZE_MASK, SPI_MCR_DOZE(value)))
#define SPI_BWR_MCR_DOZE(base, value) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_DOZE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_MCR, field PCSIS[21:16] (RW)
 *
 * Determines the inactive state of PCSx. Refer to the chip-specific SPI
 * information for the number of PCS signals used in this MCU.
 *
 * Values:
 * - 0b000000 - The inactive state of PCSx is low.
 * - 0b000001 - The inactive state of PCSx is high.
 */
/*@{*/
/*! @brief Read current value of the SPI_MCR_PCSIS field. */
#define SPI_RD_MCR_PCSIS(base) ((SPI_MCR_REG(base) & SPI_MCR_PCSIS_MASK) >> SPI_MCR_PCSIS_SHIFT)
#define SPI_BRD_MCR_PCSIS(base) (SPI_RD_MCR_PCSIS(base))

/*! @brief Set the PCSIS field to a new value. */
#define SPI_WR_MCR_PCSIS(base, value) (SPI_RMW_MCR(base, SPI_MCR_PCSIS_MASK, SPI_MCR_PCSIS(value)))
#define SPI_BWR_MCR_PCSIS(base, value) (SPI_WR_MCR_PCSIS(base, value))
/*@}*/

/*!
 * @name Register SPI_MCR, field ROOE[24] (RW)
 *
 * In the RX FIFO overflow condition, configures the module to ignore the
 * incoming serial data or overwrite existing data. If the RX FIFO is full and new data
 * is received, the data from the transfer, generating the overflow, is ignored
 * or shifted into the shift register.
 *
 * Values:
 * - 0b0 - Incoming data is ignored.
 * - 0b1 - Incoming data is shifted into the shift register.
 */
/*@{*/
/*! @brief Read current value of the SPI_MCR_ROOE field. */
#define SPI_RD_MCR_ROOE(base) ((SPI_MCR_REG(base) & SPI_MCR_ROOE_MASK) >> SPI_MCR_ROOE_SHIFT)
#define SPI_BRD_MCR_ROOE(base) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_ROOE_SHIFT))

/*! @brief Set the ROOE field to a new value. */
#define SPI_WR_MCR_ROOE(base, value) (SPI_RMW_MCR(base, SPI_MCR_ROOE_MASK, SPI_MCR_ROOE(value)))
#define SPI_BWR_MCR_ROOE(base, value) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_ROOE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_MCR, field PCSSE[25] (RW)
 *
 * Enables the PCS5/ PCSS to operate as a PCS Strobe output signal.
 *
 * Values:
 * - 0b0 - PCS5/ PCSS is used as the Peripheral Chip Select[5] signal.
 * - 0b1 - PCS5/ PCSS is used as an active-low PCS Strobe signal.
 */
/*@{*/
/*! @brief Read current value of the SPI_MCR_PCSSE field. */
#define SPI_RD_MCR_PCSSE(base) ((SPI_MCR_REG(base) & SPI_MCR_PCSSE_MASK) >> SPI_MCR_PCSSE_SHIFT)
#define SPI_BRD_MCR_PCSSE(base) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_PCSSE_SHIFT))

/*! @brief Set the PCSSE field to a new value. */
#define SPI_WR_MCR_PCSSE(base, value) (SPI_RMW_MCR(base, SPI_MCR_PCSSE_MASK, SPI_MCR_PCSSE(value)))
#define SPI_BWR_MCR_PCSSE(base, value) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_PCSSE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_MCR, field MTFE[26] (RW)
 *
 * Enables a modified transfer format to be used.
 *
 * Values:
 * - 0b0 - Modified SPI transfer format disabled.
 * - 0b1 - Modified SPI transfer format enabled.
 */
/*@{*/
/*! @brief Read current value of the SPI_MCR_MTFE field. */
#define SPI_RD_MCR_MTFE(base) ((SPI_MCR_REG(base) & SPI_MCR_MTFE_MASK) >> SPI_MCR_MTFE_SHIFT)
#define SPI_BRD_MCR_MTFE(base) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_MTFE_SHIFT))

/*! @brief Set the MTFE field to a new value. */
#define SPI_WR_MCR_MTFE(base, value) (SPI_RMW_MCR(base, SPI_MCR_MTFE_MASK, SPI_MCR_MTFE(value)))
#define SPI_BWR_MCR_MTFE(base, value) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_MTFE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_MCR, field FRZ[27] (RW)
 *
 * Enables transfers to be stopped on the next frame boundary when the device
 * enters Debug mode.
 *
 * Values:
 * - 0b0 - Do not halt serial transfers in Debug mode.
 * - 0b1 - Halt serial transfers in Debug mode.
 */
/*@{*/
/*! @brief Read current value of the SPI_MCR_FRZ field. */
#define SPI_RD_MCR_FRZ(base) ((SPI_MCR_REG(base) & SPI_MCR_FRZ_MASK) >> SPI_MCR_FRZ_SHIFT)
#define SPI_BRD_MCR_FRZ(base) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_FRZ_SHIFT))

/*! @brief Set the FRZ field to a new value. */
#define SPI_WR_MCR_FRZ(base, value) (SPI_RMW_MCR(base, SPI_MCR_FRZ_MASK, SPI_MCR_FRZ(value)))
#define SPI_BWR_MCR_FRZ(base, value) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_FRZ_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_MCR, field DCONF[29:28] (RO)
 *
 * Selects among the different configurations of the module.
 *
 * Values:
 * - 0b00 - SPI
 * - 0b01 - Reserved
 * - 0b10 - Reserved
 * - 0b11 - Reserved
 */
/*@{*/
/*! @brief Read current value of the SPI_MCR_DCONF field. */
#define SPI_RD_MCR_DCONF(base) ((SPI_MCR_REG(base) & SPI_MCR_DCONF_MASK) >> SPI_MCR_DCONF_SHIFT)
#define SPI_BRD_MCR_DCONF(base) (SPI_RD_MCR_DCONF(base))
/*@}*/

/*!
 * @name Register SPI_MCR, field CONT_SCKE[30] (RW)
 *
 * Enables the Serial Communication Clock (SCK) to run continuously.
 *
 * Values:
 * - 0b0 - Continuous SCK disabled.
 * - 0b1 - Continuous SCK enabled.
 */
/*@{*/
/*! @brief Read current value of the SPI_MCR_CONT_SCKE field. */
#define SPI_RD_MCR_CONT_SCKE(base) ((SPI_MCR_REG(base) & SPI_MCR_CONT_SCKE_MASK) >> SPI_MCR_CONT_SCKE_SHIFT)
#define SPI_BRD_MCR_CONT_SCKE(base) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_CONT_SCKE_SHIFT))

/*! @brief Set the CONT_SCKE field to a new value. */
#define SPI_WR_MCR_CONT_SCKE(base, value) (SPI_RMW_MCR(base, SPI_MCR_CONT_SCKE_MASK, SPI_MCR_CONT_SCKE(value)))
#define SPI_BWR_MCR_CONT_SCKE(base, value) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_CONT_SCKE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_MCR, field MSTR[31] (RW)
 *
 * Enables either Master mode (if supported) or Slave mode (if supported)
 * operation.
 *
 * Values:
 * - 0b0 - Enables Slave mode
 * - 0b1 - Enables Master mode
 */
/*@{*/
/*! @brief Read current value of the SPI_MCR_MSTR field. */
#define SPI_RD_MCR_MSTR(base) ((SPI_MCR_REG(base) & SPI_MCR_MSTR_MASK) >> SPI_MCR_MSTR_SHIFT)
#define SPI_BRD_MCR_MSTR(base) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_MSTR_SHIFT))

/*! @brief Set the MSTR field to a new value. */
#define SPI_WR_MCR_MSTR(base, value) (SPI_RMW_MCR(base, SPI_MCR_MSTR_MASK, SPI_MCR_MSTR(value)))
#define SPI_BWR_MCR_MSTR(base, value) (BITBAND_ACCESS32(&SPI_MCR_REG(base), SPI_MCR_MSTR_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * SPI_TCR - Transfer Count Register
 ******************************************************************************/

/*!
 * @brief SPI_TCR - Transfer Count Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * TCR contains a counter that indicates the number of SPI transfers made. The
 * transfer counter is intended to assist in queue management. Do not write the
 * TCR when the module is in the Running state.
 */
/*!
 * @name Constants and macros for entire SPI_TCR register
 */
/*@{*/
#define SPI_RD_TCR(base)         (SPI_TCR_REG(base))
#define SPI_WR_TCR(base, value)  (SPI_TCR_REG(base) = (value))
#define SPI_RMW_TCR(base, mask, value) (SPI_WR_TCR(base, (SPI_RD_TCR(base) & ~(mask)) | (value)))
#define SPI_SET_TCR(base, value) (SPI_WR_TCR(base, SPI_RD_TCR(base) |  (value)))
#define SPI_CLR_TCR(base, value) (SPI_WR_TCR(base, SPI_RD_TCR(base) & ~(value)))
#define SPI_TOG_TCR(base, value) (SPI_WR_TCR(base, SPI_RD_TCR(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SPI_TCR bitfields
 */

/*!
 * @name Register SPI_TCR, field SPI_TCNT[31:16] (RW)
 *
 * Counts the number of SPI transfers the module makes. The SPI_TCNT field
 * increments every time the last bit of an SPI frame is transmitted. A value written
 * to SPI_TCNT presets the counter to that value. SPI_TCNT is reset to zero at
 * the beginning of the frame when the CTCNT field is set in the executing SPI
 * command. The Transfer Counter wraps around; incrementing the counter past 65535
 * resets the counter to zero.
 */
/*@{*/
/*! @brief Read current value of the SPI_TCR_SPI_TCNT field. */
#define SPI_RD_TCR_SPI_TCNT(base) ((SPI_TCR_REG(base) & SPI_TCR_SPI_TCNT_MASK) >> SPI_TCR_SPI_TCNT_SHIFT)
#define SPI_BRD_TCR_SPI_TCNT(base) (SPI_RD_TCR_SPI_TCNT(base))

/*! @brief Set the SPI_TCNT field to a new value. */
#define SPI_WR_TCR_SPI_TCNT(base, value) (SPI_RMW_TCR(base, SPI_TCR_SPI_TCNT_MASK, SPI_TCR_SPI_TCNT(value)))
#define SPI_BWR_TCR_SPI_TCNT(base, value) (SPI_WR_TCR_SPI_TCNT(base, value))
/*@}*/

/*******************************************************************************
 * SPI_CTAR_SLAVE - Clock and Transfer Attributes Register (In Slave Mode)
 ******************************************************************************/

/*!
 * @brief SPI_CTAR_SLAVE - Clock and Transfer Attributes Register (In Slave Mode) (RW)
 *
 * Reset value: 0x78000000U
 *
 * When the module is configured as an SPI bus slave, the CTAR0 register is used.
 */
/*!
 * @name Constants and macros for entire SPI_CTAR_SLAVE register
 */
/*@{*/
#define SPI_RD_CTAR_SLAVE(base, index) (SPI_CTAR_SLAVE_REG(base, index))
#define SPI_WR_CTAR_SLAVE(base, index, value) (SPI_CTAR_SLAVE_REG(base, index) = (value))
#define SPI_RMW_CTAR_SLAVE(base, index, mask, value) (SPI_WR_CTAR_SLAVE(base, index, (SPI_RD_CTAR_SLAVE(base, index) & ~(mask)) | (value)))
#define SPI_SET_CTAR_SLAVE(base, index, value) (SPI_WR_CTAR_SLAVE(base, index, SPI_RD_CTAR_SLAVE(base, index) |  (value)))
#define SPI_CLR_CTAR_SLAVE(base, index, value) (SPI_WR_CTAR_SLAVE(base, index, SPI_RD_CTAR_SLAVE(base, index) & ~(value)))
#define SPI_TOG_CTAR_SLAVE(base, index, value) (SPI_WR_CTAR_SLAVE(base, index, SPI_RD_CTAR_SLAVE(base, index) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SPI_CTAR_SLAVE bitfields
 */

/*!
 * @name Register SPI_CTAR_SLAVE, field CPHA[25] (RW)
 *
 * Selects which edge of SCK causes data to change and which edge causes data to
 * be captured. This bit is used in both master and slave mode. For successful
 * communication between serial devices, the devices must have identical clock
 * phase settings. In Continuous SCK mode, the bit value is ignored and the
 * transfers are done as if the CPHA bit is set to 1.
 *
 * Values:
 * - 0b0 - Data is captured on the leading edge of SCK and changed on the
 *     following edge.
 * - 0b1 - Data is changed on the leading edge of SCK and captured on the
 *     following edge.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_SLAVE_CPHA field. */
#define SPI_RD_CTAR_SLAVE_CPHA(base, index) ((SPI_CTAR_SLAVE_REG(base, index) & SPI_CTAR_SLAVE_CPHA_MASK) >> SPI_CTAR_SLAVE_CPHA_SHIFT)
#define SPI_BRD_CTAR_SLAVE_CPHA(base, index) (BITBAND_ACCESS32(&SPI_CTAR_SLAVE_REG(base, index), SPI_CTAR_SLAVE_CPHA_SHIFT))

/*! @brief Set the CPHA field to a new value. */
#define SPI_WR_CTAR_SLAVE_CPHA(base, index, value) (SPI_RMW_CTAR_SLAVE(base, index, SPI_CTAR_SLAVE_CPHA_MASK, SPI_CTAR_SLAVE_CPHA(value)))
#define SPI_BWR_CTAR_SLAVE_CPHA(base, index, value) (BITBAND_ACCESS32(&SPI_CTAR_SLAVE_REG(base, index), SPI_CTAR_SLAVE_CPHA_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_CTAR_SLAVE, field CPOL[26] (RW)
 *
 * Selects the inactive state of the Serial Communications Clock (SCK). In case
 * of Continuous SCK mode, when the module goes in low power mode(disabled),
 * inactive state of SCK is not guaranted.
 *
 * Values:
 * - 0b0 - The inactive state value of SCK is low.
 * - 0b1 - The inactive state value of SCK is high.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_SLAVE_CPOL field. */
#define SPI_RD_CTAR_SLAVE_CPOL(base, index) ((SPI_CTAR_SLAVE_REG(base, index) & SPI_CTAR_SLAVE_CPOL_MASK) >> SPI_CTAR_SLAVE_CPOL_SHIFT)
#define SPI_BRD_CTAR_SLAVE_CPOL(base, index) (BITBAND_ACCESS32(&SPI_CTAR_SLAVE_REG(base, index), SPI_CTAR_SLAVE_CPOL_SHIFT))

/*! @brief Set the CPOL field to a new value. */
#define SPI_WR_CTAR_SLAVE_CPOL(base, index, value) (SPI_RMW_CTAR_SLAVE(base, index, SPI_CTAR_SLAVE_CPOL_MASK, SPI_CTAR_SLAVE_CPOL(value)))
#define SPI_BWR_CTAR_SLAVE_CPOL(base, index, value) (BITBAND_ACCESS32(&SPI_CTAR_SLAVE_REG(base, index), SPI_CTAR_SLAVE_CPOL_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_CTAR_SLAVE, field FMSZ[30:27] (RW)
 *
 * The number of bits transfered per frame is equal to the FMSZ field value plus
 * 1. Note that the minimum valid value of frame size is 4.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_SLAVE_FMSZ field. */
#define SPI_RD_CTAR_SLAVE_FMSZ(base, index) ((SPI_CTAR_SLAVE_REG(base, index) & SPI_CTAR_SLAVE_FMSZ_MASK) >> SPI_CTAR_SLAVE_FMSZ_SHIFT)
#define SPI_BRD_CTAR_SLAVE_FMSZ(base, index) (SPI_RD_CTAR_SLAVE_FMSZ(base, index))

/*! @brief Set the FMSZ field to a new value. */
#define SPI_WR_CTAR_SLAVE_FMSZ(base, index, value) (SPI_RMW_CTAR_SLAVE(base, index, SPI_CTAR_SLAVE_FMSZ_MASK, SPI_CTAR_SLAVE_FMSZ(value)))
#define SPI_BWR_CTAR_SLAVE_FMSZ(base, index, value) (SPI_WR_CTAR_SLAVE_FMSZ(base, index, value))
/*@}*/

/*******************************************************************************
 * SPI_CTAR - Clock and Transfer Attributes Register (In Master Mode)
 ******************************************************************************/

/*!
 * @brief SPI_CTAR - Clock and Transfer Attributes Register (In Master Mode) (RW)
 *
 * Reset value: 0x78000000U
 *
 * CTAR registers are used to define different transfer attributes. Do not write
 * to the CTAR registers while the module is in the Running state. In Master
 * mode, the CTAR registers define combinations of transfer attributes such as frame
 * size, clock phase and polarity, data bit ordering, baud rate, and various
 * delays. In slave mode, a subset of the bitfields in CTAR0 are used to set the
 * slave transfer attributes. When the module is configured as an SPI master, the
 * CTAS field in the command portion of the TX FIFO entry selects which of the CTAR
 * registers is used. When the module is configured as an SPI bus slave, it uses
 * the CTAR0 register.
 */
/*!
 * @name Constants and macros for entire SPI_CTAR register
 */
/*@{*/
#define SPI_RD_CTAR(base, index) (SPI_CTAR_REG(base, index))
#define SPI_WR_CTAR(base, index, value) (SPI_CTAR_REG(base, index) = (value))
#define SPI_RMW_CTAR(base, index, mask, value) (SPI_WR_CTAR(base, index, (SPI_RD_CTAR(base, index) & ~(mask)) | (value)))
#define SPI_SET_CTAR(base, index, value) (SPI_WR_CTAR(base, index, SPI_RD_CTAR(base, index) |  (value)))
#define SPI_CLR_CTAR(base, index, value) (SPI_WR_CTAR(base, index, SPI_RD_CTAR(base, index) & ~(value)))
#define SPI_TOG_CTAR(base, index, value) (SPI_WR_CTAR(base, index, SPI_RD_CTAR(base, index) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SPI_CTAR bitfields
 */

/*!
 * @name Register SPI_CTAR, field BR[3:0] (RW)
 *
 * Selects the scaler value for the baud rate. This field is used only in master
 * mode. The prescaled protocol clock is divided by the Baud Rate Scaler to
 * generate the frequency of the SCK. The baud rate is computed according to the
 * following equation: SCK baud rate = (fP /PBR) x [(1+DBR)/BR] The following table
 * lists the baud rate scaler values. Baud Rate Scaler CTARn[BR] Baud Rate Scaler
 * Value 0000 2 0001 4 0010 6 0011 8 0100 16 0101 32 0110 64 0111 128 1000 256
 * 1001 512 1010 1024 1011 2048 1100 4096 1101 8192 1110 16384 1111 32768
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_BR field. */
#define SPI_RD_CTAR_BR(base, index) ((SPI_CTAR_REG(base, index) & SPI_CTAR_BR_MASK) >> SPI_CTAR_BR_SHIFT)
#define SPI_BRD_CTAR_BR(base, index) (SPI_RD_CTAR_BR(base, index))

/*! @brief Set the BR field to a new value. */
#define SPI_WR_CTAR_BR(base, index, value) (SPI_RMW_CTAR(base, index, SPI_CTAR_BR_MASK, SPI_CTAR_BR(value)))
#define SPI_BWR_CTAR_BR(base, index, value) (SPI_WR_CTAR_BR(base, index, value))
/*@}*/

/*!
 * @name Register SPI_CTAR, field DT[7:4] (RW)
 *
 * Selects the Delay after Transfer Scaler. This field is used only in master
 * mode. The Delay after Transfer is the time between the negation of the PCS
 * signal at the end of a frame and the assertion of PCS at the beginning of the next
 * frame. In the Continuous Serial Communications Clock operation, the DT value
 * is fixed to one SCK clock period, The Delay after Transfer is a multiple of the
 * protocol clock period, and it is computed according to the following
 * equation: tDT = (1/fP ) x PDT x DT See Delay Scaler Encoding table in CTARn[CSSCK] bit
 * field description for scaler values.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_DT field. */
#define SPI_RD_CTAR_DT(base, index) ((SPI_CTAR_REG(base, index) & SPI_CTAR_DT_MASK) >> SPI_CTAR_DT_SHIFT)
#define SPI_BRD_CTAR_DT(base, index) (SPI_RD_CTAR_DT(base, index))

/*! @brief Set the DT field to a new value. */
#define SPI_WR_CTAR_DT(base, index, value) (SPI_RMW_CTAR(base, index, SPI_CTAR_DT_MASK, SPI_CTAR_DT(value)))
#define SPI_BWR_CTAR_DT(base, index, value) (SPI_WR_CTAR_DT(base, index, value))
/*@}*/

/*!
 * @name Register SPI_CTAR, field ASC[11:8] (RW)
 *
 * Selects the scaler value for the After SCK Delay. This field is used only in
 * master mode. The After SCK Delay is the delay between the last edge of SCK and
 * the negation of PCS. The delay is a multiple of the protocol clock period,
 * and it is computed according to the following equation: t ASC = (1/fP) x PASC x
 * ASC See Delay Scaler Encoding table in CTARn[CSSCK] bit field description for
 * scaler values. Refer After SCK Delay (tASC ) for more details.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_ASC field. */
#define SPI_RD_CTAR_ASC(base, index) ((SPI_CTAR_REG(base, index) & SPI_CTAR_ASC_MASK) >> SPI_CTAR_ASC_SHIFT)
#define SPI_BRD_CTAR_ASC(base, index) (SPI_RD_CTAR_ASC(base, index))

/*! @brief Set the ASC field to a new value. */
#define SPI_WR_CTAR_ASC(base, index, value) (SPI_RMW_CTAR(base, index, SPI_CTAR_ASC_MASK, SPI_CTAR_ASC(value)))
#define SPI_BWR_CTAR_ASC(base, index, value) (SPI_WR_CTAR_ASC(base, index, value))
/*@}*/

/*!
 * @name Register SPI_CTAR, field CSSCK[15:12] (RW)
 *
 * Selects the scaler value for the PCS to SCK delay. This field is used only in
 * master mode. The PCS to SCK Delay is the delay between the assertion of PCS
 * and the first edge of the SCK. The delay is a multiple of the protocol clock
 * period, and it is computed according to the following equation: t CSC = (1/fP )
 * x PCSSCK x CSSCK. The following table lists the delay scaler values. Delay
 * Scaler Encoding Field Value Delay Scaler Value 0000 2 0001 4 0010 8 0011 16 0100
 * 32 0101 64 0110 128 0111 256 1000 512 1001 1024 1010 2048 1011 4096 1100 8192
 * 1101 16384 1110 32768 1111 65536 Refer PCS to SCK Delay (tCSC ) for more
 * details.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_CSSCK field. */
#define SPI_RD_CTAR_CSSCK(base, index) ((SPI_CTAR_REG(base, index) & SPI_CTAR_CSSCK_MASK) >> SPI_CTAR_CSSCK_SHIFT)
#define SPI_BRD_CTAR_CSSCK(base, index) (SPI_RD_CTAR_CSSCK(base, index))

/*! @brief Set the CSSCK field to a new value. */
#define SPI_WR_CTAR_CSSCK(base, index, value) (SPI_RMW_CTAR(base, index, SPI_CTAR_CSSCK_MASK, SPI_CTAR_CSSCK(value)))
#define SPI_BWR_CTAR_CSSCK(base, index, value) (SPI_WR_CTAR_CSSCK(base, index, value))
/*@}*/

/*!
 * @name Register SPI_CTAR, field PBR[17:16] (RW)
 *
 * Selects the prescaler value for the baud rate. This field is used only in
 * master mode. The baud rate is the frequency of the SCK. The protocol clock is
 * divided by the prescaler value before the baud rate selection takes place. See
 * the BR field description for details on how to compute the baud rate.
 *
 * Values:
 * - 0b00 - Baud Rate Prescaler value is 2.
 * - 0b01 - Baud Rate Prescaler value is 3.
 * - 0b10 - Baud Rate Prescaler value is 5.
 * - 0b11 - Baud Rate Prescaler value is 7.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_PBR field. */
#define SPI_RD_CTAR_PBR(base, index) ((SPI_CTAR_REG(base, index) & SPI_CTAR_PBR_MASK) >> SPI_CTAR_PBR_SHIFT)
#define SPI_BRD_CTAR_PBR(base, index) (SPI_RD_CTAR_PBR(base, index))

/*! @brief Set the PBR field to a new value. */
#define SPI_WR_CTAR_PBR(base, index, value) (SPI_RMW_CTAR(base, index, SPI_CTAR_PBR_MASK, SPI_CTAR_PBR(value)))
#define SPI_BWR_CTAR_PBR(base, index, value) (SPI_WR_CTAR_PBR(base, index, value))
/*@}*/

/*!
 * @name Register SPI_CTAR, field PDT[19:18] (RW)
 *
 * Selects the prescaler value for the delay between the negation of the PCS
 * signal at the end of a frame and the assertion of PCS at the beginning of the
 * next frame. The PDT field is only used in master mode. See the DT field
 * description for details on how to compute the Delay after Transfer. Refer Delay after
 * Transfer (tDT ) for more details.
 *
 * Values:
 * - 0b00 - Delay after Transfer Prescaler value is 1.
 * - 0b01 - Delay after Transfer Prescaler value is 3.
 * - 0b10 - Delay after Transfer Prescaler value is 5.
 * - 0b11 - Delay after Transfer Prescaler value is 7.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_PDT field. */
#define SPI_RD_CTAR_PDT(base, index) ((SPI_CTAR_REG(base, index) & SPI_CTAR_PDT_MASK) >> SPI_CTAR_PDT_SHIFT)
#define SPI_BRD_CTAR_PDT(base, index) (SPI_RD_CTAR_PDT(base, index))

/*! @brief Set the PDT field to a new value. */
#define SPI_WR_CTAR_PDT(base, index, value) (SPI_RMW_CTAR(base, index, SPI_CTAR_PDT_MASK, SPI_CTAR_PDT(value)))
#define SPI_BWR_CTAR_PDT(base, index, value) (SPI_WR_CTAR_PDT(base, index, value))
/*@}*/

/*!
 * @name Register SPI_CTAR, field PASC[21:20] (RW)
 *
 * Selects the prescaler value for the delay between the last edge of SCK and
 * the negation of PCS. See the ASC field description for information on how to
 * compute the After SCK Delay. Refer After SCK Delay (tASC ) for more details.
 *
 * Values:
 * - 0b00 - Delay after Transfer Prescaler value is 1.
 * - 0b01 - Delay after Transfer Prescaler value is 3.
 * - 0b10 - Delay after Transfer Prescaler value is 5.
 * - 0b11 - Delay after Transfer Prescaler value is 7.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_PASC field. */
#define SPI_RD_CTAR_PASC(base, index) ((SPI_CTAR_REG(base, index) & SPI_CTAR_PASC_MASK) >> SPI_CTAR_PASC_SHIFT)
#define SPI_BRD_CTAR_PASC(base, index) (SPI_RD_CTAR_PASC(base, index))

/*! @brief Set the PASC field to a new value. */
#define SPI_WR_CTAR_PASC(base, index, value) (SPI_RMW_CTAR(base, index, SPI_CTAR_PASC_MASK, SPI_CTAR_PASC(value)))
#define SPI_BWR_CTAR_PASC(base, index, value) (SPI_WR_CTAR_PASC(base, index, value))
/*@}*/

/*!
 * @name Register SPI_CTAR, field PCSSCK[23:22] (RW)
 *
 * Selects the prescaler value for the delay between assertion of PCS and the
 * first edge of the SCK. See the CSSCK field description for information on how to
 * compute the PCS to SCK Delay. Refer PCS to SCK Delay (tCSC ) for more details.
 *
 * Values:
 * - 0b00 - PCS to SCK Prescaler value is 1.
 * - 0b01 - PCS to SCK Prescaler value is 3.
 * - 0b10 - PCS to SCK Prescaler value is 5.
 * - 0b11 - PCS to SCK Prescaler value is 7.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_PCSSCK field. */
#define SPI_RD_CTAR_PCSSCK(base, index) ((SPI_CTAR_REG(base, index) & SPI_CTAR_PCSSCK_MASK) >> SPI_CTAR_PCSSCK_SHIFT)
#define SPI_BRD_CTAR_PCSSCK(base, index) (SPI_RD_CTAR_PCSSCK(base, index))

/*! @brief Set the PCSSCK field to a new value. */
#define SPI_WR_CTAR_PCSSCK(base, index, value) (SPI_RMW_CTAR(base, index, SPI_CTAR_PCSSCK_MASK, SPI_CTAR_PCSSCK(value)))
#define SPI_BWR_CTAR_PCSSCK(base, index, value) (SPI_WR_CTAR_PCSSCK(base, index, value))
/*@}*/

/*!
 * @name Register SPI_CTAR, field LSBFE[24] (RW)
 *
 * Specifies whether the LSB or MSB of the frame is transferred first.
 *
 * Values:
 * - 0b0 - Data is transferred MSB first.
 * - 0b1 - Data is transferred LSB first.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_LSBFE field. */
#define SPI_RD_CTAR_LSBFE(base, index) ((SPI_CTAR_REG(base, index) & SPI_CTAR_LSBFE_MASK) >> SPI_CTAR_LSBFE_SHIFT)
#define SPI_BRD_CTAR_LSBFE(base, index) (BITBAND_ACCESS32(&SPI_CTAR_REG(base, index), SPI_CTAR_LSBFE_SHIFT))

/*! @brief Set the LSBFE field to a new value. */
#define SPI_WR_CTAR_LSBFE(base, index, value) (SPI_RMW_CTAR(base, index, SPI_CTAR_LSBFE_MASK, SPI_CTAR_LSBFE(value)))
#define SPI_BWR_CTAR_LSBFE(base, index, value) (BITBAND_ACCESS32(&SPI_CTAR_REG(base, index), SPI_CTAR_LSBFE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_CTAR, field CPHA[25] (RW)
 *
 * Selects which edge of SCK causes data to change and which edge causes data to
 * be captured. This bit is used in both master and slave mode. For successful
 * communication between serial devices, the devices must have identical clock
 * phase settings. In Continuous SCK mode, the bit value is ignored and the
 * transfers are done as if the CPHA bit is set to 1.
 *
 * Values:
 * - 0b0 - Data is captured on the leading edge of SCK and changed on the
 *     following edge.
 * - 0b1 - Data is changed on the leading edge of SCK and captured on the
 *     following edge.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_CPHA field. */
#define SPI_RD_CTAR_CPHA(base, index) ((SPI_CTAR_REG(base, index) & SPI_CTAR_CPHA_MASK) >> SPI_CTAR_CPHA_SHIFT)
#define SPI_BRD_CTAR_CPHA(base, index) (BITBAND_ACCESS32(&SPI_CTAR_REG(base, index), SPI_CTAR_CPHA_SHIFT))

/*! @brief Set the CPHA field to a new value. */
#define SPI_WR_CTAR_CPHA(base, index, value) (SPI_RMW_CTAR(base, index, SPI_CTAR_CPHA_MASK, SPI_CTAR_CPHA(value)))
#define SPI_BWR_CTAR_CPHA(base, index, value) (BITBAND_ACCESS32(&SPI_CTAR_REG(base, index), SPI_CTAR_CPHA_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_CTAR, field CPOL[26] (RW)
 *
 * Selects the inactive state of the Serial Communications Clock (SCK). This bit
 * is used in both master and slave mode. For successful communication between
 * serial devices, the devices must have identical clock polarities. When the
 * Continuous Selection Format is selected, switching between clock polarities
 * without stopping the module can cause errors in the transfer due to the peripheral
 * device interpreting the switch of clock polarity as a valid clock edge. In case
 * of Continuous SCK mode, when the module goes in low power mode(disabled),
 * inactive state of SCK is not guaranted.
 *
 * Values:
 * - 0b0 - The inactive state value of SCK is low.
 * - 0b1 - The inactive state value of SCK is high.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_CPOL field. */
#define SPI_RD_CTAR_CPOL(base, index) ((SPI_CTAR_REG(base, index) & SPI_CTAR_CPOL_MASK) >> SPI_CTAR_CPOL_SHIFT)
#define SPI_BRD_CTAR_CPOL(base, index) (BITBAND_ACCESS32(&SPI_CTAR_REG(base, index), SPI_CTAR_CPOL_SHIFT))

/*! @brief Set the CPOL field to a new value. */
#define SPI_WR_CTAR_CPOL(base, index, value) (SPI_RMW_CTAR(base, index, SPI_CTAR_CPOL_MASK, SPI_CTAR_CPOL(value)))
#define SPI_BWR_CTAR_CPOL(base, index, value) (BITBAND_ACCESS32(&SPI_CTAR_REG(base, index), SPI_CTAR_CPOL_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_CTAR, field FMSZ[30:27] (RW)
 *
 * The number of bits transferred per frame is equal to the FMSZ value plus 1.
 * Regardless of the transmission mode, the minimum valid frame size value is 4.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_FMSZ field. */
#define SPI_RD_CTAR_FMSZ(base, index) ((SPI_CTAR_REG(base, index) & SPI_CTAR_FMSZ_MASK) >> SPI_CTAR_FMSZ_SHIFT)
#define SPI_BRD_CTAR_FMSZ(base, index) (SPI_RD_CTAR_FMSZ(base, index))

/*! @brief Set the FMSZ field to a new value. */
#define SPI_WR_CTAR_FMSZ(base, index, value) (SPI_RMW_CTAR(base, index, SPI_CTAR_FMSZ_MASK, SPI_CTAR_FMSZ(value)))
#define SPI_BWR_CTAR_FMSZ(base, index, value) (SPI_WR_CTAR_FMSZ(base, index, value))
/*@}*/

/*!
 * @name Register SPI_CTAR, field DBR[31] (RW)
 *
 * Doubles the effective baud rate of the Serial Communications Clock (SCK).
 * This field is used only in master mode. It effectively halves the Baud Rate
 * division ratio, supporting faster frequencies, and odd division ratios for the
 * Serial Communications Clock (SCK). When the DBR bit is set, the duty cycle of the
 * Serial Communications Clock (SCK) depends on the value in the Baud Rate
 * Prescaler and the Clock Phase bit as listed in the following table. See the BR field
 * description for details on how to compute the baud rate. SPI SCK Duty Cycle
 * DBR CPHA PBR SCK Duty Cycle 0 any any 50/50 1 0 00 50/50 1 0 01 33/66 1 0 10
 * 40/60 1 0 11 43/57 1 1 00 50/50 1 1 01 66/33 1 1 10 60/40 1 1 11 57/43
 *
 * Values:
 * - 0b0 - The baud rate is computed normally with a 50/50 duty cycle.
 * - 0b1 - The baud rate is doubled with the duty cycle depending on the Baud
 *     Rate Prescaler.
 */
/*@{*/
/*! @brief Read current value of the SPI_CTAR_DBR field. */
#define SPI_RD_CTAR_DBR(base, index) ((SPI_CTAR_REG(base, index) & SPI_CTAR_DBR_MASK) >> SPI_CTAR_DBR_SHIFT)
#define SPI_BRD_CTAR_DBR(base, index) (BITBAND_ACCESS32(&SPI_CTAR_REG(base, index), SPI_CTAR_DBR_SHIFT))

/*! @brief Set the DBR field to a new value. */
#define SPI_WR_CTAR_DBR(base, index, value) (SPI_RMW_CTAR(base, index, SPI_CTAR_DBR_MASK, SPI_CTAR_DBR(value)))
#define SPI_BWR_CTAR_DBR(base, index, value) (BITBAND_ACCESS32(&SPI_CTAR_REG(base, index), SPI_CTAR_DBR_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * SPI_SR - Status Register
 ******************************************************************************/

/*!
 * @brief SPI_SR - Status Register (RW)
 *
 * Reset value: 0x02000000U
 *
 * SR contains status and flag bits. The bits reflect the status of the module
 * and indicate the occurrence of events that can generate interrupt or DMA
 * requests. Software can clear flag bits in the SR by writing a 1 to them. Writing a 0
 * to a flag bit has no effect. This register may not be writable in Module
 * Disable mode due to the use of power saving mechanisms.
 */
/*!
 * @name Constants and macros for entire SPI_SR register
 */
/*@{*/
#define SPI_RD_SR(base)          (SPI_SR_REG(base))
#define SPI_WR_SR(base, value)   (SPI_SR_REG(base) = (value))
#define SPI_RMW_SR(base, mask, value) (SPI_WR_SR(base, (SPI_RD_SR(base) & ~(mask)) | (value)))
#define SPI_SET_SR(base, value)  (SPI_WR_SR(base, SPI_RD_SR(base) |  (value)))
#define SPI_CLR_SR(base, value)  (SPI_WR_SR(base, SPI_RD_SR(base) & ~(value)))
#define SPI_TOG_SR(base, value)  (SPI_WR_SR(base, SPI_RD_SR(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SPI_SR bitfields
 */

/*!
 * @name Register SPI_SR, field POPNXTPTR[3:0] (RO)
 *
 * Contains a pointer to the RX FIFO entry to be returned when the POPR is read.
 * The POPNXTPTR is updated when the POPR is read.
 */
/*@{*/
/*! @brief Read current value of the SPI_SR_POPNXTPTR field. */
#define SPI_RD_SR_POPNXTPTR(base) ((SPI_SR_REG(base) & SPI_SR_POPNXTPTR_MASK) >> SPI_SR_POPNXTPTR_SHIFT)
#define SPI_BRD_SR_POPNXTPTR(base) (SPI_RD_SR_POPNXTPTR(base))
/*@}*/

/*!
 * @name Register SPI_SR, field RXCTR[7:4] (RO)
 *
 * Indicates the number of entries in the RX FIFO. The RXCTR is decremented
 * every time the POPR is read. The RXCTR is incremented every time data is
 * transferred from the shift register to the RX FIFO.
 */
/*@{*/
/*! @brief Read current value of the SPI_SR_RXCTR field. */
#define SPI_RD_SR_RXCTR(base) ((SPI_SR_REG(base) & SPI_SR_RXCTR_MASK) >> SPI_SR_RXCTR_SHIFT)
#define SPI_BRD_SR_RXCTR(base) (SPI_RD_SR_RXCTR(base))
/*@}*/

/*!
 * @name Register SPI_SR, field TXNXTPTR[11:8] (RO)
 *
 * Indicates which TX FIFO entry is transmitted during the next transfer. The
 * TXNXTPTR field is updated every time SPI data is transferred from the TX FIFO to
 * the shift register.
 */
/*@{*/
/*! @brief Read current value of the SPI_SR_TXNXTPTR field. */
#define SPI_RD_SR_TXNXTPTR(base) ((SPI_SR_REG(base) & SPI_SR_TXNXTPTR_MASK) >> SPI_SR_TXNXTPTR_SHIFT)
#define SPI_BRD_SR_TXNXTPTR(base) (SPI_RD_SR_TXNXTPTR(base))
/*@}*/

/*!
 * @name Register SPI_SR, field TXCTR[15:12] (RO)
 *
 * Indicates the number of valid entries in the TX FIFO. The TXCTR is
 * incremented every time the PUSHR is written. The TXCTR is decremented every time an SPI
 * command is executed and the SPI data is transferred to the shift register.
 */
/*@{*/
/*! @brief Read current value of the SPI_SR_TXCTR field. */
#define SPI_RD_SR_TXCTR(base) ((SPI_SR_REG(base) & SPI_SR_TXCTR_MASK) >> SPI_SR_TXCTR_SHIFT)
#define SPI_BRD_SR_TXCTR(base) (SPI_RD_SR_TXCTR(base))
/*@}*/

/*!
 * @name Register SPI_SR, field RFDF[17] (W1C)
 *
 * Provides a method for the module to request that entries be removed from the
 * RX FIFO. The bit is set while the RX FIFO is not empty. The RFDF bit can be
 * cleared by writing 1 to it or by acknowledgement from the DMA controller when
 * the RX FIFO is empty.
 *
 * Values:
 * - 0b0 - RX FIFO is empty.
 * - 0b1 - RX FIFO is not empty.
 */
/*@{*/
/*! @brief Read current value of the SPI_SR_RFDF field. */
#define SPI_RD_SR_RFDF(base) ((SPI_SR_REG(base) & SPI_SR_RFDF_MASK) >> SPI_SR_RFDF_SHIFT)
#define SPI_BRD_SR_RFDF(base) (BITBAND_ACCESS32(&SPI_SR_REG(base), SPI_SR_RFDF_SHIFT))

/*! @brief Set the RFDF field to a new value. */
#define SPI_WR_SR_RFDF(base, value) (SPI_RMW_SR(base, (SPI_SR_RFDF_MASK | SPI_SR_RFOF_MASK | SPI_SR_TFFF_MASK | SPI_SR_TFUF_MASK | SPI_SR_EOQF_MASK | SPI_SR_TXRXS_MASK | SPI_SR_TCF_MASK), SPI_SR_RFDF(value)))
#define SPI_BWR_SR_RFDF(base, value) (BITBAND_ACCESS32(&SPI_SR_REG(base), SPI_SR_RFDF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_SR, field RFOF[19] (W1C)
 *
 * Indicates an overflow condition in the RX FIFO. The field is set when the RX
 * FIFO and shift register are full and a transfer is initiated. The bit remains
 * set until it is cleared by writing a 1 to it.
 *
 * Values:
 * - 0b0 - No Rx FIFO overflow.
 * - 0b1 - Rx FIFO overflow has occurred.
 */
/*@{*/
/*! @brief Read current value of the SPI_SR_RFOF field. */
#define SPI_RD_SR_RFOF(base) ((SPI_SR_REG(base) & SPI_SR_RFOF_MASK) >> SPI_SR_RFOF_SHIFT)
#define SPI_BRD_SR_RFOF(base) (BITBAND_ACCESS32(&SPI_SR_REG(base), SPI_SR_RFOF_SHIFT))

/*! @brief Set the RFOF field to a new value. */
#define SPI_WR_SR_RFOF(base, value) (SPI_RMW_SR(base, (SPI_SR_RFOF_MASK | SPI_SR_RFDF_MASK | SPI_SR_TFFF_MASK | SPI_SR_TFUF_MASK | SPI_SR_EOQF_MASK | SPI_SR_TXRXS_MASK | SPI_SR_TCF_MASK), SPI_SR_RFOF(value)))
#define SPI_BWR_SR_RFOF(base, value) (BITBAND_ACCESS32(&SPI_SR_REG(base), SPI_SR_RFOF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_SR, field TFFF[25] (W1C)
 *
 * Provides a method for the module to request more entries to be added to the
 * TX FIFO. The TFFF bit is set while the TX FIFO is not full. The TFFF bit can be
 * cleared by writing 1 to it or by acknowledgement from the DMA controller to
 * the TX FIFO full request.
 *
 * Values:
 * - 0b0 - TX FIFO is full.
 * - 0b1 - TX FIFO is not full.
 */
/*@{*/
/*! @brief Read current value of the SPI_SR_TFFF field. */
#define SPI_RD_SR_TFFF(base) ((SPI_SR_REG(base) & SPI_SR_TFFF_MASK) >> SPI_SR_TFFF_SHIFT)
#define SPI_BRD_SR_TFFF(base) (BITBAND_ACCESS32(&SPI_SR_REG(base), SPI_SR_TFFF_SHIFT))

/*! @brief Set the TFFF field to a new value. */
#define SPI_WR_SR_TFFF(base, value) (SPI_RMW_SR(base, (SPI_SR_TFFF_MASK | SPI_SR_RFDF_MASK | SPI_SR_RFOF_MASK | SPI_SR_TFUF_MASK | SPI_SR_EOQF_MASK | SPI_SR_TXRXS_MASK | SPI_SR_TCF_MASK), SPI_SR_TFFF(value)))
#define SPI_BWR_SR_TFFF(base, value) (BITBAND_ACCESS32(&SPI_SR_REG(base), SPI_SR_TFFF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_SR, field TFUF[27] (W1C)
 *
 * Indicates an underflow condition in the TX FIFO. The transmit underflow
 * condition is detected only for SPI blocks operating in Slave mode and SPI
 * configuration. TFUF is set when the TX FIFO of the module operating in SPI Slave mode
 * is empty and an external SPI master initiates a transfer. The TFUF bit remains
 * set until cleared by writing 1 to it.
 *
 * Values:
 * - 0b0 - No TX FIFO underflow.
 * - 0b1 - TX FIFO underflow has occurred.
 */
/*@{*/
/*! @brief Read current value of the SPI_SR_TFUF field. */
#define SPI_RD_SR_TFUF(base) ((SPI_SR_REG(base) & SPI_SR_TFUF_MASK) >> SPI_SR_TFUF_SHIFT)
#define SPI_BRD_SR_TFUF(base) (BITBAND_ACCESS32(&SPI_SR_REG(base), SPI_SR_TFUF_SHIFT))

/*! @brief Set the TFUF field to a new value. */
#define SPI_WR_SR_TFUF(base, value) (SPI_RMW_SR(base, (SPI_SR_TFUF_MASK | SPI_SR_RFDF_MASK | SPI_SR_RFOF_MASK | SPI_SR_TFFF_MASK | SPI_SR_EOQF_MASK | SPI_SR_TXRXS_MASK | SPI_SR_TCF_MASK), SPI_SR_TFUF(value)))
#define SPI_BWR_SR_TFUF(base, value) (BITBAND_ACCESS32(&SPI_SR_REG(base), SPI_SR_TFUF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_SR, field EOQF[28] (W1C)
 *
 * Indicates that the last entry in a queue has been transmitted when the module
 * is in Master mode. The EOQF bit is set when the TX FIFO entry has the EOQ bit
 * set in the command halfword and the end of the transfer is reached. The EOQF
 * bit remains set until cleared by writing a 1 to it. When the EOQF bit is set,
 * the TXRXS bit is automatically cleared.
 *
 * Values:
 * - 0b0 - EOQ is not set in the executing command.
 * - 0b1 - EOQ is set in the executing SPI command.
 */
/*@{*/
/*! @brief Read current value of the SPI_SR_EOQF field. */
#define SPI_RD_SR_EOQF(base) ((SPI_SR_REG(base) & SPI_SR_EOQF_MASK) >> SPI_SR_EOQF_SHIFT)
#define SPI_BRD_SR_EOQF(base) (BITBAND_ACCESS32(&SPI_SR_REG(base), SPI_SR_EOQF_SHIFT))

/*! @brief Set the EOQF field to a new value. */
#define SPI_WR_SR_EOQF(base, value) (SPI_RMW_SR(base, (SPI_SR_EOQF_MASK | SPI_SR_RFDF_MASK | SPI_SR_RFOF_MASK | SPI_SR_TFFF_MASK | SPI_SR_TFUF_MASK | SPI_SR_TXRXS_MASK | SPI_SR_TCF_MASK), SPI_SR_EOQF(value)))
#define SPI_BWR_SR_EOQF(base, value) (BITBAND_ACCESS32(&SPI_SR_REG(base), SPI_SR_EOQF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_SR, field TXRXS[30] (W1C)
 *
 * Reflects the run status of the module.
 *
 * Values:
 * - 0b0 - Transmit and receive operations are disabled (The module is in
 *     Stopped state).
 * - 0b1 - Transmit and receive operations are enabled (The module is in Running
 *     state).
 */
/*@{*/
/*! @brief Read current value of the SPI_SR_TXRXS field. */
#define SPI_RD_SR_TXRXS(base) ((SPI_SR_REG(base) & SPI_SR_TXRXS_MASK) >> SPI_SR_TXRXS_SHIFT)
#define SPI_BRD_SR_TXRXS(base) (BITBAND_ACCESS32(&SPI_SR_REG(base), SPI_SR_TXRXS_SHIFT))

/*! @brief Set the TXRXS field to a new value. */
#define SPI_WR_SR_TXRXS(base, value) (SPI_RMW_SR(base, (SPI_SR_TXRXS_MASK | SPI_SR_RFDF_MASK | SPI_SR_RFOF_MASK | SPI_SR_TFFF_MASK | SPI_SR_TFUF_MASK | SPI_SR_EOQF_MASK | SPI_SR_TCF_MASK), SPI_SR_TXRXS(value)))
#define SPI_BWR_SR_TXRXS(base, value) (BITBAND_ACCESS32(&SPI_SR_REG(base), SPI_SR_TXRXS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_SR, field TCF[31] (W1C)
 *
 * Indicates that all bits in a frame have been shifted out. TCF remains set
 * until it is cleared by writing a 1 to it.
 *
 * Values:
 * - 0b0 - Transfer not complete.
 * - 0b1 - Transfer complete.
 */
/*@{*/
/*! @brief Read current value of the SPI_SR_TCF field. */
#define SPI_RD_SR_TCF(base)  ((SPI_SR_REG(base) & SPI_SR_TCF_MASK) >> SPI_SR_TCF_SHIFT)
#define SPI_BRD_SR_TCF(base) (BITBAND_ACCESS32(&SPI_SR_REG(base), SPI_SR_TCF_SHIFT))

/*! @brief Set the TCF field to a new value. */
#define SPI_WR_SR_TCF(base, value) (SPI_RMW_SR(base, (SPI_SR_TCF_MASK | SPI_SR_RFDF_MASK | SPI_SR_RFOF_MASK | SPI_SR_TFFF_MASK | SPI_SR_TFUF_MASK | SPI_SR_EOQF_MASK | SPI_SR_TXRXS_MASK), SPI_SR_TCF(value)))
#define SPI_BWR_SR_TCF(base, value) (BITBAND_ACCESS32(&SPI_SR_REG(base), SPI_SR_TCF_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * SPI_RSER - DMA/Interrupt Request Select and Enable Register
 ******************************************************************************/

/*!
 * @brief SPI_RSER - DMA/Interrupt Request Select and Enable Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * RSER controls DMA and interrupt requests. Do not write to the RSER while the
 * module is in the Running state.
 */
/*!
 * @name Constants and macros for entire SPI_RSER register
 */
/*@{*/
#define SPI_RD_RSER(base)        (SPI_RSER_REG(base))
#define SPI_WR_RSER(base, value) (SPI_RSER_REG(base) = (value))
#define SPI_RMW_RSER(base, mask, value) (SPI_WR_RSER(base, (SPI_RD_RSER(base) & ~(mask)) | (value)))
#define SPI_SET_RSER(base, value) (SPI_WR_RSER(base, SPI_RD_RSER(base) |  (value)))
#define SPI_CLR_RSER(base, value) (SPI_WR_RSER(base, SPI_RD_RSER(base) & ~(value)))
#define SPI_TOG_RSER(base, value) (SPI_WR_RSER(base, SPI_RD_RSER(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SPI_RSER bitfields
 */

/*!
 * @name Register SPI_RSER, field RFDF_DIRS[16] (RW)
 *
 * Selects between generating a DMA request or an interrupt request. When the
 * RFDF flag bit in the SR is set, and the RFDF_RE bit in the RSER is set, the
 * RFDF_DIRS bit selects between generating an interrupt request or a DMA request.
 *
 * Values:
 * - 0b0 - Interrupt request.
 * - 0b1 - DMA request.
 */
/*@{*/
/*! @brief Read current value of the SPI_RSER_RFDF_DIRS field. */
#define SPI_RD_RSER_RFDF_DIRS(base) ((SPI_RSER_REG(base) & SPI_RSER_RFDF_DIRS_MASK) >> SPI_RSER_RFDF_DIRS_SHIFT)
#define SPI_BRD_RSER_RFDF_DIRS(base) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_RFDF_DIRS_SHIFT))

/*! @brief Set the RFDF_DIRS field to a new value. */
#define SPI_WR_RSER_RFDF_DIRS(base, value) (SPI_RMW_RSER(base, SPI_RSER_RFDF_DIRS_MASK, SPI_RSER_RFDF_DIRS(value)))
#define SPI_BWR_RSER_RFDF_DIRS(base, value) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_RFDF_DIRS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_RSER, field RFDF_RE[17] (RW)
 *
 * Enables the RFDF flag in the SR to generate a request. The RFDF_DIRS bit
 * selects between generating an interrupt request or a DMA request.
 *
 * Values:
 * - 0b0 - RFDF interrupt or DMA requests are disabled.
 * - 0b1 - RFDF interrupt or DMA requests are enabled.
 */
/*@{*/
/*! @brief Read current value of the SPI_RSER_RFDF_RE field. */
#define SPI_RD_RSER_RFDF_RE(base) ((SPI_RSER_REG(base) & SPI_RSER_RFDF_RE_MASK) >> SPI_RSER_RFDF_RE_SHIFT)
#define SPI_BRD_RSER_RFDF_RE(base) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_RFDF_RE_SHIFT))

/*! @brief Set the RFDF_RE field to a new value. */
#define SPI_WR_RSER_RFDF_RE(base, value) (SPI_RMW_RSER(base, SPI_RSER_RFDF_RE_MASK, SPI_RSER_RFDF_RE(value)))
#define SPI_BWR_RSER_RFDF_RE(base, value) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_RFDF_RE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_RSER, field RFOF_RE[19] (RW)
 *
 * Enables the RFOF flag in the SR to generate an interrupt request.
 *
 * Values:
 * - 0b0 - RFOF interrupt requests are disabled.
 * - 0b1 - RFOF interrupt requests are enabled.
 */
/*@{*/
/*! @brief Read current value of the SPI_RSER_RFOF_RE field. */
#define SPI_RD_RSER_RFOF_RE(base) ((SPI_RSER_REG(base) & SPI_RSER_RFOF_RE_MASK) >> SPI_RSER_RFOF_RE_SHIFT)
#define SPI_BRD_RSER_RFOF_RE(base) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_RFOF_RE_SHIFT))

/*! @brief Set the RFOF_RE field to a new value. */
#define SPI_WR_RSER_RFOF_RE(base, value) (SPI_RMW_RSER(base, SPI_RSER_RFOF_RE_MASK, SPI_RSER_RFOF_RE(value)))
#define SPI_BWR_RSER_RFOF_RE(base, value) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_RFOF_RE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_RSER, field TFFF_DIRS[24] (RW)
 *
 * Selects between generating a DMA request or an interrupt request. When
 * SR[TFFF] and RSER[TFFF_RE] are set, this field selects between generating an
 * interrupt request or a DMA request.
 *
 * Values:
 * - 0b0 - TFFF flag generates interrupt requests.
 * - 0b1 - TFFF flag generates DMA requests.
 */
/*@{*/
/*! @brief Read current value of the SPI_RSER_TFFF_DIRS field. */
#define SPI_RD_RSER_TFFF_DIRS(base) ((SPI_RSER_REG(base) & SPI_RSER_TFFF_DIRS_MASK) >> SPI_RSER_TFFF_DIRS_SHIFT)
#define SPI_BRD_RSER_TFFF_DIRS(base) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_TFFF_DIRS_SHIFT))

/*! @brief Set the TFFF_DIRS field to a new value. */
#define SPI_WR_RSER_TFFF_DIRS(base, value) (SPI_RMW_RSER(base, SPI_RSER_TFFF_DIRS_MASK, SPI_RSER_TFFF_DIRS(value)))
#define SPI_BWR_RSER_TFFF_DIRS(base, value) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_TFFF_DIRS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_RSER, field TFFF_RE[25] (RW)
 *
 * Enables the TFFF flag in the SR to generate a request. The TFFF_DIRS bit
 * selects between generating an interrupt request or a DMA request.
 *
 * Values:
 * - 0b0 - TFFF interrupts or DMA requests are disabled.
 * - 0b1 - TFFF interrupts or DMA requests are enabled.
 */
/*@{*/
/*! @brief Read current value of the SPI_RSER_TFFF_RE field. */
#define SPI_RD_RSER_TFFF_RE(base) ((SPI_RSER_REG(base) & SPI_RSER_TFFF_RE_MASK) >> SPI_RSER_TFFF_RE_SHIFT)
#define SPI_BRD_RSER_TFFF_RE(base) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_TFFF_RE_SHIFT))

/*! @brief Set the TFFF_RE field to a new value. */
#define SPI_WR_RSER_TFFF_RE(base, value) (SPI_RMW_RSER(base, SPI_RSER_TFFF_RE_MASK, SPI_RSER_TFFF_RE(value)))
#define SPI_BWR_RSER_TFFF_RE(base, value) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_TFFF_RE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_RSER, field TFUF_RE[27] (RW)
 *
 * Enables the TFUF flag in the SR to generate an interrupt request.
 *
 * Values:
 * - 0b0 - TFUF interrupt requests are disabled.
 * - 0b1 - TFUF interrupt requests are enabled.
 */
/*@{*/
/*! @brief Read current value of the SPI_RSER_TFUF_RE field. */
#define SPI_RD_RSER_TFUF_RE(base) ((SPI_RSER_REG(base) & SPI_RSER_TFUF_RE_MASK) >> SPI_RSER_TFUF_RE_SHIFT)
#define SPI_BRD_RSER_TFUF_RE(base) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_TFUF_RE_SHIFT))

/*! @brief Set the TFUF_RE field to a new value. */
#define SPI_WR_RSER_TFUF_RE(base, value) (SPI_RMW_RSER(base, SPI_RSER_TFUF_RE_MASK, SPI_RSER_TFUF_RE(value)))
#define SPI_BWR_RSER_TFUF_RE(base, value) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_TFUF_RE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_RSER, field EOQF_RE[28] (RW)
 *
 * Enables the EOQF flag in the SR to generate an interrupt request.
 *
 * Values:
 * - 0b0 - EOQF interrupt requests are disabled.
 * - 0b1 - EOQF interrupt requests are enabled.
 */
/*@{*/
/*! @brief Read current value of the SPI_RSER_EOQF_RE field. */
#define SPI_RD_RSER_EOQF_RE(base) ((SPI_RSER_REG(base) & SPI_RSER_EOQF_RE_MASK) >> SPI_RSER_EOQF_RE_SHIFT)
#define SPI_BRD_RSER_EOQF_RE(base) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_EOQF_RE_SHIFT))

/*! @brief Set the EOQF_RE field to a new value. */
#define SPI_WR_RSER_EOQF_RE(base, value) (SPI_RMW_RSER(base, SPI_RSER_EOQF_RE_MASK, SPI_RSER_EOQF_RE(value)))
#define SPI_BWR_RSER_EOQF_RE(base, value) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_EOQF_RE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_RSER, field TCF_RE[31] (RW)
 *
 * Enables TCF flag in the SR to generate an interrupt request.
 *
 * Values:
 * - 0b0 - TCF interrupt requests are disabled.
 * - 0b1 - TCF interrupt requests are enabled.
 */
/*@{*/
/*! @brief Read current value of the SPI_RSER_TCF_RE field. */
#define SPI_RD_RSER_TCF_RE(base) ((SPI_RSER_REG(base) & SPI_RSER_TCF_RE_MASK) >> SPI_RSER_TCF_RE_SHIFT)
#define SPI_BRD_RSER_TCF_RE(base) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_TCF_RE_SHIFT))

/*! @brief Set the TCF_RE field to a new value. */
#define SPI_WR_RSER_TCF_RE(base, value) (SPI_RMW_RSER(base, SPI_RSER_TCF_RE_MASK, SPI_RSER_TCF_RE(value)))
#define SPI_BWR_RSER_TCF_RE(base, value) (BITBAND_ACCESS32(&SPI_RSER_REG(base), SPI_RSER_TCF_RE_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * SPI_PUSHR - PUSH TX FIFO Register In Master Mode
 ******************************************************************************/

/*!
 * @brief SPI_PUSHR - PUSH TX FIFO Register In Master Mode (RW)
 *
 * Reset value: 0x00000000U
 *
 * Specifies data to be transferred to the TX FIFO. An 8- or 16-bit write access
 * transfers all 32 bits to the TX FIFO. In Master mode, the register transfers
 * 16 bits of data and 16 bits of command information. In Slave mode, all 32 bits
 * can be used as data, supporting up to 32-bit frame operation. A read access
 * of PUSHR returns the topmost TX FIFO entry. When the module is disabled,
 * writing to this register does not update the FIFO. Therefore, any reads performed
 * while the module is disabled return the last PUSHR write performed while the
 * module was still enabled.
 */
/*!
 * @name Constants and macros for entire SPI_PUSHR register
 */
/*@{*/
#define SPI_RD_PUSHR(base)       (SPI_PUSHR_REG(base))
#define SPI_WR_PUSHR(base, value) (SPI_PUSHR_REG(base) = (value))
#define SPI_RMW_PUSHR(base, mask, value) (SPI_WR_PUSHR(base, (SPI_RD_PUSHR(base) & ~(mask)) | (value)))
#define SPI_SET_PUSHR(base, value) (SPI_WR_PUSHR(base, SPI_RD_PUSHR(base) |  (value)))
#define SPI_CLR_PUSHR(base, value) (SPI_WR_PUSHR(base, SPI_RD_PUSHR(base) & ~(value)))
#define SPI_TOG_PUSHR(base, value) (SPI_WR_PUSHR(base, SPI_RD_PUSHR(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SPI_PUSHR bitfields
 */

/*!
 * @name Register SPI_PUSHR, field TXDATA[15:0] (RW)
 *
 * Holds SPI data to be transferred according to the associated SPI command.
 */
/*@{*/
/*! @brief Read current value of the SPI_PUSHR_TXDATA field. */
#define SPI_RD_PUSHR_TXDATA(base) ((SPI_PUSHR_REG(base) & SPI_PUSHR_TXDATA_MASK) >> SPI_PUSHR_TXDATA_SHIFT)
#define SPI_BRD_PUSHR_TXDATA(base) (SPI_RD_PUSHR_TXDATA(base))

/*! @brief Set the TXDATA field to a new value. */
#define SPI_WR_PUSHR_TXDATA(base, value) (SPI_RMW_PUSHR(base, SPI_PUSHR_TXDATA_MASK, SPI_PUSHR_TXDATA(value)))
#define SPI_BWR_PUSHR_TXDATA(base, value) (SPI_WR_PUSHR_TXDATA(base, value))
/*@}*/

/*!
 * @name Register SPI_PUSHR, field PCS[21:16] (RW)
 *
 * Select which PCS signals are to be asserted for the transfer. Refer to the
 * chip-specific SPI information for the number of PCS signals used in this MCU.
 *
 * Values:
 * - 0b000000 - Negate the PCS[x] signal.
 * - 0b000001 - Assert the PCS[x] signal.
 */
/*@{*/
/*! @brief Read current value of the SPI_PUSHR_PCS field. */
#define SPI_RD_PUSHR_PCS(base) ((SPI_PUSHR_REG(base) & SPI_PUSHR_PCS_MASK) >> SPI_PUSHR_PCS_SHIFT)
#define SPI_BRD_PUSHR_PCS(base) (SPI_RD_PUSHR_PCS(base))

/*! @brief Set the PCS field to a new value. */
#define SPI_WR_PUSHR_PCS(base, value) (SPI_RMW_PUSHR(base, SPI_PUSHR_PCS_MASK, SPI_PUSHR_PCS(value)))
#define SPI_BWR_PUSHR_PCS(base, value) (SPI_WR_PUSHR_PCS(base, value))
/*@}*/

/*!
 * @name Register SPI_PUSHR, field CTCNT[26] (RW)
 *
 * Clears the TCNT field in the TCR register. The TCNT field is cleared before
 * the module starts transmitting the current SPI frame.
 *
 * Values:
 * - 0b0 - Do not clear the TCR[TCNT] field.
 * - 0b1 - Clear the TCR[TCNT] field.
 */
/*@{*/
/*! @brief Read current value of the SPI_PUSHR_CTCNT field. */
#define SPI_RD_PUSHR_CTCNT(base) ((SPI_PUSHR_REG(base) & SPI_PUSHR_CTCNT_MASK) >> SPI_PUSHR_CTCNT_SHIFT)
#define SPI_BRD_PUSHR_CTCNT(base) (BITBAND_ACCESS32(&SPI_PUSHR_REG(base), SPI_PUSHR_CTCNT_SHIFT))

/*! @brief Set the CTCNT field to a new value. */
#define SPI_WR_PUSHR_CTCNT(base, value) (SPI_RMW_PUSHR(base, SPI_PUSHR_CTCNT_MASK, SPI_PUSHR_CTCNT(value)))
#define SPI_BWR_PUSHR_CTCNT(base, value) (BITBAND_ACCESS32(&SPI_PUSHR_REG(base), SPI_PUSHR_CTCNT_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_PUSHR, field EOQ[27] (RW)
 *
 * Host software uses this bit to signal to the module that the current SPI
 * transfer is the last in a queue. At the end of the transfer, the EOQF bit in the
 * SR is set.
 *
 * Values:
 * - 0b0 - The SPI data is not the last data to transfer.
 * - 0b1 - The SPI data is the last data to transfer.
 */
/*@{*/
/*! @brief Read current value of the SPI_PUSHR_EOQ field. */
#define SPI_RD_PUSHR_EOQ(base) ((SPI_PUSHR_REG(base) & SPI_PUSHR_EOQ_MASK) >> SPI_PUSHR_EOQ_SHIFT)
#define SPI_BRD_PUSHR_EOQ(base) (BITBAND_ACCESS32(&SPI_PUSHR_REG(base), SPI_PUSHR_EOQ_SHIFT))

/*! @brief Set the EOQ field to a new value. */
#define SPI_WR_PUSHR_EOQ(base, value) (SPI_RMW_PUSHR(base, SPI_PUSHR_EOQ_MASK, SPI_PUSHR_EOQ(value)))
#define SPI_BWR_PUSHR_EOQ(base, value) (BITBAND_ACCESS32(&SPI_PUSHR_REG(base), SPI_PUSHR_EOQ_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SPI_PUSHR, field CTAS[30:28] (RW)
 *
 * Selects which CTAR to use in master mode to specify the transfer attributes
 * for the associated SPI frame. In SPI Slave mode, CTAR0 is used. See the chip
 * configuration details to determine how many CTARs this device has. You should
 * not program a value in this field for a register that is not present.
 *
 * Values:
 * - 0b000 - CTAR0
 * - 0b001 - CTAR1
 * - 0b010 - Reserved
 * - 0b011 - Reserved
 * - 0b100 - Reserved
 * - 0b101 - Reserved
 * - 0b110 - Reserved
 * - 0b111 - Reserved
 */
/*@{*/
/*! @brief Read current value of the SPI_PUSHR_CTAS field. */
#define SPI_RD_PUSHR_CTAS(base) ((SPI_PUSHR_REG(base) & SPI_PUSHR_CTAS_MASK) >> SPI_PUSHR_CTAS_SHIFT)
#define SPI_BRD_PUSHR_CTAS(base) (SPI_RD_PUSHR_CTAS(base))

/*! @brief Set the CTAS field to a new value. */
#define SPI_WR_PUSHR_CTAS(base, value) (SPI_RMW_PUSHR(base, SPI_PUSHR_CTAS_MASK, SPI_PUSHR_CTAS(value)))
#define SPI_BWR_PUSHR_CTAS(base, value) (SPI_WR_PUSHR_CTAS(base, value))
/*@}*/

/*!
 * @name Register SPI_PUSHR, field CONT[31] (RW)
 *
 * Selects a continuous selection format. The bit is used in SPI Master mode.
 * The bit enables the selected PCS signals to remain asserted between transfers.
 *
 * Values:
 * - 0b0 - Return PCSn signals to their inactive state between transfers.
 * - 0b1 - Keep PCSn signals asserted between transfers.
 */
/*@{*/
/*! @brief Read current value of the SPI_PUSHR_CONT field. */
#define SPI_RD_PUSHR_CONT(base) ((SPI_PUSHR_REG(base) & SPI_PUSHR_CONT_MASK) >> SPI_PUSHR_CONT_SHIFT)
#define SPI_BRD_PUSHR_CONT(base) (BITBAND_ACCESS32(&SPI_PUSHR_REG(base), SPI_PUSHR_CONT_SHIFT))

/*! @brief Set the CONT field to a new value. */
#define SPI_WR_PUSHR_CONT(base, value) (SPI_RMW_PUSHR(base, SPI_PUSHR_CONT_MASK, SPI_PUSHR_CONT(value)))
#define SPI_BWR_PUSHR_CONT(base, value) (BITBAND_ACCESS32(&SPI_PUSHR_REG(base), SPI_PUSHR_CONT_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * SPI_PUSHR_SLAVE - PUSH TX FIFO Register In Slave Mode
 ******************************************************************************/

/*!
 * @brief SPI_PUSHR_SLAVE - PUSH TX FIFO Register In Slave Mode (RW)
 *
 * Reset value: 0x00000000U
 *
 * Specifies data to be transferred to the TX FIFO. An 8- or 16-bit write access
 * to PUSHR transfers all 32 bits to the TX FIFO. In master mode, the register
 * transfers 16 bits of data and 16 bits of command information to the TX FIFO. In
 * slave mode, all 32 register bits can be used as data, supporting up to 32-bit
 * SPI Frame operation.
 */
/*!
 * @name Constants and macros for entire SPI_PUSHR_SLAVE register
 */
/*@{*/
#define SPI_RD_PUSHR_SLAVE(base) (SPI_PUSHR_SLAVE_REG(base))
#define SPI_WR_PUSHR_SLAVE(base, value) (SPI_PUSHR_SLAVE_REG(base) = (value))
#define SPI_RMW_PUSHR_SLAVE(base, mask, value) (SPI_WR_PUSHR_SLAVE(base, (SPI_RD_PUSHR_SLAVE(base) & ~(mask)) | (value)))
#define SPI_SET_PUSHR_SLAVE(base, value) (SPI_WR_PUSHR_SLAVE(base, SPI_RD_PUSHR_SLAVE(base) |  (value)))
#define SPI_CLR_PUSHR_SLAVE(base, value) (SPI_WR_PUSHR_SLAVE(base, SPI_RD_PUSHR_SLAVE(base) & ~(value)))
#define SPI_TOG_PUSHR_SLAVE(base, value) (SPI_WR_PUSHR_SLAVE(base, SPI_RD_PUSHR_SLAVE(base) ^  (value)))
/*@}*/

/*******************************************************************************
 * SPI_POPR - POP RX FIFO Register
 ******************************************************************************/

/*!
 * @brief SPI_POPR - POP RX FIFO Register (RO)
 *
 * Reset value: 0x00000000U
 *
 * POPR is used to read the RX FIFO. Eight- or sixteen-bit read accesses to the
 * POPR have the same effect on the RX FIFO as 32-bit read accesses. A write to
 * this register will generate a Transfer Error.
 */
/*!
 * @name Constants and macros for entire SPI_POPR register
 */
/*@{*/
#define SPI_RD_POPR(base)        (SPI_POPR_REG(base))
/*@}*/

/*******************************************************************************
 * SPI_TXFR0 - Transmit FIFO Registers
 ******************************************************************************/

/*!
 * @brief SPI_TXFR0 - Transmit FIFO Registers (RO)
 *
 * Reset value: 0x00000000U
 *
 * TXFRn registers provide visibility into the TX FIFO for debugging purposes.
 * Each register is an entry in the TX FIFO. The registers are read-only and
 * cannot be modified. Reading the TXFRx registers does not alter the state of the TX
 * FIFO.
 */
/*!
 * @name Constants and macros for entire SPI_TXFR0 register
 */
/*@{*/
#define SPI_RD_TXFR0(base)       (SPI_TXFR0_REG(base))
/*@}*/

/*
 * Constants & macros for individual SPI_TXFR0 bitfields
 */

/*!
 * @name Register SPI_TXFR0, field TXDATA[15:0] (RO)
 *
 * Contains the SPI data to be shifted out.
 */
/*@{*/
/*! @brief Read current value of the SPI_TXFR0_TXDATA field. */
#define SPI_RD_TXFR0_TXDATA(base) ((SPI_TXFR0_REG(base) & SPI_TXFR0_TXDATA_MASK) >> SPI_TXFR0_TXDATA_SHIFT)
#define SPI_BRD_TXFR0_TXDATA(base) (SPI_RD_TXFR0_TXDATA(base))
/*@}*/

/*!
 * @name Register SPI_TXFR0, field TXCMD_TXDATA[31:16] (RO)
 *
 * In Master mode the TXCMD field contains the command that sets the transfer
 * attributes for the SPI data. In Slave mode, the TXDATA contains 16 MSB bits of
 * the SPI data to be shifted out.
 */
/*@{*/
/*! @brief Read current value of the SPI_TXFR0_TXCMD_TXDATA field. */
#define SPI_RD_TXFR0_TXCMD_TXDATA(base) ((SPI_TXFR0_REG(base) & SPI_TXFR0_TXCMD_TXDATA_MASK) >> SPI_TXFR0_TXCMD_TXDATA_SHIFT)
#define SPI_BRD_TXFR0_TXCMD_TXDATA(base) (SPI_RD_TXFR0_TXCMD_TXDATA(base))
/*@}*/

/*******************************************************************************
 * SPI_TXFR1 - Transmit FIFO Registers
 ******************************************************************************/

/*!
 * @brief SPI_TXFR1 - Transmit FIFO Registers (RO)
 *
 * Reset value: 0x00000000U
 *
 * TXFRn registers provide visibility into the TX FIFO for debugging purposes.
 * Each register is an entry in the TX FIFO. The registers are read-only and
 * cannot be modified. Reading the TXFRx registers does not alter the state of the TX
 * FIFO.
 */
/*!
 * @name Constants and macros for entire SPI_TXFR1 register
 */
/*@{*/
#define SPI_RD_TXFR1(base)       (SPI_TXFR1_REG(base))
/*@}*/

/*
 * Constants & macros for individual SPI_TXFR1 bitfields
 */

/*!
 * @name Register SPI_TXFR1, field TXDATA[15:0] (RO)
 *
 * Contains the SPI data to be shifted out.
 */
/*@{*/
/*! @brief Read current value of the SPI_TXFR1_TXDATA field. */
#define SPI_RD_TXFR1_TXDATA(base) ((SPI_TXFR1_REG(base) & SPI_TXFR1_TXDATA_MASK) >> SPI_TXFR1_TXDATA_SHIFT)
#define SPI_BRD_TXFR1_TXDATA(base) (SPI_RD_TXFR1_TXDATA(base))
/*@}*/

/*!
 * @name Register SPI_TXFR1, field TXCMD_TXDATA[31:16] (RO)
 *
 * In Master mode the TXCMD field contains the command that sets the transfer
 * attributes for the SPI data. In Slave mode, the TXDATA contains 16 MSB bits of
 * the SPI data to be shifted out.
 */
/*@{*/
/*! @brief Read current value of the SPI_TXFR1_TXCMD_TXDATA field. */
#define SPI_RD_TXFR1_TXCMD_TXDATA(base) ((SPI_TXFR1_REG(base) & SPI_TXFR1_TXCMD_TXDATA_MASK) >> SPI_TXFR1_TXCMD_TXDATA_SHIFT)
#define SPI_BRD_TXFR1_TXCMD_TXDATA(base) (SPI_RD_TXFR1_TXCMD_TXDATA(base))
/*@}*/

/*******************************************************************************
 * SPI_TXFR2 - Transmit FIFO Registers
 ******************************************************************************/

/*!
 * @brief SPI_TXFR2 - Transmit FIFO Registers (RO)
 *
 * Reset value: 0x00000000U
 *
 * TXFRn registers provide visibility into the TX FIFO for debugging purposes.
 * Each register is an entry in the TX FIFO. The registers are read-only and
 * cannot be modified. Reading the TXFRx registers does not alter the state of the TX
 * FIFO.
 */
/*!
 * @name Constants and macros for entire SPI_TXFR2 register
 */
/*@{*/
#define SPI_RD_TXFR2(base)       (SPI_TXFR2_REG(base))
/*@}*/

/*
 * Constants & macros for individual SPI_TXFR2 bitfields
 */

/*!
 * @name Register SPI_TXFR2, field TXDATA[15:0] (RO)
 *
 * Contains the SPI data to be shifted out.
 */
/*@{*/
/*! @brief Read current value of the SPI_TXFR2_TXDATA field. */
#define SPI_RD_TXFR2_TXDATA(base) ((SPI_TXFR2_REG(base) & SPI_TXFR2_TXDATA_MASK) >> SPI_TXFR2_TXDATA_SHIFT)
#define SPI_BRD_TXFR2_TXDATA(base) (SPI_RD_TXFR2_TXDATA(base))
/*@}*/

/*!
 * @name Register SPI_TXFR2, field TXCMD_TXDATA[31:16] (RO)
 *
 * In Master mode the TXCMD field contains the command that sets the transfer
 * attributes for the SPI data. In Slave mode, the TXDATA contains 16 MSB bits of
 * the SPI data to be shifted out.
 */
/*@{*/
/*! @brief Read current value of the SPI_TXFR2_TXCMD_TXDATA field. */
#define SPI_RD_TXFR2_TXCMD_TXDATA(base) ((SPI_TXFR2_REG(base) & SPI_TXFR2_TXCMD_TXDATA_MASK) >> SPI_TXFR2_TXCMD_TXDATA_SHIFT)
#define SPI_BRD_TXFR2_TXCMD_TXDATA(base) (SPI_RD_TXFR2_TXCMD_TXDATA(base))
/*@}*/

/*******************************************************************************
 * SPI_TXFR3 - Transmit FIFO Registers
 ******************************************************************************/

/*!
 * @brief SPI_TXFR3 - Transmit FIFO Registers (RO)
 *
 * Reset value: 0x00000000U
 *
 * TXFRn registers provide visibility into the TX FIFO for debugging purposes.
 * Each register is an entry in the TX FIFO. The registers are read-only and
 * cannot be modified. Reading the TXFRx registers does not alter the state of the TX
 * FIFO.
 */
/*!
 * @name Constants and macros for entire SPI_TXFR3 register
 */
/*@{*/
#define SPI_RD_TXFR3(base)       (SPI_TXFR3_REG(base))
/*@}*/

/*
 * Constants & macros for individual SPI_TXFR3 bitfields
 */

/*!
 * @name Register SPI_TXFR3, field TXDATA[15:0] (RO)
 *
 * Contains the SPI data to be shifted out.
 */
/*@{*/
/*! @brief Read current value of the SPI_TXFR3_TXDATA field. */
#define SPI_RD_TXFR3_TXDATA(base) ((SPI_TXFR3_REG(base) & SPI_TXFR3_TXDATA_MASK) >> SPI_TXFR3_TXDATA_SHIFT)
#define SPI_BRD_TXFR3_TXDATA(base) (SPI_RD_TXFR3_TXDATA(base))
/*@}*/

/*!
 * @name Register SPI_TXFR3, field TXCMD_TXDATA[31:16] (RO)
 *
 * In Master mode the TXCMD field contains the command that sets the transfer
 * attributes for the SPI data. In Slave mode, the TXDATA contains 16 MSB bits of
 * the SPI data to be shifted out.
 */
/*@{*/
/*! @brief Read current value of the SPI_TXFR3_TXCMD_TXDATA field. */
#define SPI_RD_TXFR3_TXCMD_TXDATA(base) ((SPI_TXFR3_REG(base) & SPI_TXFR3_TXCMD_TXDATA_MASK) >> SPI_TXFR3_TXCMD_TXDATA_SHIFT)
#define SPI_BRD_TXFR3_TXCMD_TXDATA(base) (SPI_RD_TXFR3_TXCMD_TXDATA(base))
/*@}*/

/*******************************************************************************
 * SPI_RXFR0 - Receive FIFO Registers
 ******************************************************************************/

/*!
 * @brief SPI_RXFR0 - Receive FIFO Registers (RO)
 *
 * Reset value: 0x00000000U
 *
 * RXFRn provide visibility into the RX FIFO for debugging purposes. Each
 * register is an entry in the RX FIFO. The RXFR registers are read-only. Reading the
 * RXFRx registers does not alter the state of the RX FIFO.
 */
/*!
 * @name Constants and macros for entire SPI_RXFR0 register
 */
/*@{*/
#define SPI_RD_RXFR0(base)       (SPI_RXFR0_REG(base))
/*@}*/

/*******************************************************************************
 * SPI_RXFR1 - Receive FIFO Registers
 ******************************************************************************/

/*!
 * @brief SPI_RXFR1 - Receive FIFO Registers (RO)
 *
 * Reset value: 0x00000000U
 *
 * RXFRn provide visibility into the RX FIFO for debugging purposes. Each
 * register is an entry in the RX FIFO. The RXFR registers are read-only. Reading the
 * RXFRx registers does not alter the state of the RX FIFO.
 */
/*!
 * @name Constants and macros for entire SPI_RXFR1 register
 */
/*@{*/
#define SPI_RD_RXFR1(base)       (SPI_RXFR1_REG(base))
/*@}*/

/*******************************************************************************
 * SPI_RXFR2 - Receive FIFO Registers
 ******************************************************************************/

/*!
 * @brief SPI_RXFR2 - Receive FIFO Registers (RO)
 *
 * Reset value: 0x00000000U
 *
 * RXFRn provide visibility into the RX FIFO for debugging purposes. Each
 * register is an entry in the RX FIFO. The RXFR registers are read-only. Reading the
 * RXFRx registers does not alter the state of the RX FIFO.
 */
/*!
 * @name Constants and macros for entire SPI_RXFR2 register
 */
/*@{*/
#define SPI_RD_RXFR2(base)       (SPI_RXFR2_REG(base))
/*@}*/

/*******************************************************************************
 * SPI_RXFR3 - Receive FIFO Registers
 ******************************************************************************/

/*!
 * @brief SPI_RXFR3 - Receive FIFO Registers (RO)
 *
 * Reset value: 0x00000000U
 *
 * RXFRn provide visibility into the RX FIFO for debugging purposes. Each
 * register is an entry in the RX FIFO. The RXFR registers are read-only. Reading the
 * RXFRx registers does not alter the state of the RX FIFO.
 */
/*!
 * @name Constants and macros for entire SPI_RXFR3 register
 */
/*@{*/
#define SPI_RD_RXFR3(base)       (SPI_RXFR3_REG(base))
/*@}*/

#endif /* __MK22FA12_EXTENSION_SPI_H__ */
/* EOF */
