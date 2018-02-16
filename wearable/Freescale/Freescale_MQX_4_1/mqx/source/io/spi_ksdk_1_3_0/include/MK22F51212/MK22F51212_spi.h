/*
 * Copyright TuringSense, Inc � 2015
 * MK22F51212_spi.h
 *
 *  Created on: Aug 17, 2015
 *      Author: cwati
 *
 * 	Taken from MK22F51212.hfrom KSDK 1.3.0
 *	Chopping off the rest of the file, only using the SPI part.
 */
#ifndef __MK22F51212_SPI_H__
#define __MK22F51212_SPI_H__

#if((EVAL_BOARD && HUB_22F) || (SAT_22F))
#include "stdint.h"
#endif

/* From KSDK 1.0 core_cm4.h */
/* IO definitions (access restrictions to peripheral registers) */
/**
    \defgroup CMSIS_glob_defs CMSIS Global Defines

    <strong>IO Type Qualifiers</strong> are used
    \li to specify the access to peripheral variables.
    \li for automatic generation of peripheral register debug information.
*/
#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions                 */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions                 */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions                */
#define     __IO    volatile             /*!< Defines 'read / write' permissions              */
/* End of From KSDK 1.0 core_cm4.h */


/*
** ###################################################################
**     Compilers:           Keil ARM C/C++ Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          GNU C Compiler - CodeSourcery Sourcery G++
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    K22P121M120SF7RM, Rev. 1, March 24, 2014
**     Version:             rev. 2.8, 2015-02-19
**     Build:               b150225
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MK22F51212
**
**     Copyright (c) 1997 - 2015 Freescale Semiconductor, Inc.
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
**     - rev. 1.0 (2013-07-23)
**         Initial version.
**     - rev. 1.1 (2013-09-17)
**         RM rev. 0.4 update.
**     - rev. 2.0 (2013-10-29)
**         Register accessor macros added to the memory map.
**         Symbols for Processor Expert memory map compatibility added to the memory map.
**         Startup file for gcc has been updated according to CMSIS 3.2.
**         System initialization updated.
**     - rev. 2.1 (2013-10-30)
**         Definition of BITBAND macros updated to support peripherals with 32-bit acces disabled.
**     - rev. 2.2 (2013-12-20)
**         Update according to reference manual rev. 0.6,
**     - rev. 2.3 (2014-01-13)
**         Update according to reference manual rev. 0.61,
**     - rev. 2.4 (2014-02-10)
**         The declaration of clock configurations has been moved to separate header file system_MK22F51212.h
**     - rev. 2.5 (2014-05-06)
**         Update according to reference manual rev. 1.0,
**         Update of system and startup files.
**         Module access macro module_BASES replaced by module_BASE_PTRS.
**     - rev. 2.6 (2014-08-28)
**         Update of system files - default clock configuration changed.
**         Update of startup files - possibility to override DefaultISR added.
**     - rev. 2.7 (2014-10-14)
**         Interrupt INT_LPTimer renamed to INT_LPTMR0, interrupt INT_Watchdog renamed to INT_WDOG_EWM.
**     - rev. 2.8 (2015-02-19)
**         Renamed interrupt vector LLW to LLWU.
**
** ###################################################################
*/

/*!
 * @file MK22F51212.h
 * @version 2.8
 * @date 2015-02-19
 * @brief CMSIS Peripheral Access Layer for MK22F51212
 *
 * CMSIS Peripheral Access Layer for MK22F51212
 */

/* ----------------------------------------------------------------------------
   -- Interrupt vector numbers
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Interrupt_vector_numbers Interrupt vector numbers
 * @{
 */

/** Interrupt Number Definitions */
#define NUMBER_OF_INT_VECTORS 102                /**< Number of interrupts in the Vector table */

typedef enum IRQn {
  /* Auxiliary constants */
  NotAvail_IRQn                = -128,             /**< Not available device specific interrupt */

  /* Core interrupts */
  NonMaskableInt_IRQn          = -14,              /**< Non Maskable Interrupt */
  HardFault_IRQn               = -13,              /**< Cortex-M4 SV Hard Fault Interrupt */
  MemoryManagement_IRQn        = -12,              /**< Cortex-M4 Memory Management Interrupt */
  BusFault_IRQn                = -11,              /**< Cortex-M4 Bus Fault Interrupt */
  UsageFault_IRQn              = -10,              /**< Cortex-M4 Usage Fault Interrupt */
  SVCall_IRQn                  = -5,               /**< Cortex-M4 SV Call Interrupt */
  DebugMonitor_IRQn            = -4,               /**< Cortex-M4 Debug Monitor Interrupt */
  PendSV_IRQn                  = -2,               /**< Cortex-M4 Pend SV Interrupt */
  SysTick_IRQn                 = -1,               /**< Cortex-M4 System Tick Interrupt */

  /* Device specific interrupts */
  DMA0_IRQn                    = 0,                /**< DMA Channel 0 Transfer Complete */
  DMA1_IRQn                    = 1,                /**< DMA Channel 1 Transfer Complete */
  DMA2_IRQn                    = 2,                /**< DMA Channel 2 Transfer Complete */
  DMA3_IRQn                    = 3,                /**< DMA Channel 3 Transfer Complete */
  DMA4_IRQn                    = 4,                /**< DMA Channel 4 Transfer Complete */
  DMA5_IRQn                    = 5,                /**< DMA Channel 5 Transfer Complete */
  DMA6_IRQn                    = 6,                /**< DMA Channel 6 Transfer Complete */
  DMA7_IRQn                    = 7,                /**< DMA Channel 7 Transfer Complete */
  DMA8_IRQn                    = 8,                /**< DMA Channel 8 Transfer Complete */
  DMA9_IRQn                    = 9,                /**< DMA Channel 9 Transfer Complete */
  DMA10_IRQn                   = 10,               /**< DMA Channel 10 Transfer Complete */
  DMA11_IRQn                   = 11,               /**< DMA Channel 11 Transfer Complete */
  DMA12_IRQn                   = 12,               /**< DMA Channel 12 Transfer Complete */
  DMA13_IRQn                   = 13,               /**< DMA Channel 13 Transfer Complete */
  DMA14_IRQn                   = 14,               /**< DMA Channel 14 Transfer Complete */
  DMA15_IRQn                   = 15,               /**< DMA Channel 15 Transfer Complete */
  DMA_Error_IRQn               = 16,               /**< DMA Error Interrupt */
  MCM_IRQn                     = 17,               /**< Normal Interrupt */
  FTF_IRQn                     = 18,               /**< FTFA Command complete interrupt */
  Read_Collision_IRQn          = 19,               /**< Read Collision Interrupt */
  LVD_LVW_IRQn                 = 20,               /**< Low Voltage Detect, Low Voltage Warning */
  LLWU_IRQn                    = 21,               /**< Low Leakage Wakeup Unit */
  WDOG_EWM_IRQn                = 22,               /**< WDOG Interrupt */
  RNG_IRQn                     = 23,               /**< RNG Interrupt */
  I2C0_IRQn                    = 24,               /**< I2C0 interrupt */
  I2C1_IRQn                    = 25,               /**< I2C1 interrupt */
  SPI0_IRQn                    = 26,               /**< SPI0 Interrupt */
  SPI1_IRQn                    = 27,               /**< SPI1 Interrupt */
  I2S0_Tx_IRQn                 = 28,               /**< I2S0 transmit interrupt */
  I2S0_Rx_IRQn                 = 29,               /**< I2S0 receive interrupt */
  LPUART0_IRQn                 = 30,               /**< LPUART0 status/error interrupt */
  UART0_RX_TX_IRQn             = 31,               /**< UART0 Receive/Transmit interrupt */
  UART0_ERR_IRQn               = 32,               /**< UART0 Error interrupt */
  UART1_RX_TX_IRQn             = 33,               /**< UART1 Receive/Transmit interrupt */
  UART1_ERR_IRQn               = 34,               /**< UART1 Error interrupt */
  UART2_RX_TX_IRQn             = 35,               /**< UART2 Receive/Transmit interrupt */
  UART2_ERR_IRQn               = 36,               /**< UART2 Error interrupt */
  Reserved53_IRQn              = 37,               /**< Reserved interrupt 53 */
  Reserved54_IRQn              = 38,               /**< Reserved interrupt 54 */
  ADC0_IRQn                    = 39,               /**< ADC0 interrupt */
  CMP0_IRQn                    = 40,               /**< CMP0 interrupt */
  CMP1_IRQn                    = 41,               /**< CMP1 interrupt */
  FTM0_IRQn                    = 42,               /**< FTM0 fault, overflow and channels interrupt */
  FTM1_IRQn                    = 43,               /**< FTM1 fault, overflow and channels interrupt */
  FTM2_IRQn                    = 44,               /**< FTM2 fault, overflow and channels interrupt */
  Reserved61_IRQn              = 45,               /**< Reserved interrupt 61 */
  RTC_IRQn                     = 46,               /**< RTC interrupt */
  RTC_Seconds_IRQn             = 47,               /**< RTC seconds interrupt */
  PIT0_IRQn                    = 48,               /**< PIT timer channel 0 interrupt */
  PIT1_IRQn                    = 49,               /**< PIT timer channel 1 interrupt */
  PIT2_IRQn                    = 50,               /**< PIT timer channel 2 interrupt */
  PIT3_IRQn                    = 51,               /**< PIT timer channel 3 interrupt */
  PDB0_IRQn                    = 52,               /**< PDB0 Interrupt */
  USB0_IRQn                    = 53,               /**< USB0 interrupt */
  Reserved70_IRQn              = 54,               /**< Reserved interrupt 70 */
  Reserved71_IRQn              = 55,               /**< Reserved interrupt 71 */
  DAC0_IRQn                    = 56,               /**< DAC0 interrupt */
  MCG_IRQn                     = 57,               /**< MCG Interrupt */
  LPTMR0_IRQn                  = 58,               /**< LPTimer interrupt */
  PORTA_IRQn                   = 59,               /**< Port A interrupt */
  PORTB_IRQn                   = 60,               /**< Port B interrupt */
  PORTC_IRQn                   = 61,               /**< Port C interrupt */
  PORTD_IRQn                   = 62,               /**< Port D interrupt */
  PORTE_IRQn                   = 63,               /**< Port E interrupt */
  SWI_IRQn                     = 64,               /**< Software interrupt */
  Reserved81_IRQn              = 65,               /**< Reserved interrupt 81 */
  Reserved82_IRQn              = 66,               /**< Reserved interrupt 82 */
  Reserved83_IRQn              = 67,               /**< Reserved interrupt 83 */
  Reserved84_IRQn              = 68,               /**< Reserved interrupt 84 */
  Reserved85_IRQn              = 69,               /**< Reserved interrupt 85 */
  Reserved86_IRQn              = 70,               /**< Reserved interrupt 86 */
  FTM3_IRQn                    = 71,               /**< FTM3 fault, overflow and channels interrupt */
  DAC1_IRQn                    = 72,               /**< DAC1 interrupt */
  ADC1_IRQn                    = 73,               /**< ADC1 interrupt */
  Reserved90_IRQn              = 74,               /**< Reserved Interrupt 90 */
  Reserved91_IRQn              = 75,               /**< Reserved Interrupt 91 */
  Reserved92_IRQn              = 76,               /**< Reserved Interrupt 92 */
  Reserved93_IRQn              = 77,               /**< Reserved Interrupt 93 */
  Reserved94_IRQn              = 78,               /**< Reserved Interrupt 94 */
  Reserved95_IRQn              = 79,               /**< Reserved Interrupt 95 */
  Reserved96_IRQn              = 80,               /**< Reserved Interrupt 96 */
  Reserved97_IRQn              = 81,               /**< Reserved Interrupt 97 */
  Reserved98_IRQn              = 82,               /**< Reserved Interrupt 98 */
  Reserved99_IRQn              = 83,               /**< Reserved Interrupt 99 */
  Reserved100_IRQn             = 84,               /**< Reserved Interrupt 100 */
  Reserved101_IRQn             = 85                /**< Reserved Interrupt 101 */
} IRQn_Type;

/*!
 * @}
 */ /* end of group Interrupt_vector_numbers */

/* ----------------------------------------------------------------------------
   -- Device Peripheral Access Layer
   ---------------------------------------------------------------------------- */
/*!
 * @addtogroup SPI_Peripheral_Access_Layer SPI Peripheral Access Layer
 * @{
 */

/** SPI - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCR;                               /**< Module Configuration Register, offset: 0x0 */
       uint8_t RESERVED_0[4];
  __IO uint32_t TCR;                               /**< Transfer Count Register, offset: 0x8 */
  union {                                          /* offset: 0xC */
    __IO uint32_t CTAR[2];                           /**< Clock and Transfer Attributes Register (In Master Mode), array offset: 0xC, array step: 0x4 */
    __IO uint32_t CTAR_SLAVE[1];                     /**< Clock and Transfer Attributes Register (In Slave Mode), array offset: 0xC, array step: 0x4 */
  };
       uint8_t RESERVED_1[24];
  __IO uint32_t SR;                                /**< Status Register, offset: 0x2C */
  __IO uint32_t RSER;                              /**< DMA/Interrupt Request Select and Enable Register, offset: 0x30 */
  union {                                          /* offset: 0x34 */
    __IO uint32_t PUSHR;                             /**< PUSH TX FIFO Register In Master Mode, offset: 0x34 */
    __IO uint32_t PUSHR_SLAVE;                       /**< PUSH TX FIFO Register In Slave Mode, offset: 0x34 */
  };
  __I  uint32_t POPR;                              /**< POP RX FIFO Register, offset: 0x38 */
  __I  uint32_t TXFR0;                             /**< Transmit FIFO Registers, offset: 0x3C */
  __I  uint32_t TXFR1;                             /**< Transmit FIFO Registers, offset: 0x40 */
  __I  uint32_t TXFR2;                             /**< Transmit FIFO Registers, offset: 0x44 */
  __I  uint32_t TXFR3;                             /**< Transmit FIFO Registers, offset: 0x48 */
       uint8_t RESERVED_2[48];
  __I  uint32_t RXFR0;                             /**< Receive FIFO Registers, offset: 0x7C */
  __I  uint32_t RXFR1;                             /**< Receive FIFO Registers, offset: 0x80 */
  __I  uint32_t RXFR2;                             /**< Receive FIFO Registers, offset: 0x84 */
  __I  uint32_t RXFR3;                             /**< Receive FIFO Registers, offset: 0x88 */
#if ((EVAL_BOARD && HUB_22F) || (SAT_22F))
} SPI_Type;
#else
} SPI_Type, *SPI_MemMapPtr;
#endif

/* ----------------------------------------------------------------------------
   -- SPI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Register_Accessor_Macros SPI - Register accessor macros
 * @{
 */


/* SPI - Register accessors */
#define SPI_MCR_REG(base)                        ((base)->MCR)
#define SPI_TCR_REG(base)                        ((base)->TCR)
#define SPI_CTAR_REG(base,index2)                ((base)->CTAR[index2])
#define SPI_CTAR_COUNT                           2
#define SPI_CTAR_SLAVE_REG(base,index2)          ((base)->CTAR_SLAVE[index2])
#define SPI_CTAR_SLAVE_COUNT                     1
#define SPI_SR_REG(base)                         ((base)->SR)
#define SPI_RSER_REG(base)                       ((base)->RSER)
#define SPI_PUSHR_REG(base)                      ((base)->PUSHR)
#define SPI_PUSHR_SLAVE_REG(base)                ((base)->PUSHR_SLAVE)
#define SPI_POPR_REG(base)                       ((base)->POPR)
#define SPI_TXFR0_REG(base)                      ((base)->TXFR0)
#define SPI_TXFR1_REG(base)                      ((base)->TXFR1)
#define SPI_TXFR2_REG(base)                      ((base)->TXFR2)
#define SPI_TXFR3_REG(base)                      ((base)->TXFR3)
#define SPI_RXFR0_REG(base)                      ((base)->RXFR0)
#define SPI_RXFR1_REG(base)                      ((base)->RXFR1)
#define SPI_RXFR2_REG(base)                      ((base)->RXFR2)
#define SPI_RXFR3_REG(base)                      ((base)->RXFR3)

/*!
 * @}
 */ /* end of group SPI_Register_Accessor_Macros */


///* ----------------------------------------------------------------------------
//   -- SPI Register Masks
//   ---------------------------------------------------------------------------- */
//
///*!
// * @addtogroup SPI_Register_Masks SPI Register Masks
// * @{
// */
//
///* MCR Bit Fields */
//#define SPI_MCR_HALT_MASK                        0x1u
//#define SPI_MCR_HALT_SHIFT                       0
//#define SPI_MCR_HALT_WIDTH                       1
//#define SPI_MCR_HALT(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_MCR_HALT_SHIFT))&SPI_MCR_HALT_MASK)
//#define SPI_MCR_SMPL_PT_MASK                     0x300u
//#define SPI_MCR_SMPL_PT_SHIFT                    8
//#define SPI_MCR_SMPL_PT_WIDTH                    2
//#define SPI_MCR_SMPL_PT(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_MCR_SMPL_PT_SHIFT))&SPI_MCR_SMPL_PT_MASK)
//#define SPI_MCR_CLR_RXF_MASK                     0x400u
//#define SPI_MCR_CLR_RXF_SHIFT                    10
//#define SPI_MCR_CLR_RXF_WIDTH                    1
//#define SPI_MCR_CLR_RXF(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_MCR_CLR_RXF_SHIFT))&SPI_MCR_CLR_RXF_MASK)
//#define SPI_MCR_CLR_TXF_MASK                     0x800u
//#define SPI_MCR_CLR_TXF_SHIFT                    11
//#define SPI_MCR_CLR_TXF_WIDTH                    1
//#define SPI_MCR_CLR_TXF(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_MCR_CLR_TXF_SHIFT))&SPI_MCR_CLR_TXF_MASK)
//#define SPI_MCR_DIS_RXF_MASK                     0x1000u
//#define SPI_MCR_DIS_RXF_SHIFT                    12
//#define SPI_MCR_DIS_RXF_WIDTH                    1
//#define SPI_MCR_DIS_RXF(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_MCR_DIS_RXF_SHIFT))&SPI_MCR_DIS_RXF_MASK)
//#define SPI_MCR_DIS_TXF_MASK                     0x2000u
//#define SPI_MCR_DIS_TXF_SHIFT                    13
//#define SPI_MCR_DIS_TXF_WIDTH                    1
//#define SPI_MCR_DIS_TXF(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_MCR_DIS_TXF_SHIFT))&SPI_MCR_DIS_TXF_MASK)
//#define SPI_MCR_MDIS_MASK                        0x4000u
//#define SPI_MCR_MDIS_SHIFT                       14
//#define SPI_MCR_MDIS_WIDTH                       1
//#define SPI_MCR_MDIS(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_MCR_MDIS_SHIFT))&SPI_MCR_MDIS_MASK)
//#define SPI_MCR_DOZE_MASK                        0x8000u
//#define SPI_MCR_DOZE_SHIFT                       15
//#define SPI_MCR_DOZE_WIDTH                       1
//#define SPI_MCR_DOZE(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_MCR_DOZE_SHIFT))&SPI_MCR_DOZE_MASK)
//#define SPI_MCR_PCSIS_MASK                       0x3F0000u
//#define SPI_MCR_PCSIS_SHIFT                      16
//#define SPI_MCR_PCSIS_WIDTH                      6
//#define SPI_MCR_PCSIS(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_MCR_PCSIS_SHIFT))&SPI_MCR_PCSIS_MASK)
//#define SPI_MCR_ROOE_MASK                        0x1000000u
//#define SPI_MCR_ROOE_SHIFT                       24
//#define SPI_MCR_ROOE_WIDTH                       1
//#define SPI_MCR_ROOE(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_MCR_ROOE_SHIFT))&SPI_MCR_ROOE_MASK)
//#define SPI_MCR_PCSSE_MASK                       0x2000000u
//#define SPI_MCR_PCSSE_SHIFT                      25
//#define SPI_MCR_PCSSE_WIDTH                      1
//#define SPI_MCR_PCSSE(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_MCR_PCSSE_SHIFT))&SPI_MCR_PCSSE_MASK)
//#define SPI_MCR_MTFE_MASK                        0x4000000u
//#define SPI_MCR_MTFE_SHIFT                       26
//#define SPI_MCR_MTFE_WIDTH                       1
//#define SPI_MCR_MTFE(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_MCR_MTFE_SHIFT))&SPI_MCR_MTFE_MASK)
//#define SPI_MCR_FRZ_MASK                         0x8000000u
//#define SPI_MCR_FRZ_SHIFT                        27
//#define SPI_MCR_FRZ_WIDTH                        1
//#define SPI_MCR_FRZ(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_MCR_FRZ_SHIFT))&SPI_MCR_FRZ_MASK)
//#define SPI_MCR_DCONF_MASK                       0x30000000u
//#define SPI_MCR_DCONF_SHIFT                      28
//#define SPI_MCR_DCONF_WIDTH                      2
//#define SPI_MCR_DCONF(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_MCR_DCONF_SHIFT))&SPI_MCR_DCONF_MASK)
//#define SPI_MCR_CONT_SCKE_MASK                   0x40000000u
//#define SPI_MCR_CONT_SCKE_SHIFT                  30
//#define SPI_MCR_CONT_SCKE_WIDTH                  1
//#define SPI_MCR_CONT_SCKE(x)                     (((uint32_t)(((uint32_t)(x))<<SPI_MCR_CONT_SCKE_SHIFT))&SPI_MCR_CONT_SCKE_MASK)
//#define SPI_MCR_MSTR_MASK                        0x80000000u
//#define SPI_MCR_MSTR_SHIFT                       31
//#define SPI_MCR_MSTR_WIDTH                       1
//#define SPI_MCR_MSTR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_MCR_MSTR_SHIFT))&SPI_MCR_MSTR_MASK)
///* TCR Bit Fields */
//#define SPI_TCR_SPI_TCNT_MASK                    0xFFFF0000u
//#define SPI_TCR_SPI_TCNT_SHIFT                   16
//#define SPI_TCR_SPI_TCNT_WIDTH                   16
//#define SPI_TCR_SPI_TCNT(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TCR_SPI_TCNT_SHIFT))&SPI_TCR_SPI_TCNT_MASK)
///* CTAR Bit Fields */
//#define SPI_CTAR_BR_MASK                         0xFu
//#define SPI_CTAR_BR_SHIFT                        0
//#define SPI_CTAR_BR_WIDTH                        4
//#define SPI_CTAR_BR(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_BR_SHIFT))&SPI_CTAR_BR_MASK)
//#define SPI_CTAR_DT_MASK                         0xF0u
//#define SPI_CTAR_DT_SHIFT                        4
//#define SPI_CTAR_DT_WIDTH                        4
//#define SPI_CTAR_DT(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_DT_SHIFT))&SPI_CTAR_DT_MASK)
//#define SPI_CTAR_ASC_MASK                        0xF00u
//#define SPI_CTAR_ASC_SHIFT                       8
//#define SPI_CTAR_ASC_WIDTH                       4
//#define SPI_CTAR_ASC(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_ASC_SHIFT))&SPI_CTAR_ASC_MASK)
//#define SPI_CTAR_CSSCK_MASK                      0xF000u
//#define SPI_CTAR_CSSCK_SHIFT                     12
//#define SPI_CTAR_CSSCK_WIDTH                     4
//#define SPI_CTAR_CSSCK(x)                        (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_CSSCK_SHIFT))&SPI_CTAR_CSSCK_MASK)
//#define SPI_CTAR_PBR_MASK                        0x30000u
//#define SPI_CTAR_PBR_SHIFT                       16
//#define SPI_CTAR_PBR_WIDTH                       2
//#define SPI_CTAR_PBR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PBR_SHIFT))&SPI_CTAR_PBR_MASK)
//#define SPI_CTAR_PDT_MASK                        0xC0000u
//#define SPI_CTAR_PDT_SHIFT                       18
//#define SPI_CTAR_PDT_WIDTH                       2
//#define SPI_CTAR_PDT(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PDT_SHIFT))&SPI_CTAR_PDT_MASK)
//#define SPI_CTAR_PASC_MASK                       0x300000u
//#define SPI_CTAR_PASC_SHIFT                      20
//#define SPI_CTAR_PASC_WIDTH                      2
//#define SPI_CTAR_PASC(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PASC_SHIFT))&SPI_CTAR_PASC_MASK)
//#define SPI_CTAR_PCSSCK_MASK                     0xC00000u
//#define SPI_CTAR_PCSSCK_SHIFT                    22
//#define SPI_CTAR_PCSSCK_WIDTH                    2
//#define SPI_CTAR_PCSSCK(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PCSSCK_SHIFT))&SPI_CTAR_PCSSCK_MASK)
//#define SPI_CTAR_LSBFE_MASK                      0x1000000u
//#define SPI_CTAR_LSBFE_SHIFT                     24
//#define SPI_CTAR_LSBFE_WIDTH                     1
//#define SPI_CTAR_LSBFE(x)                        (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_LSBFE_SHIFT))&SPI_CTAR_LSBFE_MASK)
//#define SPI_CTAR_CPHA_MASK                       0x2000000u
//#define SPI_CTAR_CPHA_SHIFT                      25
//#define SPI_CTAR_CPHA_WIDTH                      1
//#define SPI_CTAR_CPHA(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_CPHA_SHIFT))&SPI_CTAR_CPHA_MASK)
//#define SPI_CTAR_CPOL_MASK                       0x4000000u
//#define SPI_CTAR_CPOL_SHIFT                      26
//#define SPI_CTAR_CPOL_WIDTH                      1
//#define SPI_CTAR_CPOL(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_CPOL_SHIFT))&SPI_CTAR_CPOL_MASK)
//#define SPI_CTAR_FMSZ_MASK                       0x78000000u
//#define SPI_CTAR_FMSZ_SHIFT                      27
//#define SPI_CTAR_FMSZ_WIDTH                      4
//#define SPI_CTAR_FMSZ(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_FMSZ_SHIFT))&SPI_CTAR_FMSZ_MASK)
//#define SPI_CTAR_DBR_MASK                        0x80000000u
//#define SPI_CTAR_DBR_SHIFT                       31
//#define SPI_CTAR_DBR_WIDTH                       1
//#define SPI_CTAR_DBR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_DBR_SHIFT))&SPI_CTAR_DBR_MASK)
///* CTAR_SLAVE Bit Fields */
//#define SPI_CTAR_SLAVE_CPHA_MASK                 0x2000000u
//#define SPI_CTAR_SLAVE_CPHA_SHIFT                25
//#define SPI_CTAR_SLAVE_CPHA_WIDTH                1
//#define SPI_CTAR_SLAVE_CPHA(x)                   (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_SLAVE_CPHA_SHIFT))&SPI_CTAR_SLAVE_CPHA_MASK)
//#define SPI_CTAR_SLAVE_CPOL_MASK                 0x4000000u
//#define SPI_CTAR_SLAVE_CPOL_SHIFT                26
//#define SPI_CTAR_SLAVE_CPOL_WIDTH                1
//#define SPI_CTAR_SLAVE_CPOL(x)                   (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_SLAVE_CPOL_SHIFT))&SPI_CTAR_SLAVE_CPOL_MASK)
//#define SPI_CTAR_SLAVE_FMSZ_MASK                 0xF8000000u
//#define SPI_CTAR_SLAVE_FMSZ_SHIFT                27
//#define SPI_CTAR_SLAVE_FMSZ_WIDTH                5
//#define SPI_CTAR_SLAVE_FMSZ(x)                   (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_SLAVE_FMSZ_SHIFT))&SPI_CTAR_SLAVE_FMSZ_MASK)
///* SR Bit Fields */
//#define SPI_SR_POPNXTPTR_MASK                    0xFu
//#define SPI_SR_POPNXTPTR_SHIFT                   0
//#define SPI_SR_POPNXTPTR_WIDTH                   4
//#define SPI_SR_POPNXTPTR(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_SR_POPNXTPTR_SHIFT))&SPI_SR_POPNXTPTR_MASK)
//#define SPI_SR_RXCTR_MASK                        0xF0u
//#define SPI_SR_RXCTR_SHIFT                       4
//#define SPI_SR_RXCTR_WIDTH                       4
//#define SPI_SR_RXCTR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_SR_RXCTR_SHIFT))&SPI_SR_RXCTR_MASK)
//#define SPI_SR_TXNXTPTR_MASK                     0xF00u
//#define SPI_SR_TXNXTPTR_SHIFT                    8
//#define SPI_SR_TXNXTPTR_WIDTH                    4
//#define SPI_SR_TXNXTPTR(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_SR_TXNXTPTR_SHIFT))&SPI_SR_TXNXTPTR_MASK)
//#define SPI_SR_TXCTR_MASK                        0xF000u
//#define SPI_SR_TXCTR_SHIFT                       12
//#define SPI_SR_TXCTR_WIDTH                       4
//#define SPI_SR_TXCTR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_SR_TXCTR_SHIFT))&SPI_SR_TXCTR_MASK)
//#define SPI_SR_RFDF_MASK                         0x20000u
//#define SPI_SR_RFDF_SHIFT                        17
//#define SPI_SR_RFDF_WIDTH                        1
//#define SPI_SR_RFDF(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_SR_RFDF_SHIFT))&SPI_SR_RFDF_MASK)
//#define SPI_SR_RFOF_MASK                         0x80000u
//#define SPI_SR_RFOF_SHIFT                        19
//#define SPI_SR_RFOF_WIDTH                        1
//#define SPI_SR_RFOF(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_SR_RFOF_SHIFT))&SPI_SR_RFOF_MASK)
//#define SPI_SR_TFFF_MASK                         0x2000000u
//#define SPI_SR_TFFF_SHIFT                        25
//#define SPI_SR_TFFF_WIDTH                        1
//#define SPI_SR_TFFF(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_SR_TFFF_SHIFT))&SPI_SR_TFFF_MASK)
//#define SPI_SR_TFUF_MASK                         0x8000000u
//#define SPI_SR_TFUF_SHIFT                        27
//#define SPI_SR_TFUF_WIDTH                        1
//#define SPI_SR_TFUF(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_SR_TFUF_SHIFT))&SPI_SR_TFUF_MASK)
//#define SPI_SR_EOQF_MASK                         0x10000000u
//#define SPI_SR_EOQF_SHIFT                        28
//#define SPI_SR_EOQF_WIDTH                        1
//#define SPI_SR_EOQF(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_SR_EOQF_SHIFT))&SPI_SR_EOQF_MASK)
//#define SPI_SR_TXRXS_MASK                        0x40000000u
//#define SPI_SR_TXRXS_SHIFT                       30
//#define SPI_SR_TXRXS_WIDTH                       1
//#define SPI_SR_TXRXS(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_SR_TXRXS_SHIFT))&SPI_SR_TXRXS_MASK)
//#define SPI_SR_TCF_MASK                          0x80000000u
//#define SPI_SR_TCF_SHIFT                         31
//#define SPI_SR_TCF_WIDTH                         1
//#define SPI_SR_TCF(x)                            (((uint32_t)(((uint32_t)(x))<<SPI_SR_TCF_SHIFT))&SPI_SR_TCF_MASK)
///* RSER Bit Fields */
//#define SPI_RSER_RFDF_DIRS_MASK                  0x10000u
//#define SPI_RSER_RFDF_DIRS_SHIFT                 16
//#define SPI_RSER_RFDF_DIRS_WIDTH                 1
//#define SPI_RSER_RFDF_DIRS(x)                    (((uint32_t)(((uint32_t)(x))<<SPI_RSER_RFDF_DIRS_SHIFT))&SPI_RSER_RFDF_DIRS_MASK)
//#define SPI_RSER_RFDF_RE_MASK                    0x20000u
//#define SPI_RSER_RFDF_RE_SHIFT                   17
//#define SPI_RSER_RFDF_RE_WIDTH                   1
//#define SPI_RSER_RFDF_RE(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RSER_RFDF_RE_SHIFT))&SPI_RSER_RFDF_RE_MASK)
//#define SPI_RSER_RFOF_RE_MASK                    0x80000u
//#define SPI_RSER_RFOF_RE_SHIFT                   19
//#define SPI_RSER_RFOF_RE_WIDTH                   1
//#define SPI_RSER_RFOF_RE(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RSER_RFOF_RE_SHIFT))&SPI_RSER_RFOF_RE_MASK)
//#define SPI_RSER_TFFF_DIRS_MASK                  0x1000000u
//#define SPI_RSER_TFFF_DIRS_SHIFT                 24
//#define SPI_RSER_TFFF_DIRS_WIDTH                 1
//#define SPI_RSER_TFFF_DIRS(x)                    (((uint32_t)(((uint32_t)(x))<<SPI_RSER_TFFF_DIRS_SHIFT))&SPI_RSER_TFFF_DIRS_MASK)
//#define SPI_RSER_TFFF_RE_MASK                    0x2000000u
//#define SPI_RSER_TFFF_RE_SHIFT                   25
//#define SPI_RSER_TFFF_RE_WIDTH                   1
//#define SPI_RSER_TFFF_RE(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RSER_TFFF_RE_SHIFT))&SPI_RSER_TFFF_RE_MASK)
//#define SPI_RSER_TFUF_RE_MASK                    0x8000000u
//#define SPI_RSER_TFUF_RE_SHIFT                   27
//#define SPI_RSER_TFUF_RE_WIDTH                   1
//#define SPI_RSER_TFUF_RE(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RSER_TFUF_RE_SHIFT))&SPI_RSER_TFUF_RE_MASK)
//#define SPI_RSER_EOQF_RE_MASK                    0x10000000u
//#define SPI_RSER_EOQF_RE_SHIFT                   28
//#define SPI_RSER_EOQF_RE_WIDTH                   1
//#define SPI_RSER_EOQF_RE(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RSER_EOQF_RE_SHIFT))&SPI_RSER_EOQF_RE_MASK)
//#define SPI_RSER_TCF_RE_MASK                     0x80000000u
//#define SPI_RSER_TCF_RE_SHIFT                    31
//#define SPI_RSER_TCF_RE_WIDTH                    1
//#define SPI_RSER_TCF_RE(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_RSER_TCF_RE_SHIFT))&SPI_RSER_TCF_RE_MASK)
///* PUSHR Bit Fields */
//#define SPI_PUSHR_TXDATA_MASK                    0xFFFFu
//#define SPI_PUSHR_TXDATA_SHIFT                   0
//#define SPI_PUSHR_TXDATA_WIDTH                   16
//#define SPI_PUSHR_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_TXDATA_SHIFT))&SPI_PUSHR_TXDATA_MASK)
//#define SPI_PUSHR_PCS_MASK                       0x3F0000u
//#define SPI_PUSHR_PCS_SHIFT                      16
//#define SPI_PUSHR_PCS_WIDTH                      6
//#define SPI_PUSHR_PCS(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_PCS_SHIFT))&SPI_PUSHR_PCS_MASK)
//#define SPI_PUSHR_CTCNT_MASK                     0x4000000u
//#define SPI_PUSHR_CTCNT_SHIFT                    26
//#define SPI_PUSHR_CTCNT_WIDTH                    1
//#define SPI_PUSHR_CTCNT(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_CTCNT_SHIFT))&SPI_PUSHR_CTCNT_MASK)
//#define SPI_PUSHR_EOQ_MASK                       0x8000000u
//#define SPI_PUSHR_EOQ_SHIFT                      27
//#define SPI_PUSHR_EOQ_WIDTH                      1
//#define SPI_PUSHR_EOQ(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_EOQ_SHIFT))&SPI_PUSHR_EOQ_MASK)
//#define SPI_PUSHR_CTAS_MASK                      0x70000000u
//#define SPI_PUSHR_CTAS_SHIFT                     28
//#define SPI_PUSHR_CTAS_WIDTH                     3
//#define SPI_PUSHR_CTAS(x)                        (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_CTAS_SHIFT))&SPI_PUSHR_CTAS_MASK)
//#define SPI_PUSHR_CONT_MASK                      0x80000000u
//#define SPI_PUSHR_CONT_SHIFT                     31
//#define SPI_PUSHR_CONT_WIDTH                     1
//#define SPI_PUSHR_CONT(x)                        (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_CONT_SHIFT))&SPI_PUSHR_CONT_MASK)
///* PUSHR_SLAVE Bit Fields */
//#define SPI_PUSHR_SLAVE_TXDATA_MASK              0xFFFFFFFFu
//#define SPI_PUSHR_SLAVE_TXDATA_SHIFT             0
//#define SPI_PUSHR_SLAVE_TXDATA_WIDTH             32
//#define SPI_PUSHR_SLAVE_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_SLAVE_TXDATA_SHIFT))&SPI_PUSHR_SLAVE_TXDATA_MASK)
///* POPR Bit Fields */
//#define SPI_POPR_RXDATA_MASK                     0xFFFFFFFFu
//#define SPI_POPR_RXDATA_SHIFT                    0
//#define SPI_POPR_RXDATA_WIDTH                    32
//#define SPI_POPR_RXDATA(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_POPR_RXDATA_SHIFT))&SPI_POPR_RXDATA_MASK)
///* TXFR0 Bit Fields */
//#define SPI_TXFR0_TXDATA_MASK                    0xFFFFu
//#define SPI_TXFR0_TXDATA_SHIFT                   0
//#define SPI_TXFR0_TXDATA_WIDTH                   16
//#define SPI_TXFR0_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TXFR0_TXDATA_SHIFT))&SPI_TXFR0_TXDATA_MASK)
//#define SPI_TXFR0_TXCMD_TXDATA_MASK              0xFFFF0000u
//#define SPI_TXFR0_TXCMD_TXDATA_SHIFT             16
//#define SPI_TXFR0_TXCMD_TXDATA_WIDTH             16
//#define SPI_TXFR0_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_TXFR0_TXCMD_TXDATA_SHIFT))&SPI_TXFR0_TXCMD_TXDATA_MASK)
///* TXFR1 Bit Fields */
//#define SPI_TXFR1_TXDATA_MASK                    0xFFFFu
//#define SPI_TXFR1_TXDATA_SHIFT                   0
//#define SPI_TXFR1_TXDATA_WIDTH                   16
//#define SPI_TXFR1_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TXFR1_TXDATA_SHIFT))&SPI_TXFR1_TXDATA_MASK)
//#define SPI_TXFR1_TXCMD_TXDATA_MASK              0xFFFF0000u
//#define SPI_TXFR1_TXCMD_TXDATA_SHIFT             16
//#define SPI_TXFR1_TXCMD_TXDATA_WIDTH             16
//#define SPI_TXFR1_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_TXFR1_TXCMD_TXDATA_SHIFT))&SPI_TXFR1_TXCMD_TXDATA_MASK)
///* TXFR2 Bit Fields */
//#define SPI_TXFR2_TXDATA_MASK                    0xFFFFu
//#define SPI_TXFR2_TXDATA_SHIFT                   0
//#define SPI_TXFR2_TXDATA_WIDTH                   16
//#define SPI_TXFR2_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TXFR2_TXDATA_SHIFT))&SPI_TXFR2_TXDATA_MASK)
//#define SPI_TXFR2_TXCMD_TXDATA_MASK              0xFFFF0000u
//#define SPI_TXFR2_TXCMD_TXDATA_SHIFT             16
//#define SPI_TXFR2_TXCMD_TXDATA_WIDTH             16
//#define SPI_TXFR2_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_TXFR2_TXCMD_TXDATA_SHIFT))&SPI_TXFR2_TXCMD_TXDATA_MASK)
///* TXFR3 Bit Fields */
//#define SPI_TXFR3_TXDATA_MASK                    0xFFFFu
//#define SPI_TXFR3_TXDATA_SHIFT                   0
//#define SPI_TXFR3_TXDATA_WIDTH                   16
//#define SPI_TXFR3_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TXFR3_TXDATA_SHIFT))&SPI_TXFR3_TXDATA_MASK)
//#define SPI_TXFR3_TXCMD_TXDATA_MASK              0xFFFF0000u
//#define SPI_TXFR3_TXCMD_TXDATA_SHIFT             16
//#define SPI_TXFR3_TXCMD_TXDATA_WIDTH             16
//#define SPI_TXFR3_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_TXFR3_TXCMD_TXDATA_SHIFT))&SPI_TXFR3_TXCMD_TXDATA_MASK)
///* RXFR0 Bit Fields */
//#define SPI_RXFR0_RXDATA_MASK                    0xFFFFFFFFu
//#define SPI_RXFR0_RXDATA_SHIFT                   0
//#define SPI_RXFR0_RXDATA_WIDTH                   32
//#define SPI_RXFR0_RXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RXFR0_RXDATA_SHIFT))&SPI_RXFR0_RXDATA_MASK)
///* RXFR1 Bit Fields */
//#define SPI_RXFR1_RXDATA_MASK                    0xFFFFFFFFu
//#define SPI_RXFR1_RXDATA_SHIFT                   0
//#define SPI_RXFR1_RXDATA_WIDTH                   32
//#define SPI_RXFR1_RXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RXFR1_RXDATA_SHIFT))&SPI_RXFR1_RXDATA_MASK)
///* RXFR2 Bit Fields */
//#define SPI_RXFR2_RXDATA_MASK                    0xFFFFFFFFu
//#define SPI_RXFR2_RXDATA_SHIFT                   0
//#define SPI_RXFR2_RXDATA_WIDTH                   32
//#define SPI_RXFR2_RXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RXFR2_RXDATA_SHIFT))&SPI_RXFR2_RXDATA_MASK)
///* RXFR3 Bit Fields */
//#define SPI_RXFR3_RXDATA_MASK                    0xFFFFFFFFu
//#define SPI_RXFR3_RXDATA_SHIFT                   0
//#define SPI_RXFR3_RXDATA_WIDTH                   32
//#define SPI_RXFR3_RXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RXFR3_RXDATA_SHIFT))&SPI_RXFR3_RXDATA_MASK)
//
///*!
// * @}
// */ /* end of group SPI_Register_Masks */


/* SPI - Peripheral instance base addresses */
/** Peripheral SPI0 base address */
#define SPI0_BASE                                (0x4002C000u)
/** Peripheral SPI0 base pointer */
#define SPI0                                     ((SPI_Type *)SPI0_BASE)
//#define SPI0_BASE_PTR                            (SPI0)
/** Peripheral SPI1 base address */
#define SPI1_BASE                                (0x4002D000u)
/** Peripheral SPI1 base pointer */
#define SPI1                                     ((SPI_Type *)SPI1_BASE)
//#define SPI1_BASE_PTR                            (SPI1)
/** Array initializer of SPI peripheral base addresses */
#define SPI_BASE_ADDRS                           { SPI0_BASE, SPI1_BASE }
/** Array initializer of SPI peripheral base pointers */
//#define SPI_BASE_PTRS                            { SPI0, SPI1 }
/** Interrupt vectors for the SPI peripheral type */
#define SPI_IRQS                                 { SPI0_IRQn, SPI1_IRQn }

///* ----------------------------------------------------------------------------
//   -- SPI - Register accessor macros
//   ---------------------------------------------------------------------------- */
//
///*!
// * @addtogroup SPI_Register_Accessor_Macros SPI - Register accessor macros
// * @{
// */
//
//
///* SPI - Register instance definitions */
///* SPI0 */
//#define SPI0_MCR                                 SPI_MCR_REG(SPI0)
//#define SPI0_TCR                                 SPI_TCR_REG(SPI0)
//#define SPI0_CTAR0                               SPI_CTAR_REG(SPI0,0)
//#define SPI0_CTAR0_SLAVE                         SPI_CTAR_SLAVE_REG(SPI0,0)
//#define SPI0_CTAR1                               SPI_CTAR_REG(SPI0,1)
//#define SPI0_SR                                  SPI_SR_REG(SPI0)
//#define SPI0_RSER                                SPI_RSER_REG(SPI0)
//#define SPI0_PUSHR                               SPI_PUSHR_REG(SPI0)
//#define SPI0_PUSHR_SLAVE                         SPI_PUSHR_SLAVE_REG(SPI0)
//#define SPI0_POPR                                SPI_POPR_REG(SPI0)
//#define SPI0_TXFR0                               SPI_TXFR0_REG(SPI0)
//#define SPI0_TXFR1                               SPI_TXFR1_REG(SPI0)
//#define SPI0_TXFR2                               SPI_TXFR2_REG(SPI0)
//#define SPI0_TXFR3                               SPI_TXFR3_REG(SPI0)
//#define SPI0_RXFR0                               SPI_RXFR0_REG(SPI0)
//#define SPI0_RXFR1                               SPI_RXFR1_REG(SPI0)
//#define SPI0_RXFR2                               SPI_RXFR2_REG(SPI0)
//#define SPI0_RXFR3                               SPI_RXFR3_REG(SPI0)
///* SPI1 */
//#define SPI1_MCR                                 SPI_MCR_REG(SPI1)
//#define SPI1_TCR                                 SPI_TCR_REG(SPI1)
//#define SPI1_CTAR0                               SPI_CTAR_REG(SPI1,0)
//#define SPI1_CTAR0_SLAVE                         SPI_CTAR_SLAVE_REG(SPI1,0)
//#define SPI1_CTAR1                               SPI_CTAR_REG(SPI1,1)
//#define SPI1_SR                                  SPI_SR_REG(SPI1)
//#define SPI1_RSER                                SPI_RSER_REG(SPI1)
//#define SPI1_PUSHR                               SPI_PUSHR_REG(SPI1)
//#define SPI1_PUSHR_SLAVE                         SPI_PUSHR_SLAVE_REG(SPI1)
//#define SPI1_POPR                                SPI_POPR_REG(SPI1)
//#define SPI1_TXFR0                               SPI_TXFR0_REG(SPI1)
//#define SPI1_TXFR1                               SPI_TXFR1_REG(SPI1)
//#define SPI1_TXFR2                               SPI_TXFR2_REG(SPI1)
//#define SPI1_TXFR3                               SPI_TXFR3_REG(SPI1)
//#define SPI1_RXFR0                               SPI_RXFR0_REG(SPI1)
//#define SPI1_RXFR1                               SPI_RXFR1_REG(SPI1)
//#define SPI1_RXFR2                               SPI_RXFR2_REG(SPI1)
//#define SPI1_RXFR3                               SPI_RXFR3_REG(SPI1)
//
///* SPI - Register array accessors */
//#define SPI0_CTAR(index2)                        SPI_CTAR_REG(SPI0,index2)
//#define SPI1_CTAR(index2)                        SPI_CTAR_REG(SPI1,index2)
//#define SPI0_CTAR_SLAVE(index2)                  SPI_CTAR_SLAVE_REG(SPI0,index2)
//#define SPI1_CTAR_SLAVE(index2)                  SPI_CTAR_SLAVE_REG(SPI1,index2)
//
///*!
// * @}
// */ /* end of group SPI_Register_Accessor_Macros */

/*!
 * @}
 */ /* end of group SPI_Peripheral_Access_Layer */

#endif /* __MK22F51212_SPI_H__ */