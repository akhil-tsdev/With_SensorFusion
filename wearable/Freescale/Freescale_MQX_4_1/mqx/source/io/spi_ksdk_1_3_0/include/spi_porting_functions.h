/*
 * Copyright TuringSense, Inc © 2015
 * spi_porting_functions.h
 *
 *  Created on: July 30, 2015
 *      Author: cwati
 *
 *
 * We found that SPI driver from MQX 4.1 is not working quite as expected,
 * we decided to port the SPI driver from KSDK 1.3.0
 * For that, we use this header file to port several files including:
 * from KSDK 1.3.0, MK22FA12.h & MK22FA12_extension.h
 * From KSDK 1.0 core_cm4.h
 * fsl_clock_K22F51212.h
 */

#ifndef SPI_PORTING_FUNCTIONS_H_
#define SPI_PORTING_FUNCTIONS_H_


#if (HUB_22F && PRODUCTION1)
#include "MK22FA12_spi.h"
#include "MK22FA12_extension_spi.h"
#elif ((EVAL_BOARD && HUB_22F) || (SAT_22F && PRODUCTION1))
#include "MK22F51212_spi.h"
#include "MK22F51212_extension_spi.h"
#endif

//#include <spi.h>
//#include <bsp.h>
//#include "bsp_prv.h"

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


/* From fsl_clock_K22F51212.h */
uint32_t    CLOCK_SYS_GetSpiFreq(uint32_t instance);
void SIM_HAL_EnableSpiClock(uint32_t instance);
void SIM_HAL_DisableSpiClock(uint32_t instance);

/*!
 * @brief Enable the clock for SPI module.
 *
 * This function enables the clock for SPI moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_EnableSpiClock(uint32_t instance)
{
    SIM_HAL_EnableSpiClock(instance);
}

/*!
 * @brief Disable the clock for SPI module.
 *
 * This function disables the clock for SPI moudle.
 * @param instance module device instance
 */
static inline void CLOCK_SYS_DisableSpiClock(uint32_t instance)
{
    SIM_HAL_DisableSpiClock(instance);
}

/* End of From fsl_clock_K22F51212.h */






































//#if PRODUCTION1
//#include "MK21FA12_spi.h"
//
//#define	HW_SPI_INSTANCE_COUNT					SPI_INSTANCE_COUNT
//#define	BM_SPI_SR_TCF	SPI_SR_TCF_MASK
//#define	BM_SPI_SR_EOQF	SPI_SR_EOQF_MASK
//#define	BM_SPI_SR_TFUF	SPI_SR_TFUF_MASK
//#define	BM_SPI_SR_TFFF	SPI_SR_TFFF_MASK
//#define	BM_SPI_SR_RFOF	SPI_SR_RFOF_MASK
//#define	BM_SPI_SR_RFDF	SPI_SR_RFDF_MASK
//
//#define BM_SPI_MCR_HALT	SPI_MCR_HALT_MASK
//#define BM_SPI_MCR_MDIS	SPI_MCR_MDIS_MASK
///* The following is from
// * C:\Freescale\KSDK_1.3.0\platform\devices\MK21FA12\include\MK21FA12.h
// * rev. 1.1, 2015-02-19
// */
//
///* SPI - Peripheral instance base addresses */
///** Peripheral SPI0 base address */
//#define SPI0_BASE                                (0x4002C000u)
///** Peripheral SPI0 base pointer */
//#define SPI0                                     ((SPI_Type *)SPI0_BASE)
////#define SPI0_BASE_PTR                            (SPI0)
///** Peripheral SPI1 base address */
//#define SPI1_BASE                                (0x4002D000u)
///** Peripheral SPI1 base pointer */
//#define SPI1                                     ((SPI_Type *)SPI1_BASE)
////#define SPI1_BASE_PTR                            (SPI1)
///** Peripheral SPI2 base address */
//#define SPI2_BASE                                (0x400AC000u)
///** Peripheral SPI2 base pointer */
//#define SPI2                                     ((SPI_Type *)SPI2_BASE)
////#define SPI2_BASE_PTR                            (SPI2)
///** Array initializer of SPI peripheral base addresses */
//#define SPI_BASE_ADDRS                           { SPI0_BASE, SPI1_BASE, SPI2_BASE }
///** Array initializer of SPI peripheral base pointers */
////#define SPI_BASE_PTRS                            { SPI0, SPI1, SPI2 }
///** Interrupt vectors for the SPI peripheral type */
//#define SPI_IRQS                                 { SPI0_IRQn, SPI1_IRQn, SPI2_IRQn }
//
///* ----------------------------------------------------------------------------
//   -- Interrupt vector numbers
//   ---------------------------------------------------------------------------- */
//
///*!
// * @addtogroup Interrupt_vector_numbers Interrupt vector numbers
// * @{
// */
//
///** Interrupt Number Definitions */
//#define NUMBER_OF_INT_VECTORS 98                 /**< Number of interrupts in the Vector table */
//
//typedef enum IRQn {
//  /* Auxiliary constants */
//  NotAvail_IRQn                = -128,             /**< Not available device specific interrupt */
//
//  /* Core interrupts */
//  NonMaskableInt_IRQn          = -14,              /**< Non Maskable Interrupt */
//  HardFault_IRQn               = -13,              /**< Cortex-M4 SV Hard Fault Interrupt */
//  MemoryManagement_IRQn        = -12,              /**< Cortex-M4 Memory Management Interrupt */
//  BusFault_IRQn                = -11,              /**< Cortex-M4 Bus Fault Interrupt */
//  UsageFault_IRQn              = -10,              /**< Cortex-M4 Usage Fault Interrupt */
//  SVCall_IRQn                  = -5,               /**< Cortex-M4 SV Call Interrupt */
//  DebugMonitor_IRQn            = -4,               /**< Cortex-M4 Debug Monitor Interrupt */
//  PendSV_IRQn                  = -2,               /**< Cortex-M4 Pend SV Interrupt */
//  SysTick_IRQn                 = -1,               /**< Cortex-M4 System Tick Interrupt */
//
//  /* Device specific interrupts */
//  DMA0_IRQn                    = 0,                /**< DMA Channel 0 Transfer Complete */
//  DMA1_IRQn                    = 1,                /**< DMA Channel 1 Transfer Complete */
//  DMA2_IRQn                    = 2,                /**< DMA Channel 2 Transfer Complete */
//  DMA3_IRQn                    = 3,                /**< DMA Channel 3 Transfer Complete */
//  DMA4_IRQn                    = 4,                /**< DMA Channel 4 Transfer Complete */
//  DMA5_IRQn                    = 5,                /**< DMA Channel 5 Transfer Complete */
//  DMA6_IRQn                    = 6,                /**< DMA Channel 6 Transfer Complete */
//  DMA7_IRQn                    = 7,                /**< DMA Channel 7 Transfer Complete */
//  DMA8_IRQn                    = 8,                /**< DMA Channel 8 Transfer Complete */
//  DMA9_IRQn                    = 9,                /**< DMA Channel 9 Transfer Complete */
//  DMA10_IRQn                   = 10,               /**< DMA Channel 10 Transfer Complete */
//  DMA11_IRQn                   = 11,               /**< DMA Channel 11 Transfer Complete */
//  DMA12_IRQn                   = 12,               /**< DMA Channel 12 Transfer Complete */
//  DMA13_IRQn                   = 13,               /**< DMA Channel 13 Transfer Complete */
//  DMA14_IRQn                   = 14,               /**< DMA Channel 14 Transfer Complete */
//  DMA15_IRQn                   = 15,               /**< DMA Channel 15 Transfer Complete */
//  DMA_Error_IRQn               = 16,               /**< DMA Error Interrupt */
//  MCM_IRQn                     = 17,               /**< Normal Interrupt */
//  FTFE_IRQn                    = 18,               /**< FTFE Command complete interrupt */
//  Read_Collision_IRQn          = 19,               /**< Read Collision Interrupt */
//  LVD_LVW_IRQn                 = 20,               /**< Low Voltage Detect, Low Voltage Warning */
//  LLWU_IRQn                    = 21,               /**< Low Leakage Wakeup Unit */
//  WDOG_EWM_IRQn                = 22,               /**< WDOG Interrupt */
//  RNG_IRQn                     = 23,               /**< RNG Interrupt */
//  I2C0_IRQn                    = 24,               /**< I2C0 interrupt */
//  I2C1_IRQn                    = 25,               /**< I2C1 interrupt */
//  SPI0_IRQn                    = 26,               /**< SPI0 Interrupt */
//  SPI1_IRQn                    = 27,               /**< SPI1 Interrupt */
//  I2S0_Tx_IRQn                 = 28,               /**< I2S0 transmit interrupt */
//  I2S0_Rx_IRQn                 = 29,               /**< I2S0 receive interrupt */
//  Reserved46_IRQn              = 30,               /**< Reserved interrupt 46 */
//  UART0_RX_TX_IRQn             = 31,               /**< UART0 Receive/Transmit interrupt */
//  UART0_ERR_IRQn               = 32,               /**< UART0 Error interrupt */
//  UART1_RX_TX_IRQn             = 33,               /**< UART1 Receive/Transmit interrupt */
//  UART1_ERR_IRQn               = 34,               /**< UART1 Error interrupt */
//  UART2_RX_TX_IRQn             = 35,               /**< UART2 Receive/Transmit interrupt */
//  UART2_ERR_IRQn               = 36,               /**< UART2 Error interrupt */
//  UART3_RX_TX_IRQn             = 37,               /**< UART3 Receive/Transmit interrupt */
//  UART3_ERR_IRQn               = 38,               /**< UART3 Error interrupt */
//  ADC0_IRQn                    = 39,               /**< ADC0 interrupt */
//  CMP0_IRQn                    = 40,               /**< CMP0 interrupt */
//  CMP1_IRQn                    = 41,               /**< CMP1 interrupt */
//  FTM0_IRQn                    = 42,               /**< FTM0 fault, overflow and channels interrupt */
//  FTM1_IRQn                    = 43,               /**< FTM1 fault, overflow and channels interrupt */
//  FTM2_IRQn                    = 44,               /**< FTM2 fault, overflow and channels interrupt */
//  CMT_IRQn                     = 45,               /**< CMT interrupt */
//  RTC_IRQn                     = 46,               /**< RTC interrupt */
//  RTC_Seconds_IRQn             = 47,               /**< RTC seconds interrupt */
//  PIT0_IRQn                    = 48,               /**< PIT timer channel 0 interrupt */
//  PIT1_IRQn                    = 49,               /**< PIT timer channel 1 interrupt */
//  PIT2_IRQn                    = 50,               /**< PIT timer channel 2 interrupt */
//  PIT3_IRQn                    = 51,               /**< PIT timer channel 3 interrupt */
//  PDB0_IRQn                    = 52,               /**< PDB0 Interrupt */
//  USB0_IRQn                    = 53,               /**< USB0 interrupt */
//  USBDCD_IRQn                  = 54,               /**< USBDCD Interrupt */
//  Reserved71_IRQn              = 55,               /**< Reserved interrupt 71 */
//  DAC0_IRQn                    = 56,               /**< DAC0 interrupt */
//  MCG_IRQn                     = 57,               /**< MCG Interrupt */
//  LPTMR0_IRQn                  = 58,               /**< LPTimer interrupt */
//  PORTA_IRQn                   = 59,               /**< Port A interrupt */
//  PORTB_IRQn                   = 60,               /**< Port B interrupt */
//  PORTC_IRQn                   = 61,               /**< Port C interrupt */
//  PORTD_IRQn                   = 62,               /**< Port D interrupt */
//  PORTE_IRQn                   = 63,               /**< Port E interrupt */
//  SWI_IRQn                     = 64,               /**< Software interrupt */
//  SPI2_IRQn                    = 65,               /**< SPI2 Interrupt */
//  UART4_RX_TX_IRQn             = 66,               /**< UART4 Receive/Transmit interrupt */
//  UART4_ERR_IRQn               = 67,               /**< UART4 Error interrupt */
//  UART5_RX_TX_IRQn             = 68,               /**< UART5 Receive/Transmit interrupt */
//  UART5_ERR_IRQn               = 69,               /**< UART5 Error interrupt */
//  CMP2_IRQn                    = 70,               /**< CMP2 interrupt */
//  FTM3_IRQn                    = 71,               /**< FTM3 fault, overflow and channels interrupt */
//  DAC1_IRQn                    = 72,               /**< DAC1 interrupt */
//  ADC1_IRQn                    = 73,               /**< ADC1 interrupt */
//  I2C2_IRQn                    = 74,               /**< I2C2 interrupt */
//  CAN0_ORed_Message_buffer_IRQn = 75,              /**< CAN0 OR'd message buffers interrupt */
//  CAN0_Bus_Off_IRQn            = 76,               /**< CAN0 bus off interrupt */
//  CAN0_Error_IRQn              = 77,               /**< CAN0 error interrupt */
//  CAN0_Tx_Warning_IRQn         = 78,               /**< CAN0 Tx warning interrupt */
//  CAN0_Rx_Warning_IRQn         = 79,               /**< CAN0 Rx warning interrupt */
//  CAN0_Wake_Up_IRQn            = 80,               /**< CAN0 wake up interrupt */
//  SDHC_IRQn                    = 81                /**< SDHC interrupt */
//} IRQn_Type;
//
///*!
// * @}
// */ /* end of group Interrupt_vector_numbers */
//
//#elif EVAL_BOARD
//
///* The following from MK22F51212.h Rev 1 March 24, 2014 */
//
///* SPI - Peripheral instance base addresses */
///** Peripheral SPI0 base address */
//#define SPI0_BASE                                (0x4002C000u)
///** Peripheral SPI0 base pointer */
//#define SPI0                                     ((SPI_Type *)SPI0_BASE)
///** Peripheral SPI1 base address */
//#define SPI1_BASE                                (0x4002D000u)
///** Peripheral SPI1 base pointer */
//#define SPI1                                     ((SPI_Type *)SPI1_BASE)
///** Array initializer of SPI peripheral base addresses */
//#define SPI_BASE_ADDRS                           { SPI0_BASE, SPI1_BASE }
//
///** Interrupt vectors for the SPI peripheral type */
//#define SPI_IRQS                                 { SPI0_IRQn, SPI1_IRQn }
//
//
///** Interrupt Number Definitions */
//#define NUMBER_OF_INT_VECTORS 102                /**< Number of interrupts in the Vector table */
//
//typedef enum IRQn {
//  /* Core interrupts */
//  NonMaskableInt_IRQn          = -14,              /**< Non Maskable Interrupt */
//  HardFault_IRQn               = -13,              /**< Cortex-M4 SV Hard Fault Interrupt */
//  MemoryManagement_IRQn        = -12,              /**< Cortex-M4 Memory Management Interrupt */
//  BusFault_IRQn                = -11,              /**< Cortex-M4 Bus Fault Interrupt */
//  UsageFault_IRQn              = -10,              /**< Cortex-M4 Usage Fault Interrupt */
//  SVCall_IRQn                  = -5,               /**< Cortex-M4 SV Call Interrupt */
//  DebugMonitor_IRQn            = -4,               /**< Cortex-M4 Debug Monitor Interrupt */
//  PendSV_IRQn                  = -2,               /**< Cortex-M4 Pend SV Interrupt */
//  SysTick_IRQn                 = -1,               /**< Cortex-M4 System Tick Interrupt */
//
//  /* Device specific interrupts */
//  DMA0_IRQn                    = 0,                /**< DMA Channel 0 Transfer Complete */
//  DMA1_IRQn                    = 1,                /**< DMA Channel 1 Transfer Complete */
//  DMA2_IRQn                    = 2,                /**< DMA Channel 2 Transfer Complete */
//  DMA3_IRQn                    = 3,                /**< DMA Channel 3 Transfer Complete */
//  DMA4_IRQn                    = 4,                /**< DMA Channel 4 Transfer Complete */
//  DMA5_IRQn                    = 5,                /**< DMA Channel 5 Transfer Complete */
//  DMA6_IRQn                    = 6,                /**< DMA Channel 6 Transfer Complete */
//  DMA7_IRQn                    = 7,                /**< DMA Channel 7 Transfer Complete */
//  DMA8_IRQn                    = 8,                /**< DMA Channel 8 Transfer Complete */
//  DMA9_IRQn                    = 9,                /**< DMA Channel 9 Transfer Complete */
//  DMA10_IRQn                   = 10,               /**< DMA Channel 10 Transfer Complete */
//  DMA11_IRQn                   = 11,               /**< DMA Channel 11 Transfer Complete */
//  DMA12_IRQn                   = 12,               /**< DMA Channel 12 Transfer Complete */
//  DMA13_IRQn                   = 13,               /**< DMA Channel 13 Transfer Complete */
//  DMA14_IRQn                   = 14,               /**< DMA Channel 14 Transfer Complete */
//  DMA15_IRQn                   = 15,               /**< DMA Channel 15 Transfer Complete */
//  DMA_Error_IRQn               = 16,               /**< DMA Error Interrupt */
//  MCM_IRQn                     = 17,               /**< Normal Interrupt */
//  FTF_IRQn                     = 18,               /**< FTFA Command complete interrupt */
//  Read_Collision_IRQn          = 19,               /**< Read Collision Interrupt */
//  LVD_LVW_IRQn                 = 20,               /**< Low Voltage Detect, Low Voltage Warning */
//  LLW_IRQn                     = 21,               /**< Low Leakage Wakeup */
//  Watchdog_IRQn                = 22,               /**< WDOG Interrupt */
//  RNG_IRQn                     = 23,               /**< RNG Interrupt */
//  I2C0_IRQn                    = 24,               /**< I2C0 interrupt */
//  I2C1_IRQn                    = 25,               /**< I2C1 interrupt */
//  SPI0_IRQn                    = 26,               /**< SPI0 Interrupt */
//  SPI1_IRQn                    = 27,               /**< SPI1 Interrupt */
//  I2S0_Tx_IRQn                 = 28,               /**< I2S0 transmit interrupt */
//  I2S0_Rx_IRQn                 = 29,               /**< I2S0 receive interrupt */
//  LPUART0_IRQn                 = 30,               /**< LPUART0 status/error interrupt */
//  UART0_RX_TX_IRQn             = 31,               /**< UART0 Receive/Transmit interrupt */
//  UART0_ERR_IRQn               = 32,               /**< UART0 Error interrupt */
//  UART1_RX_TX_IRQn             = 33,               /**< UART1 Receive/Transmit interrupt */
//  UART1_ERR_IRQn               = 34,               /**< UART1 Error interrupt */
//  UART2_RX_TX_IRQn             = 35,               /**< UART2 Receive/Transmit interrupt */
//  UART2_ERR_IRQn               = 36,               /**< UART2 Error interrupt */
//  Reserved53_IRQn              = 37,               /**< Reserved interrupt 53 */
//  Reserved54_IRQn              = 38,               /**< Reserved interrupt 54 */
//  ADC0_IRQn                    = 39,               /**< ADC0 interrupt */
//  CMP0_IRQn                    = 40,               /**< CMP0 interrupt */
//  CMP1_IRQn                    = 41,               /**< CMP1 interrupt */
//  FTM0_IRQn                    = 42,               /**< FTM0 fault, overflow and channels interrupt */
//  FTM1_IRQn                    = 43,               /**< FTM1 fault, overflow and channels interrupt */
//  FTM2_IRQn                    = 44,               /**< FTM2 fault, overflow and channels interrupt */
//  Reserved61_IRQn              = 45,               /**< Reserved interrupt 61 */
//  RTC_IRQn                     = 46,               /**< RTC interrupt */
//  RTC_Seconds_IRQn             = 47,               /**< RTC seconds interrupt */
//  PIT0_IRQn                    = 48,               /**< PIT timer channel 0 interrupt */
//  PIT1_IRQn                    = 49,               /**< PIT timer channel 1 interrupt */
//  PIT2_IRQn                    = 50,               /**< PIT timer channel 2 interrupt */
//  PIT3_IRQn                    = 51,               /**< PIT timer channel 3 interrupt */
//  PDB0_IRQn                    = 52,               /**< PDB0 Interrupt */
//  USB0_IRQn                    = 53,               /**< USB0 interrupt */
//  Reserved70_IRQn              = 54,               /**< Reserved interrupt 70 */
//  Reserved71_IRQn              = 55,               /**< Reserved interrupt 71 */
//  DAC0_IRQn                    = 56,               /**< DAC0 interrupt */
//  MCG_IRQn                     = 57,               /**< MCG Interrupt */
//  LPTimer_IRQn                 = 58,               /**< LPTimer interrupt */
//  PORTA_IRQn                   = 59,               /**< Port A interrupt */
//  PORTB_IRQn                   = 60,               /**< Port B interrupt */
//  PORTC_IRQn                   = 61,               /**< Port C interrupt */
//  PORTD_IRQn                   = 62,               /**< Port D interrupt */
//  PORTE_IRQn                   = 63,               /**< Port E interrupt */
//  SWI_IRQn                     = 64,               /**< Software interrupt */
//  Reserved81_IRQn              = 65,               /**< Reserved interrupt 81 */
//  Reserved82_IRQn              = 66,               /**< Reserved interrupt 82 */
//  Reserved83_IRQn              = 67,               /**< Reserved interrupt 83 */
//  Reserved84_IRQn              = 68,               /**< Reserved interrupt 84 */
//  Reserved85_IRQn              = 69,               /**< Reserved interrupt 85 */
//  Reserved86_IRQn              = 70,               /**< Reserved interrupt 86 */
//  FTM3_IRQn                    = 71,               /**< FTM3 fault, overflow and channels interrupt */
//  DAC1_IRQn                    = 72,               /**< DAC1 interrupt */
//  ADC1_IRQn                    = 73,               /**< ADC1 interrupt */
//  Reserved90_IRQn              = 74,               /**< Reserved Interrupt 90 */
//  Reserved91_IRQn              = 75,               /**< Reserved Interrupt 91 */
//  Reserved92_IRQn              = 76,               /**< Reserved Interrupt 92 */
//  Reserved93_IRQn              = 77,               /**< Reserved Interrupt 93 */
//  Reserved94_IRQn              = 78,               /**< Reserved Interrupt 94 */
//  Reserved95_IRQn              = 79,               /**< Reserved Interrupt 95 */
//  Reserved96_IRQn              = 80,               /**< Reserved Interrupt 96 */
//  Reserved97_IRQn              = 81,               /**< Reserved Interrupt 97 */
//  Reserved98_IRQn              = 82,               /**< Reserved Interrupt 98 */
//  Reserved99_IRQn              = 83,               /**< Reserved Interrupt 99 */
//  Reserved100_IRQn             = 84,               /**< Reserved Interrupt 100 */
//  Reserved101_IRQn             = 85                /**< Reserved Interrupt 101 */
//} IRQn_Type;
//
//#endif /* EVAL_BOARD */

#endif /* SPI_PORTING_FUNCTIONS_H_ */
