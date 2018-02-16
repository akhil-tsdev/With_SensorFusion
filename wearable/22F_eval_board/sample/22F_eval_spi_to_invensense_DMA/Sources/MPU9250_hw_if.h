/*
 * MPU9250_hw_if.h
 *
 *  Created on: Dec 31, 2014
 *      Author: cwati
 */

#ifndef MPU9250_HW_IF_H_
#define MPU9250_HW_IF_H_
#include <stdint.h>

if defined(22F51212)
//tmp from "MK22F51212.h"
// write to ptd4
#define PORTD_BASE                               (0x4004C000u)

//tmp   from MK22F51212_port.h
typedef union _hw_port_pcrn
{
    uint32_t U;
    struct _hw_port_pcrn_bitfields
    {
        uint32_t PS : 1;               /*!< [0] Pull Select */
        uint32_t PE : 1;               /*!< [1] Pull Enable */
        uint32_t SRE : 1;              /*!< [2] Slew Rate Enable */
        uint32_t RESERVED0 : 1;        /*!< [3]  */
        uint32_t PFE : 1;              /*!< [4] Passive Filter Enable */
        uint32_t ODE : 1;              /*!< [5] Open Drain Enable */
        uint32_t DSE : 1;              /*!< [6] Drive Strength Enable */
        uint32_t RESERVED1 : 1;        /*!< [7]  */
        uint32_t MUX : 3;              /*!< [10:8] Pin Mux Control */
        uint32_t RESERVED2 : 4;        /*!< [14:11]  */
        uint32_t LK : 1;               /*!< [15] Lock Register */
        uint32_t IRQC : 4;             /*!< [19:16] Interrupt Configuration */
        uint32_t RESERVED3 : 4;        /*!< [23:20]  */
        uint32_t ISF : 1;              /*!< [24] Interrupt Status Flag */
        uint32_t RESERVED4 : 7;        /*!< [31:25]  */
    } B;
} hw_port_pcrn_t;

#define HW_PORT_PCRn_ADDR(x, n)  ((x) + 0x0U + (0x4U * (n)))
#define HW_PORT_PCRn(x, n)       (*(volatile hw_port_pcrn_t *) HW_PORT_PCRn_ADDR(x, n))
#define HW_PORT_PCRn_RD(x, n)    (HW_PORT_PCRn(x, n).U)
#define HW_PORT_PCRn_WR(x, n, v) (HW_PORT_PCRn(x, n).U = (v))
#define HW_PORT_PCRn_SET(x, n, v) (HW_PORT_PCRn_WR(x, n, HW_PORT_PCRn_RD(x, n) |  (v)))
#define HW_PORT_PCRn_CLR(x, n, v) (HW_PORT_PCRn_WR(x, n, HW_PORT_PCRn_RD(x, n) & ~(v)))
#define HW_PORT_PCRn_TOG(x, n, v) (HW_PORT_PCRn_WR(x, n, HW_PORT_PCRn_RD(x, n) ^  (v)))

/* CS is PTD4 */
#define CS_WR(v) HW_PORT_PCRn_WR(PORTD_BASE, 4u, v)
#define CS_RD() HW_PORT_PCRn_RD(PORTD_BASE, 4u)


/* from MK22F51212_spi.h */
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
#define BP_SPI_TCR_SPI_TCNT  (16U)         /*!< Bit position for SPI_TCR_SPI_TCNT. */
#define BM_SPI_TCR_SPI_TCNT  (0xFFFF0000U) /*!< Bit mask for SPI_TCR_SPI_TCNT. */
#define BS_SPI_TCR_SPI_TCNT  (16U)         /*!< Bit field size in bits for SPI_TCR_SPI_TCNT. */

/*! @brief Read current value of the SPI_TCR_SPI_TCNT field. */
#define BR_SPI_TCR_SPI_TCNT(x) (HW_SPI_TCR(x).B.SPI_TCNT)

/*! @brief Format value for bitfield SPI_TCR_SPI_TCNT. */
#define BF_SPI_TCR_SPI_TCNT(v) ((uint32_t)((uint32_t)(v) << BP_SPI_TCR_SPI_TCNT) & BM_SPI_TCR_SPI_TCNT)

/*! @brief Set the SPI_TCNT field to a new value. */
#define BW_SPI_TCR_SPI_TCNT(x, v) (HW_SPI_TCR_WR(x, (HW_SPI_TCR_RD(x) & ~BM_SPI_TCR_SPI_TCNT) | BF_SPI_TCR_SPI_TCNT(v)))
/*@}*/


/*
SPI0_nCS1	J6.4	PTD4
SPI0_SCLK	J6.5	PTD1
SPI0_MOSI	J6.6	PTD2
SPI0_MISO	J6.7	PTD */

#endif /* 22F */

#endif /* MPU9250_HW_IF_H_ */
