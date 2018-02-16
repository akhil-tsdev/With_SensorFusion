/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
*
* @defgroup nrf_radio_test_example_main main.c
* @{
* @ingroup nrf_radio_test_example
* @brief Radio Test Example Application main file.
*
* This file contains the source code for a sample application using the NRF_RADIO, and is controlled through the serial port.
*
*/


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "bsp.h"
#include "nrf.h"
#include "radio_test.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf51_bitfields.h"
#include "nordic_common.h"
#include "crc16.h"
#include "spi_slave_radio_test.h"

uint8_t mode_          = RADIO_MODE_MODE_Nrf_2Mbit;
uint8_t txpower_       = RADIO_TXPOWER_TXPOWER_0dBm;
uint8_t channel_start_     = 0;
uint8_t channel_end_       = 80;
uint8_t delayms_           = 10;

static bool sweep = false;

typedef enum
{
    RADIO_TEST_NOP,      /**< No test running.      */
    RADIO_TEST_TXCC,     /**< TX constant carrier.  */
    RADIO_TEST_TXMC,     /**< TX modulated carrier. */
    RADIO_TEST_TXSWEEP,  /**< TX sweep.             */
    RADIO_TEST_RXC,      /**< RX constant carrier.  */
    RADIO_TEST_RXSWEEP,  /**< RX sweep.             */
} radio_tests_t;

/** @brief Function for configuring all peripherals used in this example.
*/
static void crystal_init(void)
{
    NRF_RNG->TASKS_START = 1;
    
    // Start 16 MHz crystal oscillator
    NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_HFCLKSTART     = 1;

    // Wait for the external oscillator to start up
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }  
}


///** @brief Function for outputting usage info to the serial port.
//*/
//static void help(void)
//{
//    printf("Usage:\r\n");
//    printf("a: Enter start channel for sweep/channel for constant carrier\r\n");
//    printf("b: Enter end channel for sweep\r\n");
//    printf("c: Start TX carrier\r\n");
//    printf("d: Enter time on each channel (1ms-99ms)\r\n");
//    printf("e: Cancel sweep/carrier\r\n");
//    printf("m: Enter data rate\r\n");
//    printf("o: Start modulated TX carrier\r\n");
//    printf("p: Enter output power\r\n");
//    printf("s: Print current delay, channels and so on\r\n");
//    printf("r: Start RX sweep\r\n");
//    printf("t: Start TX sweep\r\n");
//    printf("x: Start RX carrier\r\n");
//}


/** @brief Function for reading the data rate.
*/
void set_datarate(void)
{
    uint8_t c;

		c = spi_rx.mode;
    if (c == 0)
    {
        mode_ = RADIO_MODE_MODE_Nrf_250Kbit;
    }
    else if (c == 1)
    {
        mode_ = RADIO_MODE_MODE_Nrf_1Mbit;
    }
    else
    {
        mode_ = RADIO_MODE_MODE_Nrf_2Mbit;
    }
}

uint8_t get_datarate(void)
{
    uint8_t c, ret;

		c = mode_;
    if (c == RADIO_MODE_MODE_Nrf_250Kbit)
    {
        ret = 0;
    }
    else if (c == RADIO_MODE_MODE_Nrf_1Mbit)
    {
        ret = 1;
    }
    else
    {
        ret = 2;
    }
		
		return ret;
}

/** @brief Function for reading the output power.
*/
void set_power(void)
{
    uint8_t c;

    c = spi_rx.power;
    switch(c)
    {
        case 0:
            txpower_ =  RADIO_TXPOWER_TXPOWER_Pos4dBm;
            break;
        
        case 1:
            txpower_ =  RADIO_TXPOWER_TXPOWER_0dBm;
            break;
        
        case 2:
            txpower_ = RADIO_TXPOWER_TXPOWER_Neg4dBm;
            break;
        
        case 3:
            txpower_ = RADIO_TXPOWER_TXPOWER_Neg8dBm;
            break;
        
        case 4:
            txpower_ = RADIO_TXPOWER_TXPOWER_Neg12dBm;
            break;
        
        case 5:
            txpower_ = RADIO_TXPOWER_TXPOWER_Neg16dBm;
            break;
        
        case 6:
            txpower_ = RADIO_TXPOWER_TXPOWER_Neg20dBm;
            break;
        
        case 7:
            // fall through 
        
        default:
            txpower_ = RADIO_TXPOWER_TXPOWER_Neg30dBm;
            break;
    }
}

uint8_t get_tx_power(void)
{
    uint8_t c, ret = 7;

    c = txpower_;
    switch(c)
    {
        case RADIO_TXPOWER_TXPOWER_Pos4dBm:
						ret =  0;
            break;
        
        case RADIO_TXPOWER_TXPOWER_0dBm:
            ret =  1;
            break;
        
        case RADIO_TXPOWER_TXPOWER_Neg4dBm:
            ret = 2;
            break;
        
        case RADIO_TXPOWER_TXPOWER_Neg8dBm:
            ret = 3;
            break;
        
        case RADIO_TXPOWER_TXPOWER_Neg12dBm:
            ret = 4;
            break;
        
        case RADIO_TXPOWER_TXPOWER_Neg16dBm:
            ret = 5;
            break;
        
        case RADIO_TXPOWER_TXPOWER_Neg20dBm:
            ret = 6;
            break;
        
        case RADIO_TXPOWER_TXPOWER_Neg30dBm:
            // fall through 
        
        default:
            ret = 7;
            break;
    }
		
		return ret;
}
///** @brief Function for printing parameters to the serial port.
//*/
//void print_parameters(void)
//{
//    printf("Parameters:\r\n");
//    switch(mode_)
//    {
//        case RADIO_MODE_MODE_Nrf_250Kbit:
//            printf("Data rate...........: 250 Kbit/s\r\n");
//            break;
//        
//        case RADIO_MODE_MODE_Nrf_1Mbit:
//            printf("Data rate...........: 1 Mbit/s\r\n");
//            break;
//        
//        case RADIO_MODE_MODE_Nrf_2Mbit:
//            printf("Data rate...........: 2 Mbit/s\r\n");
//            break;
//    }
//    
//    switch(txpower_)
//    {
//        case RADIO_TXPOWER_TXPOWER_Pos4dBm:
//            printf("TX Power............: +4 dBm\r\n");
//            break;
//        
//        case RADIO_TXPOWER_TXPOWER_0dBm:
//            printf("TX Power............: 0 dBm\r\n");
//            break;
//        
//        case RADIO_TXPOWER_TXPOWER_Neg4dBm:
//            printf("TX Power............: -4 dBm\r\n");
//            break;
//        
//        case RADIO_TXPOWER_TXPOWER_Neg8dBm:
//            printf("TX Power............: -8 dBm\r\n");
//            break;
//        
//        case RADIO_TXPOWER_TXPOWER_Neg12dBm:
//            printf("TX Power............: -12 dBm\r\n");
//            break;
//        
//        case RADIO_TXPOWER_TXPOWER_Neg16dBm:
//            printf("TX Power............: -16 dBm\r\n");
//            break;
//        
//        case RADIO_TXPOWER_TXPOWER_Neg20dBm:
//            printf("TX Power............: -20 dBm\r\n");
//            break;
//        
//        case RADIO_TXPOWER_TXPOWER_Neg30dBm:
//            printf("TX Power............: -30 dBm\r\n");
//            break;
//        
//        default:
//            // No implementation needed.
//            break;
//        
//    }
//    printf("(Start) Channel.....: %d\r\n",channel_start_);
//    printf("End Channel.........: %d\r\n",channel_end_);
//    printf("Time on each channel: %d\r\n",delayms_);
//    printf(" ms\r\n");
//}

void process_command(void) {
	static radio_tests_t test     = RADIO_TEST_NOP;
	static radio_tests_t cur_test = RADIO_TEST_NOP;

	switch (spi_rx.command)
	{
			case 'a':
					channel_start_ = spi_rx.start_channel;
					test = cur_test;
					break;

			case 'b':
					channel_end_ = spi_rx.end_channel;
					test = cur_test;
					break;

			case 'c':
					test = RADIO_TEST_TXCC;
					break;

			case 'd':
					delayms_ = spi_rx.delayms;
					test = cur_test;
					break;

			case 'e':
					radio_sweep_end();
					cur_test = RADIO_TEST_NOP;
					break;

			case 'm':
					set_datarate();
					test = cur_test;
					break;

			case 'o':
					test = RADIO_TEST_TXMC;
					//printf("TX modulated carrier\r\n");
					break;

			case 'p':
					set_power();
					test = cur_test;
					break;

			case 'r':
					test = RADIO_TEST_RXSWEEP;
					//printf("RX Sweep\r\n");
					break;

//			case 's':
//					print_parameters();
//					break;

			case 't':
					test = RADIO_TEST_TXSWEEP;
					//printf("TX Sweep\r\n");
					break;

			case 'x':
					test = RADIO_TEST_RXC;
					//printf("RX constant carrier\r\n");
					break;

			//case 'h': print help
			default:
					break;
	}

	switch (test)
	{
			case RADIO_TEST_TXCC:
					if (sweep)
					{
							radio_sweep_end();
							sweep = false;
					}
					radio_tx_carrier(txpower_, mode_, channel_start_);
					cur_test = test;
					test     = RADIO_TEST_NOP;
					break;

			case RADIO_TEST_TXMC:
					if (sweep)
					{
							radio_sweep_end();
							sweep = false;
					}
					radio_modulated_tx_carrier(txpower_, mode_, channel_start_);
					cur_test = test;
					test     = RADIO_TEST_NOP;
					break;

			case RADIO_TEST_TXSWEEP:
					radio_tx_sweep_start(txpower_, mode_, channel_start_, channel_end_, delayms_);
					sweep    = true;
					cur_test = test;
					test     = RADIO_TEST_NOP;
					break;

			case RADIO_TEST_RXC:
					if (sweep)
					{
							radio_sweep_end();
							sweep = false;
					}
					radio_rx_carrier(mode_, channel_start_);
					cur_test = test;
					test     = RADIO_TEST_NOP;
					break;  

			case RADIO_TEST_RXSWEEP:
					radio_rx_sweep_start(mode_, channel_start_, channel_end_, delayms_);
					sweep    = true;
					cur_test = test;
					test     = RADIO_TEST_NOP;
					break;

			case RADIO_TEST_NOP:
					// Fall through.
			default:
					// No implementation needed.
					break;
	}	

}

void prep_tx_crc16() {
	/* Issue with first byte sent from Nordic to hub always has the first bit set */
	spi_tx.padding |= 0x80;
	spi_tx.mode = get_datarate();
	spi_tx.tx_power = get_tx_power();
	spi_tx.channel_start = channel_start_;
	spi_tx.channel_end = channel_end_;
	spi_tx.delayms = delayms_;
	spi_tx.crc16 = 0;
	spi_tx.crc16 = crc16_compute((uint8_t*)(&spi_tx), sizeof(spi_tx), 0);
}

/** @brief Function for main application entry.
 */
int main(void)
{ 
    crystal_init();
		spi_slave_example_init();

		NVIC_EnableIRQ(TIMER0_IRQn);
    __enable_irq();
    while (true)
    {

			 //Let the CPU sleep until the next interrupt comes along
			__WFE();
			__SEV();
			__WFE();


    }
}

/** @} */
