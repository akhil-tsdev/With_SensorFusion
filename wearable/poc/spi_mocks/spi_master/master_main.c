#include "ts_spi_master.h"

#include "bsp.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "simple_uart.h"

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

#define APP_TIMER_PRESCALER      0                      /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS     BSP_APP_TIMERS_NUMBER  /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE  2                      /**< Size of timer operation queues. */

/**@file
 * @defgroup spi_master_example_with_slave_main main.c
 * @{
 * @ingroup spi_master_example
 *
 * @brief SPI master example application to be used with the SPI slave example application.
 */


/**@brief Function for initializing bsp module.
 */
void bsp_configuration()
{
    uint32_t err_code = NRF_SUCCESS;

    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
        
    err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}


static void ui_init(void)
{
	uint32_t buttons[] = BUTTONS_LIST;
	uint32_t leds[] = LEDS_LIST;
	uint32_t i;

	for (i = 0; i < BUTTONS_NUMBER; i++)
	{
			nrf_gpio_cfg_input(buttons[i],NRF_GPIO_PIN_PULLUP);
	}
	for (i = 0; i < 3; i++)
	{
			nrf_gpio_cfg_output(leds[i]);
	}
}


/**@brief Function for application main entry. Does not return. */
int main(void)
{
	// Setup bsp module.
	bsp_configuration();

	ui_init();
	
	start_spi();
	simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, false);
/*
	uint32_t theta = 0;
	while (true) {
		theta++;
		output_pyserial_record(sin(theta/10.0),sin(theta/10.0),cos(theta/10.0),0);
		nrf_delay_ms(100);
	}
	*/
	bool transfer_in_progress = false;
	for (;;)
	{
		if(transfer_in_progress) {
			transfer_in_progress = !spi_master_check_tick();
		}
		if(!transfer_in_progress) {
			transfer_in_progress = spi_master_pull();
			nrf_delay_us(50);
		}
	}
}


/** @} */
