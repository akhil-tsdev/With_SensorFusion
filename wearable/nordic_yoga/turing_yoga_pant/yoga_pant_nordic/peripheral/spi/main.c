/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#include "nrf.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_uart.h"
#include "nrf_serial.h"
#include "app_timer.h"


//Debug Macros definitions
#define TIMER_DEBUG 1
#define PRINT_DEBUG 0





#define APP_TIMER_PRESCALER                  0   /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS                 3   /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE              4   /**< Size of timer operation queues. */




// ### Workaround Section Start ###
#include "nrf_drv_ppi.h"
#include "nrf_drv_gpiote.h"
// ### Workaround Section End   ###

//KF
// New SF
#define STDG 0.5*PI/180
#define STDA 0.02/9.81

#define CA 0.1
#define CB 0.01
#define PO 0.001
//

//APP TIMER PRESCALAR value for RTC1
#define APP_TIMER_PRESCALAR 0

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif

#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "mpu9250_firmware.h"
#include "lsm6dsl_firmware.h"
#include "common_types.h"

#include "nrf_esb.h"
#include "nrf_esb_error_codes.h"

//For KF
#include "tasks.h"


#define SPI_INSTANCE  0 /**< SPI instance index. */
const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define WHO_AM_I_VALID_VAL          0x71

#define LSM6DSL_WHOAMI_VAL					0x6A

static uint8_t       m_tx_buf[] = { 0x8F };           /**< TX buffer. */   // chitrang -- WHO_AM_I register | 0x80 (for reading)   0xF5 -- MPU and 0x8F for LSM
static uint8_t       m_rx_buf[2];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

uint8_t sensor_num;
  //static bool get_raw_data_sample(uint8_t sensor_num, sensor_data_t *raw_data, sat_sensor_record_t *sensor_record);
  //static bool get_raw_data_sample_basic(uint8_t sn, sensor_data_t *raw_data);
 static int counterDataSample[MAX_SENSORS_IN_CS] = {0};

// ### Workaround Section Start ###
//static nrf_ppi_channel_t ppi_channel;
// ### Workaround Section End   ###






// -----ESB example --------

#define NRF52840_ESB_DEMO        1
#define TX_ESB                   0
#define RX_ESB                   0


static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);

static nrf_esb_payload_t        rx_payload;


// -----ESB example --------

#define UART_TX_BUF_SIZE 64                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 64                         /**< UART RX buffer size. */

#define SATID 321

#define NUM_OF_SENSORS     7

#define MPU_ENABLE    0
#define LSM_ENABLE    1


#define OP_QUEUES_SIZE            3
#define DEBUG_UART_RX_PIN_NUMBER  8
#define DEBUG_UART_TX_PIN_NUMBER  6


static void sleep_handler(void)
{
    //__WFE();
    //__SEV();
    //__WFE();
}

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart1_drv_config,
                      DEBUG_UART_RX_PIN_NUMBER, DEBUG_UART_TX_PIN_NUMBER,
                      RTS_PIN_NUMBER, CTS_PIN_NUMBER,
                      NRF_UART_HWFC_ENABLED, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_115200,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);

#define SERIAL_FIFO_TX_SIZE 32
#define SERIAL_FIFO_RX_SIZE 32

NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);


#define SERIAL_BUFF_TX_SIZE 1
#define SERIAL_BUFF_RX_SIZE 1

NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_IRQ,
                      &serial_queues, &serial_buffs, NULL, sleep_handler);


NRF_SERIAL_UART_DEF(serial_uart, 1);

sensor_data_t   mpu_raw_sensor_data_t;


int16_t gyrBiasInt16_X[MAX_SENSORS_IN_CS] = {0};
int16_t gyrBiasInt16_Y[MAX_SENSORS_IN_CS] = {0};
int16_t gyrBiasInt16_Z[MAX_SENSORS_IN_CS] = {0};
int16_t accSensInt16_X[MAX_SENSORS_IN_CS]; //Initialized in main()
int16_t accSensInt16_Y[MAX_SENSORS_IN_CS]; //Initialized in main()
int16_t accSensInt16_Z[MAX_SENSORS_IN_CS]; //Initialized in main()
int16_t accBiasInt16_X[MAX_SENSORS_IN_CS] = {0};
int16_t accBiasInt16_Y[MAX_SENSORS_IN_CS] = {0};
int16_t accBiasInt16_Z[MAX_SENSORS_IN_CS] = {0};

uint32_t myHubID9250 = 0;
uint32_t myHubIDrcvd = 0;
uint32_t myHubID = 0;

uint16_t myLCP = 0;
uint32_t sat_id = SATID;


void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

#if 0
static void spi_send_recv(uint8_t * const p_tx_data,
                          uint8_t * const p_rx_data,
                          const uint16_t  len);
#endif

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
//    NRF_LOG_INFO("Transfer completed.");
//    if (m_rx_buf[0] != 0)
//    {
//				if(m_rx_buf[0] == LSM6DSL_WHOAMI_VAL)
//					bsp_board_led_off(BSP_BOARD_LED_1);
//        NRF_LOG_INFO(" Received:");
//        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
//    }
	
	
//	    uint32_t err_code;
//	
//			switch(p_event->type)
//				{
//						case NRF_DRV_SPI_EVENT_DONE:
//						
//								// ### Workaround Section Start ###
//						
//								// Disable the PPI channel
//								//err_code = nrf_drv_ppi_channel_disable(ppi_channel);
//								//APP_ERROR_CHECK(err_code);

//								// Disable a GPIOTE output pin task.
//								//nrf_drv_gpiote_in_event_disable(SPI_SCK_PIN);
//						
//								// ### Workaround Section End   ###
//								spi_xfer_done = true;
//								NRF_LOG_INFO("Transfer completed.\r\n");
//								if (m_rx_buf[0] != 0)
//								{
//									if(m_rx_buf[0] == LSM6DSL_WHOAMI_VAL)
//										bsp_board_led_off(BSP_BOARD_LED_1);
//									NRF_LOG_INFO(" Received: \r\n");
//									NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
//								}
//							default:
//								break;
//				}
}

////Dummy handler
//void in_pin_handler( nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
//{
//    //Do nothing
//}


//static void spi_send_recv(uint8_t * const p_tx_data,
//                          uint8_t * const p_rx_data,
//                          const uint16_t  len)
//{
//    //SEGGER_RTT_WriteString(0, "Send and Receive info \n");
//    uint32_t err_code;
//
//    if (len == 1){
//            ret_code_t err_code;
//
//            // Enable the PPI channel.
//            err_code = nrf_drv_ppi_channel_enable(ppi_channel);
//            APP_ERROR_CHECK(err_code);
//
//            // Enable the GPIOTE output pin task.
//            nrf_drv_gpiote_in_event_enable(SPI_SCK_PIN, false);
//
//            //Start transfer of data
//            err_code = nrf_drv_spi_transfer(&spi, p_tx_data, len, p_rx_data, len + 1);
//            APP_ERROR_CHECK(err_code);
//            }
//            else{
//                // Start transfer.
//                err_code = nrf_drv_spi_transfer(&spi, p_tx_data, len, p_rx_data, len);
//                APP_ERROR_CHECK(err_code);
//            }
//}




void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            NRF_LOG_DEBUG("TX SUCCESS EVENT");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            NRF_LOG_DEBUG("TX FAILED EVENT");
            (void) nrf_esb_flush_tx();
            (void) nrf_esb_start_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            NRF_LOG_DEBUG("RX RECEIVED EVENT");
            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                if (rx_payload.length > 0)
                {
                    printf("Receiving packet: %d\r\n", rx_payload.data[0]);
                }
            }
            break;
    }
    //NRF_GPIO->OUTCLR = 0xFUL << 12;
    //NRF_GPIO->OUTSET = (p_event->tx_attempts & 0x0F) << 12;
}

uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay         = 600;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
		#if TX_ESB
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
		#endif 
		#if RX_ESB
		nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;
		#endif
    nrf_esb_config.selective_auto_ack       = false;
		nrf_esb_config.payload_length           = 32;

    err_code = nrf_esb_init(&nrf_esb_config);

    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    return err_code;
}

void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

// Adding to allow KF to run


/* PIVOT_3_0 WARNING: You must first call sat_enable_sensor for the correct sensor
 * before you call this function,
 * to make sure that you enable the right sensor!
 */
static void simpleReset(int16_t setOutputMode, int16_t accSensX, int16_t accSensY, int16_t accSensZ,
		int16_t accBiasX, int16_t accBiasY, int16_t accBiasZ, int16_t gyrBiasX, int16_t gyrBiasY,
		int16_t gyrBiasZ) {

	//counterDataSample[mpu9250_num] = 0;

	//sensorFusionInit();

	//TODO - Accelerometer calibration
	accSensInt16_X[mpu9250_num] = accSensX;
	accSensInt16_Y[mpu9250_num] = accSensY;
	accSensInt16_Z[mpu9250_num] = accSensZ;
	accBiasInt16_X[mpu9250_num] = accBiasX;
	accBiasInt16_Y[mpu9250_num] = accBiasY;
	accBiasInt16_Z[mpu9250_num] = accBiasZ;

	// Gyroscope Bias
	gyrBiasInt16_X[mpu9250_num] = gyrBiasX;
	gyrBiasInt16_Y[mpu9250_num] = gyrBiasY;
	gyrBiasInt16_Z[mpu9250_num] = gyrBiasZ;
}

void set_mpu9250_sensor_num(uint8_t num) {
 mpu9250_num = num;
}


int main(void)
{
		ret_code_t ret;
    bsp_board_leds_init();
	
	  ret = nrf_serial_init(&serial_uart, &m_uart1_drv_config, &serial_config);
    APP_ERROR_CHECK(ret);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    
    NRF_LOG_INFO("SPI example.");

		#if PRINT_DEBUG
		//printf("Here in the main loop\n");
		#endif
	
		#if TIMER_DEBUG
		
		#endif
	
	
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
		spi_config.frequency = NRF_DRV_SPI_FREQ_4M;
		spi_config.mode  = NRF_DRV_SPI_MODE_0;
	

    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
		NRF_GPIO_Type * reg = NRF_P0;

    reg->PIN_CNF[SPI_SCK_PIN] |= ((uint32_t)GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos);
    reg->PIN_CNF[SPI_MOSI_PIN] |= ((uint32_t)GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos);
    
		//APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL, NULL));
		
		
		//nrf_delay_ms(100);
		uint32_t err_code;
	
		const app_uart_comm_params_t comm_params =
				{
						RX_PIN_NUMBER,
						TX_PIN_NUMBER,
						RTS_PIN_NUMBER,
						CTS_PIN_NUMBER,
						UART_HWFC,
						false,
						NRF_UART_BAUDRATE_230400
				};

		APP_UART_FIFO_INIT(&comm_params,
													 UART_RX_BUF_SIZE,
													 UART_TX_BUF_SIZE,
													 uart_error_handle,
													 APP_IRQ_PRIORITY_LOWEST,
													 err_code);

		APP_ERROR_CHECK(err_code);
		//bsp_board_led_on(BSP_BOARD_LED_2);
		reg = NRF_P1;
		uint8_t UART_TX_PIN_NUMER = 2;
		reg->PIN_CNF[UART_TX_PIN_NUMER] |= ((uint32_t)GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos);
				
		err_code = esb_init();
		APP_ERROR_CHECK(err_code);
		clocks_start();


/*************************************************
		// ### Workaround Section Start ###   THis is software workaround is to prevent SPIM from sending additional tx byte when tx_length and rx_length = 1
				
		 //uint32_t err_code;
		
		 NRF_SPIM_Type * p_spim = spi.p_registers;
		
		//Initialize GPIOTE and PPI drivers for the workaround
		
		err_code = nrf_drv_gpiote_init();
		APP_ERROR_CHECK(err_code);
		
		// Initializes the GPIOTE channel so that SCK toggles generates events
		nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
						err_code = nrf_drv_gpiote_in_init(SPI_SCK_PIN, &config, in_pin_handler);
						APP_ERROR_CHECK(err_code); 
				
		err_code = nrf_drv_ppi_init();
		APP_ERROR_CHECK(err_code);
		
		// Allocate first unused PPI channel 
		err_code = nrf_drv_ppi_channel_alloc(&ppi_channel);
		APP_ERROR_CHECK(err_code);
		
		// Assign the PPI channel to the SCK pin
		err_code = nrf_drv_ppi_channel_assign(ppi_channel,
																				nrf_drv_gpiote_in_event_addr_get(SPI_SCK_PIN),
																				(uint32_t)&p_spim->TASKS_STOP);
		APP_ERROR_CHECK(err_code);


		// ### Workaround Section End   ###    				
***********************************************/		
		
		
		
		#if  NRF52840_ESB_DEMO

			#if TX_ESB
			while (true)
			{
					//printf("Transmitting packet %02x\r\n", tx_payload.data[1]);

					tx_payload.noack = false;
					if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
					{
							// Toggle one of the LEDs.
							nrf_gpio_pin_write(LED_1, !(tx_payload.data[1]%8>0 && tx_payload.data[1]%8<=4));
							nrf_gpio_pin_write(LED_2, !(tx_payload.data[1]%8>1 && tx_payload.data[1]%8<=5));
							nrf_gpio_pin_write(LED_3, !(tx_payload.data[1]%8>2 && tx_payload.data[1]%8<=6));
							nrf_gpio_pin_write(LED_4, !(tx_payload.data[1]%8>3));
							tx_payload.data[1]++;
					}
					else
					{
							//printf("Sending packet failed\r\n");
					}

					nrf_delay_us(50000);
			}
			
			#endif
			
			
			#if RX_ESB
				  err_code = nrf_esb_start_rx();
					APP_ERROR_CHECK(err_code);
					while (true)
					{
							if (NRF_LOG_PROCESS() == false)
							{
									__WFE();
							}
					}
			#endif 
		
		#endif 

	// Init operations for KF
	// enable mag free SF by default;
	globals.newKF_enabled = true;

	// Init new SF
	Fusion_Init1(&accGyrKF,&accGyrKFpar,&UVRH,STDG,STDA,CA,CB,PO,XDIM,YDIM);

	for (sensor_num = 0; sensor_num < valid_num_sensors_in_cs; sensor_num++)
	{

		setKFStaticToZero(&accGyrKF_static[sensor_num]);
		setRefFrameStaticToZero(&UVRH_static[sensor_num]);

	   // reset the loop counter to zero for first iteration
	   globals.loopcounter[mpu9250_num] = 0;
				
		simpleReset(OUTPUT_MODE_QUMA,accSensInt16_X[sensor_num],accSensInt16_Y[sensor_num],accSensInt16_Z[sensor_num],accBiasInt16_X[sensor_num],
				accBiasInt16_Y[sensor_num],accBiasInt16_Z[sensor_num],gyrBiasInt16_X[sensor_num],gyrBiasInt16_Y[sensor_num],gyrBiasInt16_Z[sensor_num]);
	}


	uint8_t cs = 0x0;
	uint8_t byte_to_send = (cs << 5)& 0xE0;  // Setting the three most significant bits of the byte. As per the maxwell document. 
	while (app_uart_put(byte_to_send) != NRF_SUCCESS);
	//nrf_delay_ms(1);
				
				
				
				
	while (1)
	{
		// Reset rx buffer and transfer done flag
		memset(m_rx_buf, 0, m_length);
		spi_xfer_done = false;
		bsp_board_led_on(BSP_BOARD_LED_1);
	
		#if 0
			cs = 5;
			byte_to_send = (cs << 5)& 0xE0;
			while (app_uart_put(byte_to_send) != NRF_SUCCESS);
			nrf_delay_ms(1);
			APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
			//spi_send_recv(m_tx_buf, m_rx_buf, 1);
			nrf_delay_ms(1);
			cs = 0x0;
			byte_to_send = (cs << 5)& 0xE0;
			while (app_uart_put(byte_to_send) != NRF_SUCCESS);
		
		#endif 
			
		#if 1
			uint8_t my_who_am_i;
			//printf("--------------------\r\n");
			for(int i = 1; i < 8; i++)
			{
				#if MPU_ENABLE
					set_mpu9250_sensor_num(i);
					mpu9250_detect_sensor(i , &my_who_am_i);
				
					if(my_who_am_i == WHO_AM_I_VALID_VAL)
					{
						bsp_board_led_off(BSP_BOARD_LED_1);
						mpu9250_init_regs();
						mpu9250_start();
						mpu9250_read(SENSOR_ACCEL , &mpu_raw_sensor_data_t);
						mpu9250_read(SENSOR_GYRO , &mpu_raw_sensor_data_t);
						//printf("Sensor %d -- A:%d,%d,%d\r\n", i, mpu_raw_sensor_data_t.accel[0], mpu_raw_sensor_data_t.accel[1], mpu_raw_sensor_data_t.accel[2] );
						//printf("Sensor %d -- G:%d,%d,%d\r\n", i,  mpu_raw_sensor_data_t.gyro[0], mpu_raw_sensor_data_t.gyro[1], mpu_raw_sensor_data_t.gyro[2] );
						//mpu9250_read(SENSOR_MAG , &mpu_raw_sensor_data_t);
						//mpu9250_read(SENSOR_TEMP , &mpu_raw_sensor_data_t);
					}
					
		#endif 
				
		#if LSM_ENABLE
			//bsp_board_led_on(BSP_BOARD_LED_1);
			
			lsm6dsl_detect_sensor(i , &my_who_am_i);
			if(my_who_am_i == LSM6DSL_WHOAMI_VAL)
			{
				//turn on the LED
				bsp_board_led_on(BSP_BOARD_LED_1);
				lsm6dsl_init_regs();
				lsm6dsl_read(SENSOR_ACCEL | SENSOR_GYRO , &mpu_raw_sensor_data_t);
				//printf("Sensor %d -- A:%d,%d,%d  G:%d,%d,%d\r\n", i, mpu_raw_sensor_data_t.accel[0], mpu_raw_sensor_data_t.accel[1], mpu_raw_sensor_data_t.accel[2] ,
				 //                                                   mpu_raw_sensor_data_t.gyro[0], mpu_raw_sensor_data_t.gyro[1], mpu_raw_sensor_data_t.gyro[2] );
				
				
				
				// Sending the sensor data using the ESB.
				// Writing the data to the payload 
				tx_payload.length = 15;
				memcpy( tx_payload.data + 1, (uint8_t *)mpu_raw_sensor_data_t.accel, 3*sizeof(int16_t));
				memcpy( (tx_payload.data + 1 +(3 * sizeof(int16_t))),(uint8_t *)mpu_raw_sensor_data_t.gyro, 3*sizeof(int16_t));
				//printf("Transmitting packet %d\r\n", tx_payload.data[0]);
				tx_payload.noack = false;
				if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
				{
					
					// Toggle one of the LEDs.
					//nrf_gpio_pin_write(LED_1, !(tx_payload.data[1]%8>0 && tx_payload.data[1]%8<=4));
					//nrf_gpio_pin_write(LED_2, !(tx_payload.data[1]%8>1 && tx_payload.data[1]%8<=5));
					//nrf_gpio_pin_write(LED_3, !(tx_payload.data[1]%8>2 && tx_payload.data[1]%8<=6));
					//nrf_gpio_pin_write(LED_4, !(tx_payload.data[1]%8>3));
					tx_payload.data[0]++;
					if(tx_payload.data[0] == 255)
						tx_payload.data[0] = 0;
				}
				else
				{
					printf("Sending packet failed\r\n");
				}

				//nrf_delay_us(50000);
				
				
				
				//uint8_t mag_who_am_i = detect_stm_mag_sensor();
				//printf("mag_who_am_i: %x\r\n", mag_who_am_i);
			}
			//nrf_delay_ms(250);
		
		#endif
					// LOAD FROM STATIC TO DYNAMIC
					if (UVRH.allocated)
					{
						//loadKFandRefFrame(&accGyrKF_static[sensor_num],&UVRH_static[sensor_num]);
					}
				
					// KF - TODO ACC AND GYR CALIB
					
					
					fRunAccGyrSF(&accGyrKF,&accGyrKFpar,mpu_raw_sensor_data_t.accel,mpu_raw_sensor_data_t.gyro,&UVRH,qbn);
						
					printf("Sensor %d -- Q:%f,%f,%f,%f  \r\n", i,qbn[0],qbn[1],qbn[2],qbn[3]);
					
					
					// SAVE FROM DYNAMIC TO STATIC
					if (UVRH.allocated)
					{
						//saveKFandRefFrame(&accGyrKF_static[sensor_num],&UVRH_static[sensor_num]);
					}
							
				}
				
				printf("******************\r\n");
			#endif 
				
			#if 0
				uint8_t who_am_i[NUM_OF_SENSORS] = 0;
				
				for(int i = 7 ; i > 0; i--)
				{		
					#if MPU_ENABLE
					set_mpu9250_sensor_num(i);
					mpu9250_detect_sensor(i , &who_am_i[i-1]);
					if(who_am_i[i] != WHO_AM_I_VALID_VAL)
						bsp_board_led_off(BSP_BOARD_LED_2);
					#endif 
					
					#if LSM_ENABLE
					lsm6dsl_detect_sensor(i , &who_am_i[i-1]);
					if(who_am_i[i] != LSM6DSL_WHOAMI_VAL)
						bsp_board_led_off(BSP_BOARD_LED_2);
					
					#endif 
				}
				
			#endif 
        while (!spi_xfer_done)
        {
            __WFE();
        }
				
        NRF_LOG_FLUSH();
        bsp_board_led_invert(BSP_BOARD_LED_3);
				
        nrf_delay_ms(300);
    }
}


struct __FILE
{
  int dummyVaroable;        //    Just for redefining __FILE, we won't we using it anyways ;)
};

FILE __stdout;
FILE __stdin;

int fputc(int c, FILE * stream)
{
	(void)nrf_serial_write(&serial_uart, (uint8_t *)&c, sizeof(uint8_t), NULL, NRF_SERIAL_MAX_TIMEOUT);
	return c; //return the character written to denote a successfull write
}

int fgetc(FILE * stream)
{
	uint8_t c;
	nrf_serial_read(&serial_uart, (uint8_t *)&c, sizeof(c), NULL, NRF_SERIAL_MAX_TIMEOUT);
	return c;
}
