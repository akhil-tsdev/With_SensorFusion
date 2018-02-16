#include <stdbool.h>
#include <stdint.h>
#include <string.h>
//#include "app_scheduler.h"
#include "app_util.h"
#include "nrf.h"
#include "uesb/micro_esb.h"
#include "uesb/uesb_error_codes.h"

#include <math.h>

#include "boards.h"
#include "nrf_gpio.h"
#include "simple_uart.h"
#include "net_common.h"

#include "crc16.h"
#include "nrf_delay.h"

#include "led_trace.h"

#define DEVICE_ID 7

#define APP_SCHED_QUEUE_SIZE 10
#define APP_SCHED_EVENT_SIZE sizeof(uesb_payload_t)

void print_log_entries(unpacked_log_t entries) {
	for(int i=0;(i<48);++i)
	{
		what_t entry = entries.entries[i];
		if(entry==NO_MESSAGE) return;
		print_log_entry(entry, DEVICE_ID);
	}
	print_log_entry(SATURATED, DEVICE_ID);
}

uint32_t requests_seen = 0;
uint32_t my_requests_seen = 0;
uint32_t laggard_requests_seen = 0;
uint32_t nonsense_requests_seen = 0;
uint32_t rows_sent = 0;
uint32_t packets_generated = 0;
uint32_t ct_excessive_diff = 0;
uint32_t sum_excessive_diff = 0;


uint32_t rows_pushed = 0;
uint32_t rows_popped = 0;


uint16_t expected_crc16 = 0;
uint16_t previous_crc16 = 0;
uint8_t sequence_number = 0;
static sth_payload_t next_payload;

static bool overdrain_mode = false;
static bool spi_transfer_in_progress = false;

bool control_button_pressed(void);

static unpacked_log_t unpacked_log;

static uint32_t fake_data_currtime = 0;

float precomputed_sin[256];
float precomputed_cos[256];


sth_payload_t new_payload(void) {
	FLIP_TRACE(3);
	sth_payload_t payload = {0};
	unpacked_log = dump_log();
	//sat_spi_data_packet_t data;
	packets_generated++;
	
	payload.timestamp = fake_data_currtime;
	payload.ts_diff[0] = 10;
	payload.ts_diff[1] = 20;
	payload.ts_diff[2] = 30;
	payload.ts_diff[3] = 40;
	for(int i=0;i<5;++i) {
		payload.sensor_records[i].quat_w = 0;
		uint8_t theta = ((fake_data_currtime + (10*i))*256)/1000;
		payload.sensor_records[i].quat_x = precomputed_sin[theta];
		payload.sensor_records[i].quat_y = precomputed_cos[theta];
		payload.sensor_records[i].quat_z = precomputed_sin[theta];
	}
	fake_data_currtime += 50;

	//pack_log_messages(unpacked_log.entries, payload.log_messages);
	payload.sequence_number = ++sequence_number;
	expected_crc16 = crc16_compute((uint8_t*)(&payload),sizeof(payload),0);
	return payload;
}

uint8_t rows_replied = 0;
void process_request(hts_payload_t request) {
	rows_replied = 0;
	// Stands in for the real logic: move onto the next datum once the current has been seen.
	if (request.last_seen_crc16 == expected_crc16) {
		previous_crc16 = expected_crc16;
		log_message(S_ADVANCE);
		uint32_t rows_sent_before = rows_sent;
		next_payload = new_payload();
		rows_replied = rows_sent - rows_sent_before;
	} else if(request.last_seen_crc16 == previous_crc16) {
		laggard_requests_seen++;
		log_message(S_LAGGARD);
	} else {
		nonsense_requests_seen++;
		log_message(S_TROUBLE);
	}
}

static bool NEVER_TRUE = false;
extern uesb_mainstate_t m_uesb_mainstate;

void send_response(char c) {
	//simple_uart_putstring((uint8_t*)"\r\n>=======");
	//print_log_entries(net_log);
	//simple_uart_putstring((uint8_t*)"\r\n=======>\r\n");
	
	packet_t packet;
	packet.header.character =c;
	packet.header.sender = DEVICE_ID;
	packet.payload.satellite_to_hub_payload = next_payload;
	
	send_packet(packet);
	while(fate_of_last_tx == TX_FATE_UNKNOWN) {
		//__WFE() ??
		if(NEVER_TRUE) {
			break;
		}
	}

}

void UART0_IRQHandler(void)
{
	if (NRF_UART0->EVENTS_RXDRDY != 0)
	{
		uint8_t rxd_char;
		
		NRF_UART0->EVENTS_RXDRDY = 0;
		
		rxd_char = (uint8_t) NRF_UART0->RXD;
		
		send_response(rxd_char);
				//err_code = app_sched_event_put(&packet, sizeof(packet), uesb_send_pkt_evt_handler);
        //APP_ERROR_CHECK(err_code);
			
        //fifo_put_char(&s_uart_fifo, rxd_char);
        /*
        // Send data packet when newline character is detected or when packet size has been reached
        if (rxd_char == '\n' || rxd_char == '\r' || fifo_num_elem_get(&s_uart_fifo) >= UESB_CORE_MAX_PAYLOAD_LENGTH)
        {
            uint8_t  char_buf[UESB_CORE_MAX_PAYLOAD_LENGTH];
            
            char_buf_len = sizeof(char_buf);
            //fifo_get_pkt(&s_uart_fifo, char_buf, &char_buf_len);
            
            err_code = app_sched_event_put(char_buf, char_buf_len, uesb_send_pkt_evt_handler);
            APP_ERROR_CHECK(err_code);
        }*/
	}
}

static void uart_config(void)
{
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, false);
    // Enable interrupt on UART RX
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_SetPriority(UART0_IRQn, 3);
    NVIC_EnableIRQ(UART0_IRQn);
    NRF_UART0->INTENSET = UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos;
}

volatile bool packet_waiting = false;
volatile packet_t packet_to_process;

void satellite_rx_handler(packet_t packet) {
	if(!packet_waiting) {
		packet_to_process = packet;
		packet_waiting = true;
	}
}

void process_rx_packet(packet_t packet) {	
	requests_seen++;
	FLIP_TRACE(1);
	hts_payload_t payload = packet.payload.hub_to_satellite_payload;
	// Only interested in packets from the hub.
	// Only interested if WE were asked to speak.
	if (payload.recipient != DEVICE_ID || packet.header.sender != 0) {
		// When not using CRC, after receiving the first 
		// nrf_esb_disable();
		log_message(S_NOT_MINE);
		return;
	};
	
	my_requests_seen++;
	FLIP_TRACE(2);
	process_request(payload);
	send_response(packet_to_process.header.character);
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
	for (i = 0; i < LEDS_NUMBER; i++)
	{
			nrf_gpio_cfg_output(leds[i]);
	}
	simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
}

bool control_button_pressed(void) {
	uint32_t buttons[] = BUTTONS_LIST;
	static bool control_button_was_down = false;
	bool control_button_is_down = !nrf_gpio_pin_read(buttons[3]);
	bool press = (!control_button_was_down && control_button_is_down);
	control_button_was_down = control_button_is_down;
	return press;	
}

#define ATOM_US 20
#define CYCLES_PER_SPI_ROUND 1 /*((DELAY_MS*1000)/ATOM_US)*/

bool last_spi_had_row = false;
uint32_t backoff_atoms = 256;

int main(void)
{
    // Set 16 MHz crystal as our 16 MHz clock source (as opposed to internal RCOSC)
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Wait
    }
    
    // Initialize the scheduler module
    //APP_SCHED_INIT(APP_SCHED_EVENT_SIZE, APP_SCHED_QUEUE_SIZE);
    
    // Initialize fifo
    //fifo_init(&s_uart_fifo);
    // Configure micro ESB
    // Configure UART
    //uart_config();
    ui_init();
    //simple_uart_putstring((const uint8_t*)"\r\n Wireless UART example\n");
    
		
    int32_t counter=0;
		uint32_t timestamp_50us=0;
		
		for(int i=0;i<256;++i) {
			precomputed_sin[i] = sin(i*3.14/128);
			precomputed_cos[i] = cos(i*3.14/128);
		}
		
    // Start listening for incoming transmissions
    net_init(satellite_rx_handler, 0, STD_STM_CHANNEL,STD_MTS_CHANNEL);
    uesb_start_rx();
		
    while (true)
    {   
#ifdef HAS_CONSOLE
			if(control_button_pressed()) {
				print_log_entries(dump_log());
				char trace_msg[100];
				snprintf(trace_msg,99,"\r\n%d RF comms, %d mine, %d laggard, %d weird. %d rows, %f per req",
								requests_seen,my_requests_seen,laggard_requests_seen,nonsense_requests_seen,
								rows_sent, rows_sent*1.0/packets_generated
				);
				simple_uart_putstring((uint8_t*) trace_msg);
				snprintf(trace_msg,99,"\r\n avg excessive diff %d. buffer deficit %d",
							   sum_excessive_diff/ct_excessive_diff, rows_pushed - rows_popped);
				simple_uart_putstring((uint8_t*) trace_msg);
				
			}
#endif
        
			// Let the CPU sleep until the next interrupt comes along
			//__WFE();
			//__SEV();
			//__WFE();

			if(packet_waiting) {
				//simple_uart_put(char_to_print);
				process_rx_packet(packet_to_process);
				packet_waiting=false;
			}

			
    }

}
