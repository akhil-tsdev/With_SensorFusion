/*
 * Copyright © 2015 Turingsense, Inc.
 *
 * hub_main.c
 *
 * Main Nordic code running on the hub.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "app_util.h"
#include "nrf.h"
#include "boards.h"
#include "uesb/micro_esb.h"
#include "uesb/uesb_error_codes.h"
#include "nrf_gpio.h"
#include "simple_uart.h"

#include "hub_main_dtm.h"
#include "crc16.h"
#include "crc32.h"
#include "net_common.h"
#include "nrf_delay.h"

#include "spi_slave_comm_dtm.h"
//#include "hub_timer.h"			/* RTC */

/* DTM */
#include "ts_radio_dtm_function.h"


uint8_t g_ts_radio_dtm_debug_level = 0;
uint8_t g_ts_radio_dtm_cw = 0;
uint8_t g_ts_radio_dtm_tx_package_type = 3;
uint16_t g_ts_radio_dtm_tx_package_length = 100;
uint8_t g_ts_radio_dtm_rx_package_type = 0;
uint16_t g_ts_radio_dtm_rx_package_length = 0;
uint8_t g_ts_radio_dtm_tx_channel_id = 11;
uint8_t g_ts_radio_dtm_rx_channel_id = 0;
bool g_ts_radio_dtm_modulated = true;
uesb_tx_power_t g_ts_radio_dtm_power_level = UESB_TX_POWER_4DBM;
uint8_t g_ts_radio_dtm_tx_packet[256];
uint8_t g_ts_radio_dtm_rx_packet[256];

uint8_t g_ts_radio_dtm_cur_num = 0;
static uint32_t sRandomSeed;

// DTM
uint16_t last_received_crc16 = 0xFFFF;
uint16_t dtm_cmd_type;
uint16_t dtm_freq;
uint16_t dtm_pkt_len;
uint16_t dtm_power;
uint16_t dtm_pkt_type;
uint16_t dtm_tx_interval_ms = 5;


bool dtm_new_cmd = false;
/************************** Globals *******************************/
// Metrics
uint32_t cur_num_of_sat = 2;
uint32_t cur_sat_ids[MAX_SAT_SENSORS] = {124,120};
uint32_t new_lcp;

//USED FOR COMMAND MANAGEMENT
uint32_t command_to_sat[MAX_SAT_COMMSTRUCT]; //[0] SAT ID to send command - 0 mens ALL SAT; [1] command; [2] value 1; [3] value 2; [4] value 3; 
bool command_should_send_to_sat = false;
//static uint8_t command_sendings_left = 0;

//static uint32_t ct_rows_received[MAX_SAT_SENSORS] = {0};
uint32_t total_rows_received = 0;
//static uint32_t rs_callback_ct = 0;
//static uint32_t valid_rs_ct = 0;
//static uint32_t rows_seen = 0;
//static uint32_t last_timestamp_seen_at[MAX_SAT_SENSORS] = {0};
//static uint8_t last_row_count_seen_at[MAX_SAT_SENSORS] = {0};
//static uint32_t ct_out_of_turn = 0;
//static uint32_t rowct_from_satellite[MAX_SAT_SENSORS];
/// Index is how many checkbytes were corrupted. So if pattern2 was corrupted 
/// but not pattern1, index 1 will be incremented.
uint32_t corruption_correlation[3] = {0};
/// Index is how many bits of the checkbytes were corrupted. So if a packet
/// has two corrupted checkbits in the checkbytes, index 2 will be incremented.
uint32_t corruption_histogram[17] = {0};

#define MS_PER_SAMPLE 10
int32_t satellite_timestamp_adj[MAX_SAT_SENSORS] = {0};
uint16_t ct_gap_rows[MAX_SAT_SENSORS] = {0};


//static sth_payload_t last_payload[MAX_SAT_SENSORS];

// Current satellite idx to talk to.
// This is the index to cur_sat_ids.
//static uint8_t target_sat_idx = 0;

//static uint16_t last_received_crc16[MAX_SAT_SENSORS] = {0};

// Variables used for RTC calibration.
//static uint8_t ct_rtc_sendings_left = 0;
//static uint8_t ct_lcp_sendings_left = 0;
//static uint16_t last_rtc_id = 0;				/* The last RTC ID we sent (i.e., now waiting for the ack) */
//static bool waiting_rtc_complete = false;
//static uint16_t ack_from_sat = 0;

//static uint8_t ct_repeats_of_last_satellite = 0;
//static uint8_t rows_in_last_packet = 0;

#if UART_RENDER
/// Sends |data| across UART in a format that eMPL-client.py instances can read.
void debug_putstring(uint8_t* data) {
	// How this works: packets for arbitrary string logging have an 18-byte buffer.
	//
	// Bytes to be sent are copied into |current_buffer|. First from |scraps|,
	// then from |data|.
	// Whenever |current_buffer| fills up, send those 18 bytes and empty it again.
	// When you run out of bytes to process, anything still in |current_buffer|
	// becomes the new |scraps|, to be sent (we hope) next time.
	
	// Leftover data from previous calls to this function.
	static uint8_t scraps[17] = {0};
	static uint8_t len_scraps = 0;
	uint8_t scraps_pos = 0;
	uint8_t data_pos = 0;
	uint8_t current_buffer[18];
	int buffer_pos=0;
	// Get all leftovers in
	while(scraps_pos < len_scraps) {
		current_buffer[buffer_pos] = scraps[scraps_pos];
		scraps_pos++; buffer_pos++;
	}
	len_scraps = 0;
	while(data[data_pos]) {
		current_buffer[buffer_pos] = data[data_pos];
		data_pos++; buffer_pos++;
		if(buffer_pos == 18) {
			simple_uart_put('$');
			simple_uart_put(0x01);
			simple_uart_put(0x00);
			for(buffer_pos=0; buffer_pos < 18; ++buffer_pos)
			{
				simple_uart_put(current_buffer[buffer_pos]);
			}
			simple_uart_put('\r');
			simple_uart_put('\n');
			buffer_pos = 0;
		}
	}
	for(len_scraps = 0; len_scraps < buffer_pos; ++len_scraps) {
		scraps[len_scraps] = current_buffer[len_scraps];
	}
	
}
#else
// No weird rendering protocol. Just echo out the bytes directly.
#define debug_putstring simple_uart_putstring
#endif

void process_packet(sth_packet_t packet);
void nack_warning(void) {
}

#if EVAL_BOARD
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

/// The first time this is called during Button 4 being pressed down, it'll return true.
bool control_button_pressed(uint8_t num) {
	uint8_t button_index = num - 1; //for example button 4 would yield index 3
	uint32_t buttons[] = BUTTONS_LIST;
	static bool control_button_was_down[4] = {false, false, false, false};
	bool control_button_is_down = !nrf_gpio_pin_read(buttons[button_index]);
	bool press = (!control_button_was_down[button_index] && control_button_is_down);
	control_button_was_down[button_index] = control_button_is_down;
	return press;	
}
#endif /* EVAL_BOARD */

// Most recent control packet from the hub.
volatile bool packet_waiting = false;
volatile sth_packet_t packet_to_process;

// Our incoming packet handler.
void process_packet(sth_packet_t sth_packet) {
	// No significant work done here; just record the packet and signal
	// to the main loop that there's something to read. This method is
	// called from an interrupt handler, so having it do significant
	// work is a drain on the firmware's sanity.
	if(!packet_waiting) {
		packet_to_process = sth_packet;
		packet_waiting = true;
	}
}

	/* DTM Power
	* 0: +4dBm   DEFAULT
	* 1: 0dBm
	* 2: -4dBm
	* 3: -8dBm
	* 4: -12dBm
	* 5: -16dBm
	* 6: -20dBm
	* 7: -30dBm
	*/
uesb_tx_power_t convert_dtm_power(uint16_t dtm_power) {
	uesb_tx_power_t tx_power = (uesb_tx_power_t)dtm_power;
	switch (dtm_power) {
		case 0: 
			tx_power = UESB_TX_POWER_4DBM;
			break;
		case 1: 
			tx_power = UESB_TX_POWER_0DBM;
			break;
		case 2: 
			tx_power = UESB_TX_POWER_NEG4DBM;
			break;
		case 3: 
			tx_power = UESB_TX_POWER_NEG8DBM;
			break;
		case 4: 
			tx_power = UESB_TX_POWER_NEG12DBM;
			break;
		case 5: 
			tx_power = UESB_TX_POWER_NEG16DBM;
			break;
		case 6: 
			tx_power = UESB_TX_POWER_NEG20DBM;
			break;
		case 7: 
			tx_power = UESB_TX_POWER_NEG30DBM;
			break;
	}
	
	return tx_power;
}
void initRandom()
{
	sRandomSeed = 0x02;
}


uint32_t getRandom(uint8_t is_prbs9)
{
	uint8_t a = (is_prbs9 ? 6 : 14);
	uint32_t new_v = (((sRandomSeed >> a) ^ (sRandomSeed >> (a-1))) & 0x01);
	sRandomSeed = ((sRandomSeed << 1) | new_v) & 0x7F;
	return sRandomSeed;
}

int main(void)
{
		bool									prbs_val = false;
		uint8_t								data_repeat;
		uint8_t 							uesb_tx_data[UESB_CORE_MAX_PAYLOAD_LENGTH];	/* Max UESB packet size */

#if PRINT_DEBUG
		char report[110] = {0};
#endif 

    // Initialize the 16MHz clock used by SPI and ESB.
		// Supposedly uses a crystal instead of the internal RCOSC
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
       // Wait
    }
    
		spi_slave_example_init();
#if EVAL_BOARD
		ui_init();
#endif /* EVAL_BOARD */
		
		ts_init_ts_radio_dtm_command();
		initRandom();
		
    while (true)
    {   
			if (dtm_new_cmd == true) {
				if (dtm_pkt_type <= DTM_PKT_TYPE_MAX) {
					if (dtm_pkt_type == DTM_PKT_UNMODULATED) {
						g_ts_radio_dtm_modulated = false;
					} else {
						g_ts_radio_dtm_modulated = true;
					}
					g_ts_radio_dtm_tx_package_type = dtm_pkt_type;
				}
				if (dtm_pkt_len <= UESB_CORE_MAX_PAYLOAD_LENGTH) {
					g_ts_radio_dtm_tx_package_length = dtm_pkt_len;
				}
				if (dtm_freq <= DTM_FREQ_MAX) {
					g_ts_radio_dtm_tx_channel_id = dtm_freq;
					g_ts_radio_dtm_rx_channel_id = g_ts_radio_dtm_tx_channel_id;
				}
				g_ts_radio_dtm_power_level = convert_dtm_power(dtm_power);
				
				net_init(process_packet, 0, g_ts_radio_dtm_tx_channel_id, g_ts_radio_dtm_rx_channel_id, \
					g_ts_radio_dtm_power_level, g_ts_radio_dtm_modulated);
				
				dtm_new_cmd = false;
			}

			/* Fill in packet and send it.
			 * Only the first bytes reflect the packet type.  If the packet length is bigger than that,
			 * then we don't guarantee the content of the rest of the packet.
			 */
			prbs_val = false;
			if (g_ts_radio_dtm_tx_package_type == DTM_PKT_TYPE_PRBS9) {
				for (uint8_t qq = 0; qq < g_ts_radio_dtm_tx_package_length; qq++) {
					uesb_tx_data[qq] = getRandom(1);
				}
				prbs_val = true;
			} else if (g_ts_radio_dtm_tx_package_type == DTM_PKT_TYPE_PRBS15) {
				for (uint8_t qq = 0; qq < g_ts_radio_dtm_tx_package_length; qq++) {
					uesb_tx_data[qq] = getRandom(0);
				}
				prbs_val = true;
			} else if (g_ts_radio_dtm_tx_package_type == DTM_PKT_TYPE_11110000) {
				data_repeat = 0xF0;
			}else if (g_ts_radio_dtm_tx_package_type == DTM_PKT_TYPE_10101010) {
				data_repeat = 0xAA;
			} else { /* DTM_PKT_TYPE_01010101 */
				data_repeat = 0x55;
			}
			
			if (prbs_val == false) {
				for (uint8_t qq = 0; qq < g_ts_radio_dtm_tx_package_length; qq++) {
					uesb_tx_data[qq] = data_repeat;
				}
			}
	
			send_packet_dtm(&uesb_tx_data[0], g_ts_radio_dtm_tx_package_length);
		
			/* We can't put 0 for this function, so delay for 1 us if user enters 0ms */
			if (dtm_tx_interval_ms == 0) {
				nrf_delay_us(1);
			} else {
				nrf_delay_us(GET_DTM_TX_INTERVAL_US(dtm_tx_interval_ms));
			}
    }
}





