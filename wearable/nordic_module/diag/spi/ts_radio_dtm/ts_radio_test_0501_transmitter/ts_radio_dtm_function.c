#include "ts_radio_dtm_function.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "simple_uart.h"



bool is_digital(uint8_t v) 
{
		if (v >= 0x30 && v <= 0x39) {
				return true;
		}
		return false;
}

bool is_char(uint8_t v)
{
		if (v >= 0x41 && v <= 0x5a) {
				return true;
		}
		if (v >= 0x61 && v <= 0x7a) {
				return true;
		}
		return false;
}

bool is_valid_input_for_ts_radio_dtm_cmd(uint8_t v)
{
	  if (is_digital(v) || is_char(v)) {
			return true;
		}
		if (v == 0x5f || v == 0x2e || v == 0x20 || v == 0x2d) {
			return true;
		}
		if (v == 0x08) {
			return true;
		}
		return false;
}


uint8_t ts_simple_get_with_timeout(uint32_t timeout_ms)
{
	  while (NRF_UART0->EVENTS_RXDRDY != 1) {
        // Wait for RXD data to be received
			if (timeout_ms-- >= 1) {
         nrf_delay_us(1000);
			} else {
         return 0x00; // enter
      }
    }

    NRF_UART0->EVENTS_RXDRDY = 0;
    return (uint8_t)NRF_UART0->RXD;
}


void gary_simple_uart_put_string(uint8_t *str, uint8_t data_num)
{
		if (data_num > GARY_MAX_COM_PORT_IO_ARRAY_SIZE) {
				return;
		}
		// char myCharArr[GARY_MAX_COM_PORT_IO_ARRAY_SIZE];
		// sprintf(myCharArr, "rec data number = %d ", data_num);
		// simple_uart_putstring(myCharArr);
		for (uint8_t index = 0; index < data_num; index ++) {
				simple_uart_put(str[index]);
		}
		// simple_uart_putstring("\r\n");
		simple_uart_putstring("\r\n");
}


void ts_init_ts_radio_dtm_command(void)
{
	 sprintf(g_ts_radio_cmd_str[0], "%s", "setup");
	 sprintf(g_ts_radio_cmd_str[1], "%s", "receiver");
	 sprintf(g_ts_radio_cmd_str[2], "%s", "transmitter");
	 sprintf(g_ts_radio_cmd_str[3], "%s", "loop_test");
	 sprintf(g_ts_radio_cmd_str[4], "%s", "exit");
	
	 g_ts_radio_cmd_num = 5;
	 
	 // some default values 
	 g_ts_radio_dtm_debug_level = 0;
	 g_ts_radio_dtm_cw = 0;
	 g_ts_radio_dtm_tx_package_type = 0;
	 g_ts_radio_dtm_tx_package_length = 0;
	 g_ts_radio_dtm_rx_package_type = 0;
	 g_ts_radio_dtm_rx_package_length = 0;
	 g_ts_radio_dtm_tx_channel_id = 0;
	 g_ts_radio_dtm_rx_channel_id = 0;
	 g_ts_radio_dtm_power_level = 0;
}


bool is_ts_radio_dtm_exit_cmd(int index) 
{
	return index == (g_ts_radio_cmd_num - 1);
}

void ts_radio_dtm_command_help(void)
{
	char cmd[TSC_BUFSIZE];
	simple_uart_putstring("\r\n   Valid ts_radio_dtm command list: \r\n");
	for (int i = 0; i < g_ts_radio_cmd_num; i ++) {
		sprintf(cmd, "%s%s", "     ts_radio_dtm ", g_ts_radio_cmd_str[i]);
		switch (i) {
			case 0:
				sprintf(cmd, "%s debug_level <0~5> cw <1~5>", cmd);
				break;
			case 1:
			case 2:
			case 3:
				sprintf(cmd, "%s <package_type> <channel_num> <package_length> <power_level>", cmd);
				break;
			default:
				break;
		}
		sprintf(cmd, "%s\n", cmd);
		int cmd_length = strlen(cmd);
		gary_simple_uart_put_string(cmd, cmd_length);
	}
	
}

int get_ts_radio_dtm_command_index(char *cmdStr)
{
	// define some variable get value from command
	char userStr[TSC_BUFSIZE];
	sprintf(userStr, "%s", cmdStr);
	int curParamNumber = 0;
	int cmdIndex = -1;
	char *token= strtok(userStr, " ,");
	while (token) {
		if (curParamNumber == 0) {
			if (strcmp(token, "ts_radio_dtm") != 0) {
				break;
			}
		} else if (curParamNumber == 1) {
			for (int i = 0; i < g_ts_radio_cmd_num; i ++) {
				if (strcmp(token, g_ts_radio_cmd_str[i]) == 0) {
					cmdIndex = i;
					break;
				}
			}
			if (cmdIndex < 0) {
				break;
			}
			// set default value of each command parameters
			switch (cmdIndex) {
				case 0:
					// g_ts_radio_dtm_debug_level = 0;
					// g_ts_radio_dtm_cw = 0;
					break;
				case 1:
					g_ts_radio_dtm_rx_package_type = 0;
					g_ts_radio_dtm_rx_package_length = 0;
					g_ts_radio_dtm_rx_channel_id = 0;
					break;
				case 2:
					g_ts_radio_dtm_tx_package_type = 0;
					g_ts_radio_dtm_tx_package_length = 0;
					g_ts_radio_dtm_tx_channel_id = 0;
					g_ts_radio_dtm_power_level = 0;
					break;
				case 3:
					g_ts_radio_dtm_tx_package_type = 0;
					g_ts_radio_dtm_tx_package_length = 0;
					g_ts_radio_dtm_tx_channel_id = 0;
					g_ts_radio_dtm_power_level = 0;
					break;
				default:
					break;
			}
		} else if (curParamNumber == 2) {
			switch (cmdIndex) {
				case 0:
					if (strcmp(token, "debug_level") == 0) {
						token = strtok(0, " ,");
						if (!token) {
							break;
						}
						g_ts_radio_dtm_debug_level = atoi(token);
					} else if (strcmp(token, "cw") == 0) {
						token = strtok(0, ", ");
						if (!token) {
							break;
						}
						g_ts_radio_dtm_cw = atoi(token);
					}
					break;
				case 1:
				case 2:
				case 3:
					// package type at firest
					if (strcmp(token, "prbs9") == 0) {
						if (cmdIndex == 1) {
							g_ts_radio_dtm_rx_package_type = 0;
						} else if (cmdIndex == 2) {
							g_ts_radio_dtm_tx_package_type = 0;
						}
					} else if (strcmp(token, "prbs15") == 0) {
						if (cmdIndex == 1) {
							g_ts_radio_dtm_rx_package_type = 1;
						} else if (cmdIndex == 2) {
							g_ts_radio_dtm_tx_package_type = 1;
						}
					} else if (strcmp(token, "11110000") == 0) {
						if (cmdIndex == 1) {
							g_ts_radio_dtm_rx_package_type = 2;
						} else if (cmdIndex == 2) {
							g_ts_radio_dtm_tx_package_type = 2;
						}
					} else if (strcmp(token, "01010101") == 0) {
						if (cmdIndex == 1) {
							g_ts_radio_dtm_rx_package_type = 3;
						} else if (cmdIndex == 2) {
							g_ts_radio_dtm_tx_package_type = 3;
						}
					}
					token = strtok(0, ", ");
					if (!token) {
						break;
					}
					if (cmdIndex == 1) {
						g_ts_radio_dtm_rx_channel_id = atoi(token);
					} else if (cmdIndex == 2) {
						g_ts_radio_dtm_tx_channel_id = atoi(token);
					}
					token = strtok(0, ", ");
					if (!token) {
						break;
					}
					if (cmdIndex == 1) {
						g_ts_radio_dtm_rx_package_length = atoi(token);
					} else if (cmdIndex == 2) {
						g_ts_radio_dtm_tx_package_length = atoi(token);
					}
					token = strtok(0, ", ");
					if (!token) {
						break;
					}
					g_ts_radio_dtm_power_level = atoi(token);
					if (g_ts_radio_dtm_power_level >= 4) {
						g_ts_radio_dtm_power_level = RADIO_TXPOWER_TXPOWER_Pos4dBm;
					} else if (g_ts_radio_dtm_power_level >= 0) {
						g_ts_radio_dtm_power_level = RADIO_TXPOWER_TXPOWER_0dBm;
					} else if (g_ts_radio_dtm_power_level >= -4) {
						g_ts_radio_dtm_power_level = RADIO_TXPOWER_TXPOWER_Neg4dBm;
					} else if (g_ts_radio_dtm_power_level >= -8) {
						g_ts_radio_dtm_power_level = RADIO_TXPOWER_TXPOWER_Neg8dBm;
					} else if (g_ts_radio_dtm_power_level >= -16) {
						g_ts_radio_dtm_power_level = RADIO_TXPOWER_TXPOWER_Neg16dBm;
					} else if (g_ts_radio_dtm_power_level >= -20) {
						g_ts_radio_dtm_power_level = RADIO_TXPOWER_TXPOWER_Neg20dBm;
					} else if (g_ts_radio_dtm_power_level >= -30) {
						g_ts_radio_dtm_power_level = RADIO_TXPOWER_TXPOWER_Neg30dBm;
					}
					break;
				default:
					break;
			}
		}
		token = strtok(0, " ,");
		curParamNumber = curParamNumber + 1;
	}
	if (cmdIndex < 0) {
		ts_radio_dtm_command_help();
	}
	return cmdIndex;
}


void print_ts_radio_dtm_cmd(int cmdIndex)
{
	
	char userStr[TSC_BUFSIZE];
	uint8_t cur_package_type = 0;
	int32_t cur_power_level = 0;
	uint8_t cur_package_length = 0;
	uint8_t cur_channel_id = 0;
	
	if (cmdIndex == 1) {
		cur_package_type = g_ts_radio_dtm_rx_package_type;
		cur_package_length = g_ts_radio_dtm_rx_package_length;
		cur_channel_id = g_ts_radio_dtm_rx_channel_id;
	} else if (cmdIndex == 2) {
		cur_package_type = g_ts_radio_dtm_tx_package_type;
		cur_package_length = g_ts_radio_dtm_tx_package_length;
		cur_power_level = g_ts_radio_dtm_power_level;
		cur_channel_id = g_ts_radio_dtm_tx_channel_id;
	}
	
	
	switch (cmdIndex) {
		case 0:
				sprintf(userStr, "  <CMD> ts_radio_dtm %s debug_level %d cw %d \n", g_ts_radio_cmd_str[cmdIndex], g_ts_radio_dtm_debug_level,
					g_ts_radio_dtm_cw);
			break;
		case 1:
		case 2:
		case 3:
			sprintf(userStr, "  <CMD> ts_radio_dtm %s package_type ", g_ts_radio_cmd_str[cmdIndex]);
			switch (cur_package_type) {
				case 0:
					sprintf(userStr, "%s%s", userStr, "prbs9");
					break;
				case 1:
					sprintf(userStr, "%s%s", userStr, "prbs15");
					break;
				case 2:
					sprintf(userStr, "%s%s", userStr, "11110000");
					break;
				case 3:
				default:
					sprintf(userStr, "%s%s", userStr, "01010101");
					break;
			}
			sprintf(userStr, "%s channel_num %d package_length %d power_level %d\n", userStr, 
				cur_channel_id, cur_package_length, cur_power_level);
			break;
		case 4:
			sprintf(userStr, "ts_radio_dtm %s\n", g_ts_radio_cmd_str[cmdIndex]);
			break;
		default:
			sprintf(userStr, "invalid command\r\n");
			break;
	}
	gary_simple_uart_put_string(userStr, strlen(userStr));
}


uint8_t get_ts_radio_dtm_command(uint8_t *rx_data)
{
		rx_data[0] = '\0';
	  uint32_t timeout_ms = 10000; // wait for 10 sec if no input
		bool get_all_data = false;
		uint8_t rec_data_num = 0;
	
		uint8_t last_v = 0;
		while (!get_all_data) {
			last_v = ts_simple_get_with_timeout(timeout_ms);
			if (!is_valid_input_for_ts_radio_dtm_cmd(last_v)) {
					get_all_data = true;
					continue;
			}
			if (last_v == 0x08 && (rec_data_num > 0)) {
				rec_data_num --;
				
				simple_uart_putstring("\r\n");
				simple_uart_putstring(rx_data);
			} else {
				rx_data[rec_data_num] = last_v;
				rec_data_num = rec_data_num + 1;
				if (rec_data_num >= GARY_MAX_COM_PORT_IO_ARRAY_SIZE) {
						get_all_data = true;
				}
				simple_uart_put(last_v);
			}
		}
		if (last_v == 0x0d) {
			  simple_uart_putstring("\r\n");
		}
		return rec_data_num;
}



void ts_radio_dtm_init(void)
{
	// Handle BLE Radio tuning parameters from production for DTM if required.
    // Only needed for DTM without SoftDevice, as the SoftDevice normally handles this.
    // PCN-083.
    if ( ((NRF_FICR->OVERRIDEEN) & FICR_OVERRIDEEN_BLE_1MBIT_Msk) == FICR_OVERRIDEEN_BLE_1MBIT_Override)
    {
        NRF_RADIO->OVERRIDE0 = NRF_FICR->BLE_1MBIT[0];
        NRF_RADIO->OVERRIDE1 = NRF_FICR->BLE_1MBIT[1];
        NRF_RADIO->OVERRIDE2 = NRF_FICR->BLE_1MBIT[2];
        NRF_RADIO->OVERRIDE3 = NRF_FICR->BLE_1MBIT[3];
        NRF_RADIO->OVERRIDE4 = NRF_FICR->BLE_1MBIT[4]| (RADIO_OVERRIDE4_ENABLE_Pos << RADIO_OVERRIDE4_ENABLE_Enabled);
    }


    // Turn off radio before configuring it
    NRF_PPI->CHENCLR = PPI_CHENCLR_CH0_Msk | PPI_CHENCLR_CH1_Msk;

    NRF_RADIO->SHORTS          = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE   = 1;

    while (NRF_RADIO->EVENTS_DISABLED == 0)
    {
        // Do nothing
    }

    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_RXEN      = 0;
    NRF_RADIO->TASKS_TXEN      = 0;

    // Set the access address, address0/prefix0 used for both Rx and Tx address
    NRF_RADIO->PREFIX0    &= ~RADIO_PREFIX0_AP0_Msk;
    
    NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos;
    NRF_RADIO->TXADDRESS   = (0x00 << RADIO_TXADDRESS_TXADDRESS_Pos) & RADIO_TXADDRESS_TXADDRESS_Msk;


}


void ts_radio_dtm_set_power(void)
{
		NRF_RADIO->TXPOWER = g_ts_radio_dtm_power_level;
}


void ts_radio_dtm_set_tx_channel(void)
{
	 NRF_RADIO->FREQUENCY = g_ts_radio_dtm_tx_channel_id;
}


void ts_radio_dtm_set_rx_channel(void)
{
	 NRF_RADIO->FREQUENCY = g_ts_radio_dtm_rx_channel_id;
}

void radio_disable(void)
{
    NRF_RADIO->SHORTS          = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TEST            = 0;
    NRF_RADIO->TASKS_DISABLE   = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0)
    {
        // Do nothing.
    }
    NRF_RADIO->EVENTS_DISABLED = 0;
}


void ts_radio_dtm_receiver(void)
{
	// radio_disable();
	NRF_RADIO->SHORTS     = RADIO_SHORTS_READY_START_Msk;
	//ts_radio_dtm_set_power();
	//ts_radio_dtm_set_rx_channel();
	NRF_RADIO->TASKS_RXEN = 1;
}


void ts_radio_dtm_transmitter(void)
{
	// radio_disable();
	g_ts_radio_dtm_tx_packet[0] = g_ts_radio_dtm_tx_package_type & 0x0F;
	g_ts_radio_dtm_tx_packet[1] = g_ts_radio_dtm_tx_package_length & 0x0F;
	
	if (g_ts_radio_dtm_tx_package_type == 0) {
		memset(g_ts_radio_dtm_tx_packet+2, 0xF0, g_ts_radio_dtm_tx_package_length);
	} else if (g_ts_radio_dtm_tx_package_type == 2) {
		memset(g_ts_radio_dtm_tx_packet+2, RFPHY_TEST_0X0F_REF_PATTERN, g_ts_radio_dtm_tx_package_length);
	} else if (g_ts_radio_dtm_tx_package_type == 3) {
		memset(g_ts_radio_dtm_tx_packet+2, RFPHY_TEST_0X55_REF_PATTERN, g_ts_radio_dtm_tx_package_length);
	}
	
	NRF_RADIO->PACKETPTR = (uint32_t)&g_ts_radio_dtm_tx_packet;
	NRF_RADIO->SHORTS     = RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_READY_START_Msk | \
                            RADIO_SHORTS_DISABLED_TXEN_Msk;
	// ts_radio_dtm_set_power();
	// ts_radio_dtm_set_tx_channel();
	NRF_RADIO->TASKS_TXEN = 1;
}
