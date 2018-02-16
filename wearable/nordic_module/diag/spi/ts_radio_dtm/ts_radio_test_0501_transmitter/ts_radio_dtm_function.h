#include <stdbool.h>
#include <stdint.h>

#define GARY_MAX_COM_PORT_IO_ARRAY_SIZE 256
#define TSC_RADIO_DTM_COMMAND_MAX_NUMBER  20
#define TSC_BUFSIZE  256
#define RFPHY_TEST_0X0F_REF_PATTERN  0x0f  /**<  RF-PHY test packet patterns, for the repeated octet packets. */
#define RFPHY_TEST_0X55_REF_PATTERN  0x55  /**<  RF-PHY test packet patterns, for the repeated octet packets. */

#define PRBS9_CONTENT {0xff, 0xc1, 0xfb, 0xe8, 0x4c, 0x90, 0x72, 0x8b,  \
                       0xe7, 0xb3, 0x51, 0x89, 0x63, 0xab, 0x23, 0x23,  \
                       0x2,  0x84, 0x18, 0x72, 0xaa, 0x61, 0x2f, 0x3b,  \
                       0x51, 0xa8, 0xe5, 0x37, 0x49, 0xfb, 0xc9, 0xca,  \
                       0xc,  0x18, 0x53, 0x2c, 0xfd}   

static char g_ts_radio_cmd_str[TSC_RADIO_DTM_COMMAND_MAX_NUMBER][TSC_BUFSIZE];
static int g_ts_radio_cmd_num;

// some gobal variable to remember package_type, channel_num, package_length, power_level, debug_level, cw
static uint8_t g_ts_radio_dtm_debug_level;
static uint8_t g_ts_radio_dtm_cw;
static uint8_t g_ts_radio_dtm_tx_package_type;
static uint8_t g_ts_radio_dtm_tx_package_length;
static uint8_t g_ts_radio_dtm_rx_package_type;
static uint8_t g_ts_radio_dtm_rx_package_length;
static uint8_t g_ts_radio_dtm_tx_channel_id;
static uint8_t g_ts_radio_dtm_rx_channel_id;
static int32_t g_ts_radio_dtm_power_level;
static uint8_t g_ts_radio_dtm_tx_packet[256];
static uint8_t g_ts_radio_dtm_rx_packet[256];


void ts_init_ts_radio_dtm_command(void);
bool is_ts_radio_dtm_exit_cmd(int index);
void ts_radio_dtm_command_help(void);
int get_ts_radio_dtm_command_index(char *cmdStr);
void print_ts_radio_dtm_cmd(int cmdIndex);
void gary_simple_uart_put_string(uint8_t *str, uint8_t data_num);
bool is_digital(uint8_t v);
bool is_char(uint8_t v);
bool is_valid_input_for_ts_radio_dtm_cmd(uint8_t v);
uint8_t get_ts_radio_dtm_command(uint8_t *rx_data);
uint8_t ts_simple_get_with_timeout(uint32_t timeout_ms);

void ts_radio_dtm_init(void);
void ts_radio_dtm_set_power(void);
void ts_radio_dtm_set_tx_channel(void);
void ts_radio_dtm_set_rx_channel(void);
void radio_disable(void);
void ts_radio_dtm_receiver(void);
void ts_radio_dtm_transmitter(void);
