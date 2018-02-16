#include "ts_spi_master.h"

#include <stdio.h>
#include <stdbool.h>
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "bsp.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "spi_master.h"
#include "nordic_common.h"
#include "simple_uart.h"
#include "sat_spi_types.h"

#define HAS_CONSOLE

/*
 * This example uses only one instance of the SPI master.
 * Please make sure that only one instance of the SPI master is enabled in config file.
 */


#define TX_RX_BUF_LENGTH         16u                 /**< SPI transaction buffer length. */

#if defined(SPI_MASTER_0_ENABLE)
    #define SPI_MASTER_HW SPI_MASTER_0
#elif defined(SPI_MASTER_1_ENABLE)
    #define SPI_MASTER_HW SPI_MASTER_1
#else
    #error "No SPI enabled"
#endif

spi_callback on_data_received = 0;

// Data buffers.
//static uint8_t m_rx_data[TX_RX_BUF_LENGTH] = {0}; /**< A buffer for incoming data. */

// Cleared when a transfer begins. Set when we notice it's ended.
static volatile bool m_transfer_completed = true;

static sat_spi_data_packet_t rx_data;
static sat_control_packet_t tx_command;


void output_pyserial_float(float value) {
	int32_t q;

	q = (int32_t) (value * (1<<30));  // Python demo uses fixed point +-1.30 bits

	simple_uart_put((q >> 24) & 0xff);
	simple_uart_put((q >> 16) & 0xff);
	simple_uart_put((q >> 8)  & 0xff);
	simple_uart_put(q & 0xff);
}

void output_pyserial_record(float qx, float qy, float qz, float qw) {
	simple_uart_put('$');
	simple_uart_put(0x02);
	simple_uart_put(0x00);
	output_pyserial_float(qw);
	output_pyserial_float(qx);
	output_pyserial_float(qy);
	output_pyserial_float(qz);
	simple_uart_put(0x00);
	simple_uart_put(0x00);
	simple_uart_put('\r');
	simple_uart_put('\n');
}

static inline char* data_as_string(sat_spi_data_packet_t data) {
	// TODO(cwajh): this is terrible.
	static char buf[200];
	//int record_count = data.record_count;
	//snprintf(buf,199,"<@%d-%d: %d sensor data points>",
	//         data.records[0].timestamp,
	//         data.records[(record_count)?(record_count-1):0].timestamp,
	//         record_count);
	if(data.timestamp == INVALID_TIMESTAMP) {
	snprintf(buf,199,"<Nothing to report>");
		
	} else {
	snprintf(buf,199,"<Record @%d [%f, %f, %f /%f]>",
	         data.timestamp,data.data.quat_x,data.data.quat_y,data.data.quat_z,data.data.quat_w);
	}
	return buf;
}

static void on_transfer_complete(void) {	
	char msg[200];
#ifdef HAS_CONSOLE
	if(rx_data.timestamp != INVALID_TIMESTAMP && rx_data.timestamp % 100 == 0) {
	snprintf(msg,199,"\r\nreceived %.70s",data_as_string(rx_data));
	simple_uart_putstring((uint8_t*)msg);
	//	output_pyserial_record(rx_data.data.quat_x,rx_data.data.quat_y,rx_data.data.quat_z,rx_data.data.quat_w);
	}
#endif
	if(on_data_received) {
		on_data_received(rx_data);
	}
	m_transfer_completed = true;
}

/**@brief Function for SPI master event callback.
 *
 * Upon receiving an SPI transaction complete event, checks if received data are valid.
 *
 * @param[in] spi_master_evt    SPI master driver event.
 */
static void spi_master_event_handler(spi_master_evt_t spi_master_evt)
{
	if(spi_master_evt.evt_type == SPI_MASTER_EVT_TRANSFER_COMPLETED){
		//on_transfer_complete();
	}
}


void start_spi(spi_callback callback) {
	on_data_received = callback;
	spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;
	spi_config.SPI_Pin_SCK = TS_SPI_CLK_PIN;
	spi_config.SPI_Pin_SS = TS_SPI_SS_PIN;
	spi_config.SPI_Pin_MISO = TS_SPI_MISO_PIN;
	spi_config.SPI_Pin_MOSI = TS_SPI_MOSI_PIN;
	spi_config.SPI_CONFIG_ORDER = SPI_CONFIG_ORDER_MsbFirst;
	spi_master_open(SPI_MASTER_HW, &spi_config);
  spi_master_evt_handler_reg(SPI_MASTER_HW, spi_master_event_handler);
}



typedef enum stop_step {
	STOP_READY,
	STOPPED,
	START_READY,
	RTC_SET,
	RUNNING,
}  stop_step_t;
typedef enum shush_step {
	SHUSH_READY,
	SHUSHED,
	SPEAK_READY,
	SPEAKING
} shush_step_t ;


#include <math.h>

static uint32_t sequence_number = 0;
static uint32_t rows_in_last_packet = 0;
static stop_step_t stop_step = START_READY;
static shush_step_t shush_step = SPEAKING;

union {
	sat_control_packet_t packet;
	uint8_t bytes[sizeof(rx_data)];
} raw_packet;
bool spi_send_command(sat_control_packet_t command) {
	tx_command = command;

	raw_packet.packet = tx_command;
	for(int i=sizeof(sat_control_packet_t);i<sizeof(rx_data);++i) {
		raw_packet.bytes[i] = 0xc2;
	}
	int sync_pattern_begin = sizeof(rx_data) - 6;
	for(int i=0;i<6;++i) {
		raw_packet.bytes[sync_pattern_begin+i] = i+0xfa;
	}
	uint32_t error_code =	spi_master_send_recv(SPI_MASTER_HW,
			(uint8_t*)(&raw_packet), sizeof(rx_data),
			(uint8_t*)&rx_data, sizeof(rx_data));
	if(error_code == NRF_SUCCESS) {
			m_transfer_completed = false;
		return true;
	}
	return false;
}

/// True IFF communication occurs
bool spi_master_pull_tick(void) {
	bool	should_communicate = true;
	sequence_number++;
	int shush_beat = sequence_number % REQUESTS_BETWEEN_SHUSHES;
	if(shush_step == SPEAKING && !shush_beat)
		shush_step = SHUSH_READY;
	else if(shush_step == SHUSHED && shush_beat > REQUESTS_PER_SHUSH)
		shush_step = SPEAK_READY;
	
	int stop_beat =  sequence_number % REQUESTS_BETWEEN_STOPS;
	if(stop_step == RUNNING && !stop_beat)
		stop_step = STOP_READY;
	else if(stop_step == STOPPED && stop_beat > REQUESTS_PER_STOP)
		stop_step = START_READY;
	
	switch(shush_step) {
		case SHUSH_READY:
			tx_command.command = SAT_SPI_WAIT;
			shush_step = SHUSHED;
			break;
		case SPEAK_READY:
			shush_step = SPEAKING;
			if(stop_step == RUNNING) {
				tx_command.command = SAT_SPI_START;
				break;
			}
		case SHUSHED:
		case SPEAKING:
			switch(stop_step) {
				case STOP_READY:
					tx_command.command = SAT_SPI_STOP;
					stop_step = STOPPED;
					break;
				case START_READY:
					tx_command.command = SAT_SPI_SET_RTC;
					tx_command.rtc_value = sequence_number;
					stop_step = RTC_SET;
					break;
				case RTC_SET:
					stop_step = RUNNING;
					if(shush_step == SPEAKING) {
						tx_command.command = SAT_SPI_START;
						break;
					}
				default:
					// No specific command needs sending. Decide whether to send a placeholder command.
					if(shush_step == SHUSHED) {
						// Other side shouldn't even be replying. Why bother?
						should_communicate = false;
						break;
					} else if (shush_step == SPEAKING && stop_step == RUNNING) {
						// All is well.
						tx_command.command = SAT_SPI_START;
						break;
					} else if (stop_step == STOPPED) {
						// If the satellite hasn't run out of things to say...
						tx_command.command = SAT_SPI_STOP;
						should_communicate = (rows_in_last_packet > 0);
						break;
					} else {
						// This shouldn't happen. Liminal states were handled in the switch()es.
						// Stable states should have been handled by the other branches.
						tx_command.command = SAT_SPI_CALIBRATE;
						break;
					}
					
					break;
			}
			break;
	}
	
	if(should_communicate) {
		spi_master_send_recv(SPI_MASTER_HW,
				(uint8_t*)(&tx_command), sizeof(tx_command),
				(uint8_t*)&rx_data, sizeof(rx_data));
				m_transfer_completed = false;
	}
	return should_communicate;
}

/// Returns true IFF transfer completed.
bool spi_master_check_tick(void) {
	spi_master_state_t spi_driver_state = spi_master_get_state(SPI_MASTER_HW);
	if (!m_transfer_completed && spi_driver_state == SPI_MASTER_STATE_IDLE) {
		on_transfer_complete();
		rows_in_last_packet = ((rx_data.timestamp == INVALID_TIMESTAMP)?0:1);
		//m_transfer_completed = false;
		return true;
	}
	return false;
}
