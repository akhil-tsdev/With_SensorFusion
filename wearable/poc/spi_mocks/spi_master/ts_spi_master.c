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
#include "../spi_types.h"

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

// Data buffers.
//static uint8_t m_rx_data[TX_RX_BUF_LENGTH] = {0}; /**< A buffer for incoming data. */

static volatile bool m_transfer_completed = false; /**< A flag to inform about completed transfer. */


typedef struct padded_nord {
	uint32_t 							buf[1];
	nordic_to_hub_t  			packet;
} padded_nordic_to_hub_t;

padded_nordic_to_hub_t				padded_rx;
static nordic_to_hub_t 				rx_data; 

static hub_to_nordic_t 				tx_command;



void debug_putstring(uint8_t* data) {
	static uint8_t scraps[17] = {0};
	static uint8_t len_scraps = 0;
	uint8_t scraps_pos = 0;
	uint8_t data_pos = 0;
	uint8_t current_buffer[18];
	int buffer_pos=0;
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

static void output_pyserial_float(float value) {
	int32_t q;

	union {
		float f;
		uint8_t b[sizeof(float)];
	} float_to_bytes;
	
	float_to_bytes.f = value;
	simple_uart_put(float_to_bytes.b[0]);
	simple_uart_put(float_to_bytes.b[1]);
	simple_uart_put(float_to_bytes.b[2]);
	simple_uart_put(float_to_bytes.b[3]);
	/*
	q = (int32_t) (value * (1<<30));  // Python demo uses fixed point +-1.30 bits

	simple_uart_put((q >> 24) & 0xff);
	simple_uart_put((q >> 16) & 0xff);
	simple_uart_put((q >> 8)  & 0xff);
	simple_uart_put(q & 0xff);*/
}

static void output_pyserial_record(float qx, float qy, float qz, float qw, uint8_t sat_id) {
	simple_uart_put('$');
	simple_uart_put(0x02);
	simple_uart_put(sat_id);
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
	char msg[200]={0};
	static int i=0;
	i++;
	if(i%100 == 0) {
		int sensor_second = i/100;
		msg[5] = 48 + sensor_second%10;
		msg[4] = 48 + sensor_second/10 % 10;
		msg[3] = 48 + sensor_second/100 % 10;
		msg[2] = 48 + sensor_second/1000 % 10;
		msg[1] = '\n';
		msg[0] = '\r';
		
		//msg[1] = 48 + sensor_second/10000 % 10;
		//msg[0] = 48 + sensor_second/100000 % 10;
		debug_putstring((uint8_t*)msg);
	}
	static int framedrop[20]={0};
	rx_data=padded_rx.packet;
	for(int i=0;i<MAX_SENSOR_RECORDS_PER_PACKET;++i) {
		sat_sensor_record_t record = rx_data.record[i];
		
		if(record.timestamp != INVALID_TIMESTAMP && record.satellite_id != INVALID_TIMESTAMP && (framedrop[record.satellite_id%20]++ % 20 == 0)) {
			output_pyserial_record(record.data.quat_x,record.data.quat_y,record.data.quat_z,record.data.quat_w,record.satellite_id);
		}

	}
		/*snprintf(msg,199,"\r\nreceived %.70s",data_as_string(rx_data));
		debug_putstring((uint8_t*)msg);
		
			debug_putstring((uint8_t*)"\r\n< ");
			for(int i=0;i<sizeof(rx_data);++i) {
				snprintf(msg,199,"%02.02x ",((uint8_t*)(&rx_data))[i]);
				debug_putstring((uint8_t*)msg);
			}
			debug_putstring((uint8_t*)">");*/
		/*
		i=(i+1)%8;
		output_pyserial_record((i&1?1:-1)*rx_data.data.quat_x,
		                       (i&2?1:-1)*rx_data.data.quat_y,
		                       (i&4?1:-1)*rx_data.data.quat_z,rx_data.data.quat_w,i);
		*/
	//if(on_data_received) {
	//	on_data_received(rx_data);
	//}
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

void start_spi(void) {
	spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;
	spi_config.SPI_Freq = SPI_FREQUENCY_FREQUENCY_M8;
	spi_config.SPI_Pin_SCK = TS_SPI_CLK_PIN;
	spi_config.SPI_Pin_SS = TS_SPI_SS_PIN;
	spi_config.SPI_Pin_MISO = TS_SPI_MISO_PIN;
	spi_config.SPI_Pin_MOSI = TS_SPI_MOSI_PIN;
	spi_config.SPI_CONFIG_CPHA = 0;
	spi_config.SPI_CONFIG_CPOL = 0;
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


bool control_button_pressed(uint8_t id) {
	uint32_t buttons[] = BUTTONS_LIST;
	static bool control_button_was_down[4] = {false,false,false,false};
	bool control_button_is_down = !nrf_gpio_pin_read(buttons[id]);
	bool press = (!control_button_was_down[id] && control_button_is_down);
	control_button_was_down[id] = control_button_is_down;
	return press;	
}

void set_light(uint8_t id, bool on) {
	uint32_t leds[] = LEDS_LIST;
	if(on)
	{
		nrf_gpio_pin_clear(leds[id]);
	} else {
		nrf_gpio_pin_set(leds[id]);
	}
}

bool spi_master_pull(void) {
	static bool active = false;
	static bool speaking = true;
	bool set_rtc = control_button_pressed(2);
	if(control_button_pressed(0)) {
		active = !active;
	}		
	if(control_button_pressed(1)) {
		speaking = !speaking;
	}
	set_light(0,active);
	set_light(1,speaking);
	
	uint32_t command = (active? NORDIC_START :0) | 
	                   (speaking? 0: NORDIC_WAIT) |
                     (set_rtc? NORDIC_SET_RTC :0);
	
	bool new_instructions = (command != tx_command.command);
	bool all_active = active && speaking;
	bool draining = !active && speaking && (rows_in_last_packet > 0);
	bool should_communicate = new_instructions || all_active || draining;

	if(should_communicate) {
		tx_command.command = command;
		spi_master_send_recv(SPI_MASTER_HW,
				(uint8_t*)(&tx_command), sizeof(tx_command),
				(uint8_t*)&padded_rx, sizeof(padded_rx));
				m_transfer_completed = false;
	}
	return should_communicate;
}

/// True IFF communication occurs
bool spi_master_pull_tick(void) {
	bool	should_communicate = false;
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
	
	tx_command.command = 0;
	
	switch(shush_step) {
		case SHUSH_READY:
			shush_step = SHUSHED;
			should_communicate = true;
		case SHUSHED:
			tx_command.command |= NORDIC_WAIT;
			break;
		
		case SPEAK_READY:
			shush_step = SPEAKING;
		case SPEAKING:
			should_communicate = true;
			break;
	}

	switch(stop_step) {
		case START_READY:
			tx_command.command |= NORDIC_SET_RTC;
			tx_command.payload.rtc_value = sequence_number;
			stop_step = RTC_SET;
		case RTC_SET:
			stop_step = RUNNING;
			should_communicate = true;
		case RUNNING:
			tx_command.command |= NORDIC_START;
			break;
		case STOP_READY:
			stop_step = STOPPED;
			should_communicate = true;
		case STOPPED:
			should_communicate = should_communicate || (shush_step == SHUSHED && (rows_in_last_packet > 0));
			break;
	}
	
	if(should_communicate) {
		spi_master_send_recv(SPI_MASTER_HW,
				(uint8_t*)(&tx_command), sizeof(tx_command),
				(uint8_t*)&padded_rx, sizeof(padded_rx));
				m_transfer_completed = false;
	}
	return should_communicate;
}

/// Returns true IFF transfer completed.
bool spi_master_check_tick(void) {
	if (!m_transfer_completed && spi_master_get_state(SPI_MASTER_HW) == SPI_MASTER_STATE_IDLE) {
		on_transfer_complete();
		rows_in_last_packet = 0;
		for(int i=0;i<MAX_SENSOR_RECORDS_PER_PACKET;++i) {
			if(rx_data.record[i].timestamp != INVALID_TIMESTAMP) {
				rows_in_last_packet++;
			}
		}
		//m_transfer_completed = false;
		return true;
	}
	return false;
}
