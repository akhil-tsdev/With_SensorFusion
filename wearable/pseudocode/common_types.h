#include "stdint.h"

typedef enum {
	SAT_SPI_INVALID,
	SAT_SPI_START,
	SAT_SPI_STOP,
	SAT_SPI_WAIT,
	SAT_SPI_CALIBRATE,
	SAT_SPI_SET_RTC
} sat_spi_command;
typedef enum {
	ESB_INVALID,
	ESB_NOT_YOU,
	ESB_START,
	ESB_STOP,
	ESB_WAIT,
	ESB_CALIBRATE,
	ESB_SET_RTC
} esb_command;
typedef enum {
	ESB_CONNECT,
	ESB_DATA
} esb_request_type;
typedef enum {
	HUB_SPI_INVALID,
	HUB_SPI_START,
	HUB_SPI_STOP,
	HUB_SPI_WAIT,
	HUB_SPI_CALIBRATE,
	HUB_SPI_SET_RTC,
	HUB_SPI_SET_SATELLITES
} hub_spi_command;
typedef enum {
	WIFI_INVALID,
	WIFI_START,
	WIFI_STOP,
	WIFI_WAIT,
	WIFI_CALIBRATE,
	WIFI_SET_RTC,
	WIFI_SET_SATELLITES
} wifi_command;

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} sensor_triple;

typedef struct {
	uint32_t timestamp;
	sensor_triple accelerometer_reading;
	sensor_triple magnetometer_reading;
	sensor_triple gyroscope_reading;
	sensor_triple linear_acceleration;
	sensor_triple quaternion_vector;
	uint16_t quaternion_theta;
} sensor_record;

typedef union {
	uint32_t rtc_value;
	uint32_t satellite_ids[15];
} hub_command_payload;

#define MAX_SENSOR_RECORDS_PER_PACKET 16

typedef struct {
	uint8_t record_count;
	sensor_record records[MAX_SENSOR_RECORDS_PER_PACKET];
} data_packet;

typedef struct {
	esb_request_type type;
	union {
		uint32_t satellite_id;
		data_packet sensor_data;
	} payload;
} esb_request_packet;

typedef struct {
	sat_spi_command command;
	uint32_t rtc_value;
} sat_mosi_packet;

typedef struct {
	esb_command command;
	uint32_t rtc_value;
} esb_response_packet;

typedef struct {
	hub_spi_command command;
	hub_command_payload payload;
} hub_mosi_packet;

typedef struct {
	wifi_command command;
	hub_command_payload payload;
} wifi_request_packet;