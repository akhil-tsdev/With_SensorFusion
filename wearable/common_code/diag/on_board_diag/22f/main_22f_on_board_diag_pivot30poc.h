/*
 * Copyright (c) 2015 TuringSense
 *
 * main_22f_sensor.h
 *
 * Oct 7, 2015 - cwati
 */

#ifndef __MAIN_22F_SENSOR_DTM_H__
#define __MAIN_22F_SENSOR_DTM_H__

#include "common_types.h"
#include "mpu9250_ossf.h"

#define READ_MAG_DIRECTLY			0

extern uint8_t valid_num_sensors_in_cs;
extern uint8_t valid_cs[];
/* [PV-1038] Bringing up Sensor Rev E */
#define TMP_VERSION		"10.1"
#define TMP_VERSION_DATE "June 13, 2017"
#define DATA_AVAIL_PERIOD (1000 / TS_GYRO_FREQ)
#define CWATI_NO_PRINTF	0

typedef enum {
	offLed,
	redLed,
	greenLed,
	blueLed,
	yellowLed,
	turqoiseLed,
	purpleLed
} sat_led_t;

static void simpleReset(bool magBiasSending, float magBiasX, float magBiasY, float magBiasZ);
static bool get_raw_data_sample(sensor_data_t *raw_data, sensor_record_t *data);
//static bool get_raw_data_sample_ts(sensor_data_t *raw_data, sensor_record_t *data, uint32_t *timestamp);
void blinkLEDerror(sat_led_t color, uint32_t multiplier);
void turnOnLED(sat_led_t color);
dspi_status_t nrf51822_init();
void nrf51822_swdio_reset();

#endif /* __MAIN_22F_SENSOR_DTM_H__ */
