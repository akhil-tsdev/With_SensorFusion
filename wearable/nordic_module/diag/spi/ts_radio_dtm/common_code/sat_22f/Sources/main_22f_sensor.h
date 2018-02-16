/*
 * Copyright (c) 2015 TuringSense
 *
 * main_22f_sensor.h
 *
 * Oct 7, 2015 - cwati
 */

#ifndef __MAIN_22F_SENSOR_H__
#define __MAIN_22F_SENSOR_H__

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
static bool get_raw_data_sample_ts(sensor_data_t *raw_data, sensor_record_t *data, uint32_t *timestamp);

#endif /* __MAIN_22F_SENSOR_H__ */
