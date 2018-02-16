/*
 * Copyright (c) 2015 TuringSense
 *
 * main_22f_sensor_rtcclkout.h
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

//static void simpleReset(bool magBiasSending, float magBiasX, float magBiasY, float magBiasZ);
static void simpleReset(int16_t setOutputMode, int16_t accSensX, int16_t accSensY, int16_t accSensZ,
		int16_t accBiasX, int16_t accBiasY, int16_t accBiasZ, int16_t gyrBiasX, int16_t gyrBiasY,
		int16_t gyrBiasZ, float magBiasX, float magBiasY, float magBiasZ);

static bool get_raw_data_sample(sensor_data_t *raw_data, sensor_record_t *data);
static bool get_raw_data_sample_ts(sensor_data_t *raw_data, sensor_record_t *data, uint32_t *timestamp);

#endif /* __MAIN_22F_SENSOR_H__ */
