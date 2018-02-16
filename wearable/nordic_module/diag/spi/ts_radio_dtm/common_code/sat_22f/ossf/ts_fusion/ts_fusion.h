/*
 * ts_fusion.h
 *
 *  Created on: Sep 5, 2015
 *      Author: cwati
 */

#ifndef FUSION_TS_FUSION_H_
#define FUSION_TS_FUSION_H_

#include "common_types.h"
#include "mpu9250_firmware.h"

void ts_fusion_init();
void process_sensor_fusion(sensor_data_t *raw_data, sensor_record_t* data);

#endif /* FUSION_TS_FUSION_H_ */
