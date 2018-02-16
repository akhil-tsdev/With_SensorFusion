#include <stdint.h>
//#include "psptypes.h"
#include "MK22F25612.h"
//#include "lwadc_kadc.h"
//#include "queue.h"
//#include "mutex.h"
#include "common_types.h"

#define NUMSIMPLE 150
static float avgResult1[NUMSIMPLE] = {0};
static float avgResult2[NUMSIMPLE] = {0};
static float multiplicativeVREF = 0;
static uint32_t avgResult1_posX = 0;

uint32_t adc_read_1(uint8_t channel);
float readBatteryApplyFilter(uint32_t res16);
