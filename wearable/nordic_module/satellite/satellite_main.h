#ifndef __SAT_MAIN_H__
#define __SAT_MAIN_H__

#include <stdint.h>
#include "net_common.h"

extern uint32_t cur_num_of_sat;
void satellite_rx_handler(void* packet, net_packet_type_t);

#endif
