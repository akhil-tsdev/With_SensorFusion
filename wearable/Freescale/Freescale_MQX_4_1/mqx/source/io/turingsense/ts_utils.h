/*
 * Copyright TuringSense, Inc © 2015
 * ts_utils.h
 *
 *  Created on: Nov 25, 2015
 *      Author: cwati
 *
 *
 */
#ifndef __TS_UTILS_H__
#define __TS_UTILS_H__

#include <a_config.h>
#include <a_types.h>
#include <stdlib.h>
#include <ctype.h>
#include "atheros_stack_offload.h"

uint_32 mystrtoul(const char* arg, const char* endptr, int base);
int_32 ath_inet_aton (const char*  name,A_UINT32*     ipaddr_ptr);
char *inet_ntoa( A_UINT32 /*struct in_addr*/ addr, char *res_str );
char hextoa(int val);
int_32 parse_ipv4_ad(unsigned long * ip_addr, unsigned *  sbits, char *   stringin);
int ishexdigit(char digit);
unsigned int hexnibble(char digit);
unsigned int atoh(char * buf);

#if ENABLE_STACK_OFFLOAD
void app_time_delay(int msec);
void app_time_delay2(int msec);
void app_time_get_elapsed(TIME_STRUCT* time);
void app_time_get(TIME_STRUCT* time);
char * print_ip6(IP6_ADDR_T * addr, char * str);
char * inet6_ntoa(char* addr, char * str);
int Inet6Pton(char * src, void * dst);
#endif

#endif /* __TS_UTILS_H__ */
