/*
 * Copyright (c) 2016 TuringSense
 *
 * ts_rtc.h
 *
 * May 12, 2016 - cwati
 */
#ifndef __TS_RTC_H__
#define __TS_RTC_H__

#define ENABLE_RTC		1

uint32_t k22f_get_rtc(void);
//void k22f_enable_rtc(void);
void k22f_set_rtc(uint32_t);

#endif /* __TS_RTC_H__ */
