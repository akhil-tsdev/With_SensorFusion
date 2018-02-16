/*
 * Copyright (c) 2016 TuringSense
 * All rights reserved.
 *
 * ts_hub_flash.h
 */

#ifndef __TS_HUB_FLASH_H__
#define __TS_HUB_FLASH_H__

#define TS_FLASH_NAME "flashx:bank3" /* last bank */
#define TS_FLASH_READ_SIZE (4)
#define TS_BUFFER_SIZE     (80)

#define TS_SATID_LIST_SIZE 15

/* Offset is per 64-bit, or 8 bytes */
#define TS_FLASH_OFFSET(x) ((x) * -8)

/*
 * Hub flash starts at 0xF_F000.
 * Size is 1 sector (0x1000)
 */
#define HUB_ID_OFF      TS_FLASH_OFFSET(1)      /* 0x10_0000 - (1*8) = 0xFFFF8 */

/* Space for 15 satellites */
#define SAT_ID_0_OFF    TS_FLASH_OFFSET(2)      /* 0x10_0000 - (2*8) = 0xFFFF0 */
#define SAT_ID_1_OFF    TS_FLASH_OFFSET(3)
#define SAT_ID_2_OFF    TS_FLASH_OFFSET(4)
#define SAT_ID_3_OFF    TS_FLASH_OFFSET(5)
#define SAT_ID_4_OFF    TS_FLASH_OFFSET(6)
#define SAT_ID_5_OFF    TS_FLASH_OFFSET(7)
#define SAT_ID_6_OFF    TS_FLASH_OFFSET(8)
#define SAT_ID_7_OFF    TS_FLASH_OFFSET(9)
#define SAT_ID_8_OFF    TS_FLASH_OFFSET(10)
#define SAT_ID_9_OFF    TS_FLASH_OFFSET(11)
#define SAT_ID_10_OFF   TS_FLASH_OFFSET(12)
#define SAT_ID_11_OFF   TS_FLASH_OFFSET(13)
#define SAT_ID_12_OFF   TS_FLASH_OFFSET(14)
#define SAT_ID_13_OFF   TS_FLASH_OFFSET(15)
#define SAT_ID_14_OFF   TS_FLASH_OFFSET(16)

#define HUB_LCP_OFF     TS_FLASH_OFFSET(26)     /* Nordic channel Pair */
#define HUB_TYP_OFF     TS_FLASH_OFFSET(27)     /* 0 for Wifi direct. Others for router connection. */
#define HUB_IPA_OFF     TS_FLASH_OFFSET(28)     /* IP Address */



err_t hub_flash_init();
err_t hub_flash_read32(void* buffer, _mqx_int offset);
err_t hub_flash_write32(void* buffer, _mqx_int offset);
uint32_t hub_flash_size(void);
err_t hub_flash_erase_ts_sector(void);

#endif /* __TS_HUB_FLASH_H__ */

