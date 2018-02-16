/* Copyright Turingsense, Inc (c) 2017 
 *
 * nordic_ts_flash.h
 *
 */
 
 #ifndef __NORDIC_TS_FLASH__
 #define __NORDIC_TS_FLASH__
 
 #include <stdbool.h>
 #include <stdint.h>
 #include "nrf.h"
 
 /* Memory region of Nordic
 * 0 to 0x40000 - 262144 bytes The whole thing
 *
 * 0 to 0x5000      Bootloader code
 * 0x5000 to 0x6000 Bootloader variables such as "APPLICATION_SELECT_ADDRESS" (size 0x1000).
 * 0x6000 to 0x21800 BANK 0 (size 0x1C800)
 * 0x22800 to 0x3E000 BANK 1 (size 0x1C800)
 * 0x3F000 to 0x40000 TS Flash (size 0x1000)
 */

#define FLASH_PAGE_SIZE							0x400

#define TS_FLASH_START_ADDR				 0x3F000

#define NORDIC_2ND_BANK_VALID_ADDR 0x3F000
#define NORDIC_FW_UPDATE_REQ_ADDR  0x3F004

#define UPDATE_REQUESTED 						0
#define UPDATE_NOT_REQUESTED				UINT32_MAX /* Cleared Flash */

#define BANK_IS_VALID								0
#define BANK_IS_INVALID 						UINT32_MAX /* Cleared Flash */

uint32_t flash_read_mem(uint32_t address);
bool flash_write_mem(uint32_t address, uint32_t value);
bool flash_erase_mem(uint32_t address);

#endif 
