#ifndef NORDIC_BOOTLOADER_H_
#define NORDIC_BOOTLOADER_H_

#include <stdbool.h>
#include "nrf.h"
#include "nordic_bl_types.h"

/* Nordic memory map
 * 0 to 0x40000 - 262144 bytes The whole thing
 *
 * 0 to 0x5000      Bootloader code
 * 0x5000 to 0x6000 Bootloader variables such as "APPLICATION_SELECT_ADDRESS" (size 0x1000).
 * 0x6000 to 0x21800 BANK 0 (size 0x1C800)
 * 0x22800 to 0x3E000 BANK 1 (size 0x1C800)
 * 0x3F000 to 0x40000 Not used yet (size 0x1000)
 */
#define BOOTLOADER_ADDRESS 0x00000
#define APPLICATION_BASE_ADDRESS 0x6000 //cwati test
#define APPLICATION_BACKUP_ADDRESS 0x22800 //cwati test
#define APPLICATION_SELECT_ADDRESS 0x05FF0
//#define APPLICATION_MAX_SIZE 0x30000
#define APPLICATION_MAX_SIZE 0x1C800
#define NUMBER_OF_CODE_BANKS 1	// 1 - single bank, 2 - dual bank

#if EVAL_BOARD
#include "boards.h"
extern const uint8_t leds_list[];
#endif

typedef enum {
	bank0 = 0,
	bank1 = 1,
} code_bank_t;

typedef enum {
	NORDIC_APP_RUN_AUTO = 0,		// automatically decide which code to run
	NORDIC_APP_RUN_CODE0 = 1,		// force to run code0
	NORDIC_APP_RUN_CODE1 = 2,		// run code1
	NORDIC_APP_MOVE_CODE1,			// data loading mode (sending hex data from sat to nordic)
	NORDIC_APP_RUN_BOOTLOADER,
	NORDIC_APP_NORMAL_RUN
} nordic_run_command_t;


typedef void (*application_main_t)(void);

uint32_t bootloader_mode(uint32_t index);
void bootloader_change_mode(uint32_t index, uint32_t mode);
void interrupts_disable(void);

bool is_in_bootloader_mode(void);
bool is_in_code0(void);
bool is_code_valid(code_bank_t i);

void bootloader_init(void);
//void bootloader_check_code(void);
//void bootloader_jump_to_app(int mode);
void bjump(void);

typedef enum {
	usb_update = 0,
	backup_update,
} update_type_t;

bool bootloader_erase_app(uint32_t base_address, uint32_t size, update_type_t);
bool bootloader_copy_app(void);

bool bootloader_write_record(hexparser_record * record);
bool bootloader_write_batch(hexparser_record record[]);
bool ts_copy_app(void);


#endif
