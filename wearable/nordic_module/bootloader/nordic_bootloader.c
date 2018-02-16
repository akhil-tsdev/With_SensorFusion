#include <stdbool.h>
#include "nordic_bootloader.h"
#include "nrf_delay.h"
#include "nordic_ts_flash.h"

#define IRQ_ENABLED             0x01           /**< Field identifying if an interrupt is enabled. */
#define MAX_NUMBER_INTERRUPTS   32

static bool is_bootloader_running = false;
static bool is_running_code0 = true;
//static bool is_app_code_valid[2];
//static uint32_t code_address[2];

bool is_in_bootloader_mode(void)
{
	return is_bootloader_running;
}

bool is_in_code0(void)
{
	return is_running_code0;
}

//cwati TODO must do checksum check for greater reliability in checking code validity
bool is_code_valid(code_bank_t i)
{
	if (i == bank0) {
		if (flash_read_mem(APPLICATION_BASE_ADDRESS) != 0xFFFFFFFF) {
			return true;
		} else {
			return false;
		}
	} else if (i == bank1) {
		if (flash_read_mem(APPLICATION_BACKUP_ADDRESS) != 0xFFFFFFFF) {
			return true;
		} else {
			return false;
		}		
	}
	//return is_app_code_valid[i];
	return false;
}

bool bootloader_erase_app(uint32_t base_address, uint32_t size, update_type_t type)
{
    uint32_t last_page_address;
	
		if (type == usb_update) {
			if (size == 0) {
				last_page_address = base_address + APPLICATION_MAX_SIZE;
			} else {
				last_page_address = base_address + ((size * 0x0010) + 0x1000);
			}
		} else {
			last_page_address = (base_address + size);
		}
		//NRF_FICR->CODEPAGESIZE * NRF_FICR->CODESIZE;

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos;
    for (uint32_t address = base_address; address < last_page_address; address += NRF_FICR->CODEPAGESIZE)
    {
#if EVAL_BOARD
				LEDS_ON(1 << leds_list[2]);
#endif
        NRF_NVMC->ERASEPAGE = address;
        while (!NRF_NVMC->READY);
#if EVAL_BOARD
				nrf_delay_us(10000);
				LEDS_OFF(1 << leds_list[2]);
				nrf_delay_us(10000);
#endif
    }
    NRF_NVMC->CONFIG = 0x0;
	return true;
}


//bool bootloader_copy_app(void)
//{
//	// copy code from CODE1 to CODE0
//	uint32_t base_address = APPLICATION_BASE_ADDRESS;
//	uint32_t last_page_address = base_address + APPLICATION_MAX_SIZE;
//		//NRF_FICR->CODEPAGESIZE * NRF_FICR->CODESIZE;

//    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
//	uint32_t address = base_address;
//	uint32_t src_address = base_address + APPLICATION_MAX_SIZE;
//    for ( ; address < last_page_address; address ++, src_address++)
//    {
//        //NRF_NVMC->ERASEPAGE = address;
//		uint32_t value = *(uint32_t *)(src_address);
//		*(uint32_t *) (address) = value;
//		while (NRF_NVMC->READY & (NVMC_READY_READY_Busy << NVMC_READY_READY_Pos)){}
//    }
//    NRF_NVMC->CONFIG = 0x0;
//	return true;
//}
	
bool bootloader_copy_app(void)
{
	// copy code from CODE1 to CODE0
	uint32_t base_address = APPLICATION_BASE_ADDRESS;
	uint32_t last_page_address = base_address + APPLICATION_MAX_SIZE;
	//NRF_FICR->CODEPAGESIZE * NRF_FICR->CODESIZE;

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
	uint32_t dest_address = base_address;
	uint32_t src_address = base_address + APPLICATION_MAX_SIZE; /* equal to APPLICATION_BACKUP_ADDRESS */
	
	for ( ; dest_address < last_page_address; dest_address ++, src_address++)
	{
		//NRF_NVMC->ERASEPAGE = address;
		uint32_t value = *(uint32_t *)(src_address);
		*(uint32_t *) (dest_address) = value;
	//	while (NRF_NVMC->READY & (NVMC_READY_READY_Busy << NVMC_READY_READY_Pos)){}
	}
	
	NRF_NVMC->CONFIG = 0x0;
	return true;
}

uint32_t bootloader_mode(uint32_t index)
{
	uint32_t address = APPLICATION_SELECT_ADDRESS + (4 * index);
	return flash_read_mem(address);
}

void bootloader_change_mode(uint32_t index, uint32_t mode)
{
	uint32_t address = APPLICATION_SELECT_ADDRESS + (4 * index);
	flash_write_mem(address, mode);
}


	
//void bootloader_check_code(void)
//{
//	uint32_t address = APPLICATION_BASE_ADDRESS;
//	is_app_code_valid[0] = is_app_code_valid[1] = false;
//	for (int i = 0; i < NUMBER_OF_CODE_BANKS; i++) {
//		code_address[i] = address;
//		if (flash_read_mem(address) == 0xFFFFFFFF) {
//			is_app_code_valid[i] = false;
//		} else {
//			is_app_code_valid[i] = true;
//		}
//		address += APPLICATION_MAX_SIZE;
//	}
//}

void bootloader_init(void)
{
	if (0 && NRF_UICR->CLENR0 == 0xFFFFFFFF)
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
        NRF_UICR->CLENR0 = APPLICATION_BASE_ADDRESS;	//APPLICATION_SELECT_ADDRESS;
        while (!NRF_NVMC->READY);
        NRF_NVMC->CONFIG = 0;
        NVIC_SystemReset();
    }
	//code_address[0] = APPLICATION_BASE_ADDRESS;
	/*
	uint32_t address = APPLICATION_BASE_ADDRESS;
	is_app_code_valid[0] = is_app_code_valid[1] = false;
	for (int i = 0; i < NUMBER_OF_CODE_BANKS; i++) {
		code_address[i] = address;
		if (flash_read_mem(address) == 0xFFFFFFFF) {
			is_app_code_valid[i] = false;
		} else {
			is_app_code_valid[i] = true;
		}
		address += APPLICATION_MAX_SIZE;
	}
	*/
}


/** Function for disabling all interrupts before jumping from bootloader to application.
 */
void interrupts_disable(void)
{
    uint32_t interrupt_setting_mask;
    uint8_t  irq;

    // We start the loop from first interrupt, i.e. interrupt 0.
    irq                    = 0;
    // Fetch the current interrupt settings.
    interrupt_setting_mask = NVIC->ISER[0];
    
    for (; irq < MAX_NUMBER_INTERRUPTS; irq++)
    {
        if (interrupt_setting_mask & (IRQ_ENABLED << irq))
        {
            // The interrupt was enabled, and hence disable it.
            NVIC_DisableIRQ((IRQn_Type) irq);
        }
    }        
}

__asm void StartApplication(uint32_t start_addr)
{
    LDR   R2, [R0]               ; Get App MSP.
    MSR   MSP, R2                ; Set the main stack pointer to the applications MSP.
    LDR   R3, [R0, #0x00000004]  ; Get application reset vector address.
    BX    R3                     ; No return - stack code is now activated only through SVC and plain interrupts.
    ALIGN
}


//void bootloader_jump_to_app(int mode)
//{
//	uint32_t current_mode = mode;
//	if (!is_app_code_valid[0] && !is_app_code_valid[1]) {
//		// if no valid code, then stay in bootloader mode
//		current_mode = NORDIC_APP_RUN_BOOTLOADER;
//	}
//	
//	uint32_t app_address = 0;
//	switch (current_mode) {
//		case NORDIC_APP_RUN_BOOTLOADER:
//			// running the boot loader mode
//			// waiting for SPI command from 22F
//			is_bootloader_running = true;
//			break;
//		case NORDIC_APP_RUN_CODE0:
//			if (is_app_code_valid[0]) {
//				app_address = code_address[0];
//			}
//			break;
//		case NORDIC_APP_RUN_CODE1:
//			if (is_app_code_valid[1]) {
//				app_address = code_address[1];
//			}
//			break;
//		case NORDIC_APP_RUN_AUTO:
//			if (is_app_code_valid[1]) {
//				if (is_app_code_valid[0]) {
//					// both CODE0 and CODE1 exist,
//					// so just run CODE1
//					app_address = code_address[1];
//				} else {
//					// CODE1 exist but CODE0 doesn't
//					// so we need to copy CODE1 to CODE0
//					// and run from CODE0
//					app_address = code_address[0];
//					current_mode = NORDIC_APP_MOVE_CODE1;
//				}
//			} else {
//				if (is_app_code_valid[0]) {
//					// only CODE0 exist, so just run CODE0
//					app_address = code_address[0];
//				}
//			}
//			break;
//	}
//	if (current_mode == NORDIC_APP_MOVE_CODE1) {
//		bootloader_erase_app(code_address[0], 0);
//		bootloader_copy_app();
//		// need to check if the copy is complete
//		bootloader_erase_app(code_address[1], 0);
//	}
//	// the next reset, change to normal mode
//	//bootloader_change_mode(0x00000000, NORDIC_APP_RUN_AUTO);
//	
//	if (app_address) {
//		application_main_t application_main = *(application_main_t *)(app_address);
//		if (application_main != (application_main_t) 0xFFFFFFFF) {
//			interrupts_disable();
//			StartApplication(app_address);
//		}
//	}
//				
///*			
//	if (current_mode == NORDIC_APP_RUN_BOOTLOADER) {
//		is_bootloader_running = true;
//		// the next reset, change to normal mode
//		bootloader_change_mode(0x00000000, NORDIC_APP_NORMAL_RUN);
//	} else {
//		is_bootloader_running = false;
//		if (is_running_code0) {
//			// currently running in code0
//			application_main_t application_main = *(application_main_t *)(APPLICATION_BASE_ADDRESS+4);
//			if (current_mode != NORDIC_APP_RUN_CODE0 
//					&& application_main != (application_main_t) 0xFFFFFFFF) {
//				// code1 exist, so just jump to code1
//				bootloader_change_mode(0x00000000, NORDIC_APP_RUN_CODE1);
//				application_main();
//			} else {
//				// code1 doesn't exist, or we are forced to run code0
//				bootloader_change_mode(0x00000000, NORDIC_APP_NORMAL_RUN);
//			}
//		} else {
//			// currently running in code1
//			bootloader_change_mode(0x00000000, NORDIC_APP_NORMAL_RUN);
//		}
//	}
//	*/
//}



void bjump(void)
{
	StartApplication(APPLICATION_BASE_ADDRESS);
}

bool bootloader_write_record(hexparser_record * record)
{
	if (!hexparser_is_record_valid(record)) {
			return false;
	}
	return true;
#if 0
	static uint16_t base_address = 0;

	if (record->type == EXTENDED_LINEAR_ADDRESS_RECORD
			|| record->type == EXTENDED_SEGMENT_ADDRESS_RECORD) {
        base_address = record->data.words[0];
	} else if (record->type ==  DATA_RECORD) {		
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
		uint32_t i;
		for (i = 0; i < record->byte_count / 4; i++)
		{
			uint32_t address = (base_address << 4 | (record->address + (4 * i)));// + APPLICATION_BASE_ADDRESS;
			*(uint32_t *) (address) = record->data.words[i];
			while (NRF_NVMC->READY & (NVMC_READY_READY_Busy << NVMC_READY_READY_Pos))
			{
			}
		}
		NRF_NVMC->CONFIG = 0;
	}
	return true;
#endif /* cwati: unreached statement @stefanus */
}


bool bootloader_write_batch(hexparser_record record[])
{
	uint32_t id;
	for (id = 0; id < NUM_HEX_PER_SPI_PACKET; id++) {
		if (!hexparser_is_record_valid(&record[id])) {
			return false;
		}
	}
	static uint16_t base_address = 0;
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
	for (id = 0; id < NUM_HEX_PER_SPI_PACKET; id++) {
		if (record[id].type == EXTENDED_LINEAR_ADDRESS_RECORD
				|| record[id].type == EXTENDED_SEGMENT_ADDRESS_RECORD) {
			base_address = record[id].data.words[0];
		} else if (record[id].type ==  DATA_RECORD) {		
			uint32_t i;
			for (i = 0; i < record[id].byte_count / 4; i++)
			{
				uint32_t address = (base_address << 4 | (record[id].address + (4 * i)));// + APPLICATION_BASE_ADDRESS;
				*(uint32_t *) (address) = record[id].data.words[i];
				while (NRF_NVMC->READY & (NVMC_READY_READY_Busy << NVMC_READY_READY_Pos))
				{
				}
			}
		}
	}
	NRF_NVMC->CONFIG = 0;
	return true;
}

bool ts_copy_app(void)
{
	// copy code from CODE1 to CODE0
	uint32_t base_address = APPLICATION_BASE_ADDRESS;
	uint32_t last_page_address = base_address + APPLICATION_MAX_SIZE;
	//NRF_FICR->CODEPAGESIZE * NRF_FICR->CODESIZE;

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
	uint32_t dest_address = base_address;
	uint32_t src_address = APPLICATION_BACKUP_ADDRESS; /* equal to APPLICATION_BACKUP_ADDRESS */
	
#if EVAL_BOARD
		LEDS_ON(1 << leds_list[3]);
#endif
	for ( ; dest_address < last_page_address; )
	{
		//NRF_NVMC->ERASEPAGE = address;
		uint32_t value = flash_read_mem(src_address);

		*(uint32_t *) (dest_address) = value;
		while (NRF_NVMC->READY & (NVMC_READY_READY_Busy << NVMC_READY_READY_Pos))
		{
		}
		dest_address += 4;
		src_address += 4;

	}
#if EVAL_BOARD
	LEDS_OFF(1 << leds_list[3]);
#endif
	
	NRF_NVMC->CONFIG = 0x0;
	return true;
}

