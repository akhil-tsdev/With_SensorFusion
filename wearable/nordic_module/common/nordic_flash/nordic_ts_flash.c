/* Copyright Turingsense, Inc (c) 2017 
 *
 * nordic_ts_flash.c
 *
 */

#include "nordic_ts_flash.h"

uint32_t flash_read_mem(uint32_t address)
{
	uint32_t data;
	data = *(uint32_t *)(address);
	return data;
}

bool flash_write_mem(uint32_t address, uint32_t value)
{
	if (flash_read_mem(address) != 0xFFFFFFFF) {
		flash_erase_mem(address);
	}
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
	*(uint32_t *) (address) = value;
	while (NRF_NVMC->READY & (NVMC_READY_READY_Busy << NVMC_READY_READY_Pos)){}
	NRF_NVMC->CONFIG = 0;
	return true;
}

bool flash_erase_mem(uint32_t address)
{
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos;
    NRF_NVMC->ERASEPAGE = address;
    while (!NRF_NVMC->READY);
    NRF_NVMC->CONFIG = 0x0;
	return true;
}


