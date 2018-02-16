/*
 * Copyright (c) 2016 TuringSense
 * All rights reserved.
 *
 * bootloader_main.h
 *
 */
 
#ifndef __BOOTLOADER_MAIN_H__
#define __BOOTLOADER_MAIN_H__

#include "nordic_bl_types.h"

extern sat_to_nordic_boot_t spi_boot_rx;
extern nordic_to_sat_boot_t spi_boot_tx;

void jump_to_app(void);
void process_rx(void);

#endif /* __BOOTLOADER_MAIN_H__ */

