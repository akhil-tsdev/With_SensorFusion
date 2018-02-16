/*
 * Copyright (c) 2016 TuringSense
 * All rights reserved.
 *
 * ts_hub_flash.c
 */

/* MQX include files */
#include <mqx.h>
#include <stdbool.h>
#include "fio.h"
#include "flashx.h"

/* TURINGSENSE */
#include "common_err.h"
#include "ts_hub_flash.h"

static MQX_FILE_PTR    flash_file;
static bool     initialized = false;

err_t hub_flash_init(void) {
  /* Open the flash device */
  flash_file = fopen(TS_FLASH_NAME, NULL);
  if (flash_file == NULL) {
      printf("\nUnable to open file %s", TS_FLASH_NAME);
      return E_FLASH_ERR;
  }
  initialized = true;
  return E_OK;
}

/* Read 32-bit from flash */
err_t hub_flash_read32(void* buffer, _mqx_int offset) {
  _mqx_int len;
  
  if (!initialized) {
    return E_FLASH_UNINIT;
  }
  /* Move STRING_SIZE offset Bytes back */  
  fseek(flash_file, offset, IO_SEEK_END);
  len = read(flash_file, buffer, TS_FLASH_READ_SIZE);
  
  if (len != TS_FLASH_READ_SIZE) {
    return E_FLASH_ERR;
  }
  
  return E_OK;
}

/* Erase sector */
err_t hub_flash_erase_ts_sector(void) {
  _mqx_int      len;
  uint32_t      ioctl_param;
  err_t         ret;
  
  if (!initialized) {
    return E_FLASH_UNINIT;
  }
  
  /* Move STRING_SIZE Bytes back.  I'm using HUB_ID_OFFset, but you can use any address
   * within the TS sector.  It will erase that whole sector. */
  fseek(flash_file, HUB_ID_OFF, IO_SEEK_END);
  
  /* Unprotecting the the FLASH might be required */
  ioctl_param = 0;
  ioctl(flash_file, FLASH_IOCTL_WRITE_PROTECT, &ioctl_param);
  
  /* Erase the sector */
  ret = ioctl(flash_file, FLASH_IOCTL_ERASE_SECTOR, &ioctl_param);
  
  return ret;
}

/* Write 32-bit from flash */
err_t hub_flash_write32(void* buffer, _mqx_int offset) {
  _mqx_int      len;
  uint32_t      ioctl_param;
 
  if (!initialized) {
    return E_FLASH_UNINIT;
  }

  /* Move STRING_SIZE Bytes back */
  fseek(flash_file, offset, IO_SEEK_END);

  len = write(flash_file, buffer, TS_FLASH_READ_SIZE);
  if (len != TS_FLASH_READ_SIZE) {
    //printf("\nError writing to the file. Error code: %d", _io_ferror(flash_file));
    return E_FLASH_ERR;
  }
  return E_OK;
}

uint32_t hub_flash_size(void) {
  
  if (!initialized) {
    return E_FLASH_UNINIT;
  }
  return ftell(flash_file);
}