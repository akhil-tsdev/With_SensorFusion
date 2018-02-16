//------------------------------------------------------------------------------
// Copyright (c) 2011 Qualcomm Atheros, Inc.
// All Rights Reserved.
// Qualcomm Atheros Confidential and Proprietary.
// Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is
// hereby granted, provided that the above copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE
// INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
// USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
//------------------------------------------------------------------------------
//==============================================================================
// Author(s): ="Atheros"
//==============================================================================

/*
this file just define the register address for TMP106.
and also define the common operation on KL25 MCU for I2C operation
*/

#ifndef _TMP106_H_
#define _TMP106_H_

#include "main.h"

#define TMP106_SLV_ADDR_0  0x49
#define TMP106_SLV_ADDR_1  0x49

#define REG_TMP106_TMP_REG   0  // the temperature Register (Read Only)
#define REG_TMP106_CFG_REG   1  // Configuration Register (R/W)
#define REG_TMP106_LOW_REG   2  //T_LOW Register (R/W)
#define REG_TMP106_HIGH_REG  3  //T_HIGHT Register (R/W)


#define TMP106_TMP_REG_LEN sizeof(uint16_t)
#define TMP106_CFG_REG_LEN sizeof(uint8_t)
#define TMP106_HIGH_REG_LEN sizeof(uint16_t)
#define TMP106_LOW_REG_LEN sizeof(uint16_t)


typedef struct {
  volatile boolean Sent;
  volatile boolean Received;
} I2C_TDataState;


#define TMP106_CFG_SD_SHIFT 0
#define TMP106_CFG_SD_MASK  0x1
#define TMP106_CFG_TM_SHIFT 1
#define TMP106_CFG_TM_MASK  0x2
#define TMP106_CFG_POL_SHIFT 2
#define TMP106_CFG_POL_MASK  0x4
#define TMP106_CFG_FQ_SHIFT 3
#define TMP106_CFG_FQ_MASK  0x18
#define TMP106_CFG_CVT_SHIFT 5
#define TMP106_CFG_CVT_MASK  0x60
#define TMP106_CFG_OS_SHIFT 7
#define TMP106_CFG_OS_MASK  0x80


typedef enum {
  TMP106_COMPARATOR_MODE = 0,
  TMP106_INTERRUPT_MODE = 0x2,
}TMP106_THERMOSTAT_MODE;

typedef enum {
  TMP106_POLAR_LOW = 0,
  TMP106_POLAR_HIGH = 0x4, 
}TMP106_POLAR_MODE;

typedef enum {
  TMP106_FAULT_QUEUE_1 = 0,
  TMP106_FAULT_QUEUE_2 = 0x8,
  TMP106_FAULT_QUEUE_4 = 0x10,
  TMP106_FAULT_QUEUE_6 = 0x18,
}TMP106_CONSECUTIVE_FAULTS;

typedef enum {
  TMP106_CVT_9_BITS    = 0x0,
  TMP106_CVT_10_BITS   = 0x20,
  TMP106_CVT_11_BITS   = 0x40,
  TMP106_CVT_12_BITS   = 0x60,
}TMP106_CONVERTER_RESOLUTION;
 
typedef struct {
  boolean                          ShutDownMode; 
  TMP106_THERMOSTAT_MODE        Thermostat ; 
  TMP106_POLAR_MODE             Polarity; 
  TMP106_CONSECUTIVE_FAULTS     FaultQueue;
  TMP106_CONVERTER_RESOLUTION   CvtReso;
  boolean                          OneShot;
}TMP106_Config;

boolean tmp106_init();
boolean tmp106_reg_write(uint8_t addr, uint8_t * val,uint8_t len);
boolean tmp106_reg_read(uint8_t addr, uint8_t * val,uint8_t len);
boolean tmp106_get_config(TMP106_Config * cfg);
boolean tmp106_set_config(TMP106_Config cfg);
boolean tmp106_read_temp(uint16_t * pval);
boolean tmp106_set_threadhold(uint16_t high,uint16_t low);
boolean tmp106_get_threadhold(uint16_t * high,uint16_t * low);
boolean tmp106_deinit();


#endif 
