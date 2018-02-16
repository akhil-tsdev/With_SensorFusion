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
 * TuringSense, Inc © 2016
 * hub_usb.c
 *
 *  Created on: June 21, 2016
 *      Author: cwati
 *
 *
 */

#include "main.h"
#include "throughput.h"
#include "virtual_com.h"
#include "hub_main_loop_dtm.h"
/*
** MQX initialization information
*/

const TASK_TEMPLATE_STRUCT MQX_template_list[] =
{
   /* Task Index,   	  Function,         Stack,  Priority, Name,       	 	Attributes,          Param,  Time Slice */
   { HUB_MAIN_LOOP_TASK,  hub_dtm_main_loop,    3000,    8,      "HUB_MAIN_LOOP",	MQX_AUTO_START_TASK,  0,      0           },
   { ATHEROS_TASK,        atheros_loop,         3000,    7,      "ATHEROS_LOOP",        MQX_AUTO_START_TASK,  0,      0           },
   //  { USB_CDC_TASK,        Usb_Cdc_Task,     1500, 9,     "USB_CDC_TASK",           MQX_AUTO_START_TASK,  0,      0           },
    {0}
};



 
/* EOF */
