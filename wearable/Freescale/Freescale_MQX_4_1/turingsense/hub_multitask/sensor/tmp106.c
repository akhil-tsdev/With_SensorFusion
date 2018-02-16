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
#ifndef _TMP106_C_
#define _TMP106_C_

#include "tmp106.h"
#include "main.h"

#define TMP_I2C_DEV     "i2c1:"


static MQX_FILE_PTR I2C_DeviceData = NULL;
I2C_TDataState  I2C_DataState;

typedef void *LDD_RTOS_TISRParameter;

LWSEM_STRUCT lock;
#define I2C_EEPROM_MEMORY_WIDTH   1


/*FUNCTION****************************************************************
* 
* Function Name    : i2c_write_polled
* Returned Value   : void
* Comments         : 
*   Writes the provided data buffer at address in I2C EEPROMs
*
*END*********************************************************************/

void i2c_write_polled
   (
      /* [IN] The file pointer for the I2C channel */
      MQX_FILE_PTR fd,

      /* [IN] The address in EEPROM to write to */
      uint_32    addr,

      /* [IN] The array of characters are to be written in EEPROM */
      uchar_ptr  buffer,

      /* [IN] Number of bytes in that buffer */
      _mqx_int   n      
   )
{ /* Body */

   uint_32       param;
   _mqx_int    length;
   _mqx_int    result;
   uint_8        mem[I2C_EEPROM_MEMORY_WIDTH];

   /* Protect I2C transaction in multitask environment */
   _lwsem_wait (&lock);

   do
   {
      /* to set the destination address */
      param = TMP106_SLV_ADDR_0;
      if (I2C_OK != ioctl (I2C_DeviceData, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
      {      
         printf ("set destination ERROR\n");

      }

      length = 1;

      /* Initiate  */
      fwrite (mem, 1, 0, fd);

      /* Check ack (device exists) */
      if (I2C_OK == ioctl (I2C_DeviceData, IO_IOCTL_FLUSH_OUTPUT, &param))
      {
         if (param) 
         {
            /* Stop I2C transfer */
            if (I2C_OK != ioctl (I2C_DeviceData, IO_IOCTL_I2C_STOP, NULL))
            {
               printf ("I2C STOP ERROR\n");
            }
            break;
         }
      } else {
         printf ("Failed to call FLUSH output\n");
      }

      /* Writing address*/
      mem[0] = (uint_8)addr;
      printf ("address  is 0x%02x ... ", mem[0]);
      result = fwrite (mem, 1, 1, I2C_DeviceData);
      if (1 == result)
      {
         printf ("Write address OK\n");
      } else {
         printf ("Write address  address failed \n");
      }

      /* data writing*/
      printf ("  write %d bytes ... ", length);
      result = fwrite (buffer, 1, length, I2C_DeviceData);
      if (result != length)
      {
         printf ("writing data failed \n");
      }
      
      /* Flush the buffer*/
      result = fflush (I2C_DeviceData);
      if (MQX_OK != result)
      {
         printf ("fflush buffer failed\n");
      }

      /* Stop I2C transfer */
      if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_STOP, NULL))
      {
         printf ("stop transmission failed\n");
      }

      /* Wait for EEPROM write cycle finish - acknowledge */
      result = 0;
      do 
      {  /* Sends just I2C bus address, returns acknowledge bit and stops */
         fwrite (&result, 1, 0, I2C_DeviceData);
         
         if (I2C_OK != ioctl (I2C_DeviceData, IO_IOCTL_FLUSH_OUTPUT, &param))
         {
            printf ("  ERROR during wait (flush)\n");
         }
         
         if (I2C_OK != ioctl (I2C_DeviceData, IO_IOCTL_I2C_STOP, NULL))
         {
            printf ("  ERROR during wait (stop)\n");
         }
         result++;
      } while ((param & 1) || (result <= 1));
      
      /* buffer point moving */
      buffer += length;
      
      addr += length;
      n -= length;
        
   } while (n);

   /* Release the transaction lock */
   _lwsem_post (&lock);
} /* Endbody */


/*FUNCTION****************************************************************
* 
* Function Name    : i2c_read_polled
* Returned Value   : void
* Comments         : 
*   Reads into the provided data buffer from address in I2C EEPROM
*
*END*********************************************************************/

void i2c_read_polled
   (
      /* [IN] The file pointer for the I2C channel */
      MQX_FILE_PTR fd,

      /* [IN] The address in EEPROM to read from */
      uint_32    addr,

      /* [IN] The array of characters to be written into */
      uchar_ptr  buffer,

      /* [IN] Number of bytes to read */
      _mqx_int   n      
   )
{ /* Body */
   _mqx_int    param;
   _mqx_int    result;
   uint_8        mem[I2C_EEPROM_MEMORY_WIDTH];

   if (0 == n) 
   {
      printf ("  Wrong data length parameters\n");
      return;
   }

   /* Protect I2C transaction in multitask environment */
   _lwsem_wait (&lock);
   
   /* Setting the destination address */
   param = TMP106_SLV_ADDR_0;
   if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
   {
      printf ("ERROR\n");
   }

   /* Initiate start and send I2C bus address */
   fwrite (mem, 1, 0, fd);

   /* Check ack (device exists) */
   if (I2C_OK == ioctl (fd, IO_IOCTL_FLUSH_OUTPUT, &param))
   {
      printf ("OK ... ack == %d\n", param);
      
      if (param) 
      {
         /* Stop  transfer */
         if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_STOP, NULL))
         {
            printf ("Stop transfer ERROR\n");
         }
         
         /* Release the transaction lock */
         _lwsem_post (&lock);
         
         return;
      }

   } else {
      printf ("check device ERROR\n");
   }

   /* Write address within memory block */

      mem[0] = (uint_8)addr;
      printf ("  Write to address 0x%02x ... ", mem[0]);
      result = fwrite (mem, 1, 1, fd);
      if (1 == result)
      {
         printf ("OK\n");
      } else {
         printf ("ERROR\n");
      }


   /* fflush the memory */
   result = fflush (fd);
   if (MQX_OK != result)
   {
      printf ("fflush Failed\n");
   }

   /* Restart I2C transfer for reading */
   if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_REPEATED_START, NULL))
   {
      printf ("call REPEAT_START failed\n");
   }

   /* Set read request */
   param = n;
   if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_SET_RX_REQUEST, &param))
   {
      printf ("set the RX REQUEST ERROR\n");
   }

   /* Read all data */
   result = fread (buffer, 1, n, fd);
   if (result != n)
   {
      printf ("fread  %d FAILED\n",n);
   }
      
   /* Stop I2C transfer */
   if (I2C_OK != ioctl (fd, IO_IOCTL_I2C_STOP, NULL))
   {
      printf ("Call I2C Stop failed ERROR\n");
   }
   
   /* Release the transaction lock */
   _lwsem_post (&lock);
} /* Endbody */


boolean tmp106_init()
{
  _mqx_int                     param, result;  
  I2C_STATISTICS_STRUCT        stats;
  I2C_DeviceData = fopen(TMP_I2C_DEV,NULL);  
  if(I2C_DeviceData == NULL)
  {
    printf("I2C2_Init failed !\n");
    return FALSE;
  }

     /* Test ioctl commands */
   param = 100000;
   if (I2C_OK != ioctl (I2C_DeviceData, IO_IOCTL_I2C_SET_BAUD, &param))
   {
      printf ("set baud rate %d failed\n",param);
   }

   if (I2C_OK != ioctl (I2C_DeviceData, IO_IOCTL_I2C_GET_BAUD, &param))
   {
      printf ("get baud rate failed\n");
   }

   if (I2C_OK != ioctl (I2C_DeviceData, IO_IOCTL_I2C_SET_MASTER_MODE, NULL))
   {
      printf ("Set master mode failed\n");
   }

   if (I2C_OK == ioctl (I2C_DeviceData, IO_IOCTL_I2C_GET_MODE, &param))
   {
      printf ("current mode 0x%02x\n", param);
   } else {
      printf ("ERROR\n");
   }
   
   param = 0x60;
   if (I2C_OK != ioctl (I2C_DeviceData, IO_IOCTL_I2C_SET_STATION_ADDRESS, &param))
   {
      printf ("set station address failed \n");
   }
   
   param = 0x00;

   if (I2C_OK != ioctl (I2C_DeviceData, IO_IOCTL_I2C_GET_STATION_ADDRESS, &param))
   {
      printf ("Get the station address Failed\n");
   }

   if (I2C_OK != ioctl (I2C_DeviceData, IO_IOCTL_I2C_CLEAR_STATISTICS, NULL))
   {
      printf ("clear STATISTICS Failed\n");
   }
   
   printf ("Get statistics ... ");
   if (I2C_OK == ioctl (I2C_DeviceData, IO_IOCTL_I2C_GET_STATISTICS, (pointer)&stats))
   {
      printf ("OK\n  Interrupts:  %d\n", stats.INTERRUPTS);
      printf ("  Rx packets:  %d\n", stats.RX_PACKETS);
      printf ("  Tx packets:  %d\n", stats.TX_PACKETS);
      printf ("  Tx lost arb: %d\n", stats.TX_LOST_ARBITRATIONS);
      printf ("  Tx as slave: %d\n", stats.TX_ADDRESSED_AS_SLAVE);
      printf ("  Tx naks:     %d\n", stats.TX_NAKS);
   } else {
      printf ("ERROR\n");
   }

   printf ("Get current state ... ");
   if (I2C_OK == ioctl (I2C_DeviceData, IO_IOCTL_I2C_GET_STATE, &param))
   {
      printf ("0x%02x\n", param);
   } else {
      printf ("ERROR\n");
   }
   
   param = 0x49;
   if (I2C_OK == ioctl (I2C_DeviceData, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
   {
      printf ("OK\n");
   } else {
      printf ("ERROR\n");
   }
   
   param = 0x00;
   if (I2C_OK == ioctl (I2C_DeviceData, IO_IOCTL_I2C_GET_DESTINATION_ADDRESS, &param))
   {
      printf ("0x%02x\n", param);
   } else {
      printf ("ERROR\n");
   }


  return TRUE;
}

boolean tmp106_reg_write(uint8_t addr, uint8_t * val,uint8_t len)
{
  ///I2C2_SlaveSendBlock(I2C_DeviceData, LDD_TData *BufferPtr, LDD_I2C_TSize Size);

#if _MQX_LITE_  
  int8 reg_addr;//TMP106_SLV_ADDR_0;
  uint16_t send_num  = 0 ; 
  LDD_I2C_TBusState BusState;  

  I2C2_TDeviceData *DeviceDataPrv = I2C_DeviceData;

  //uint8_t_t SendBuffer[MAX_REG_COUNT];
  uint8_t_t SendBuffer[16];
  
  SendBuffer[0] = addr;
  memcpy(&SendBuffer[1], val, len);
  
  I2C_DataState.Sent = FALSE;
  I2C2_MasterSendBlock(I2C_DeviceData, (LDD_TData *)&SendBuffer, len + 1, LDD_I2C_SEND_STOP);
  while (!I2C_DataState.Sent) {}
  do {I2C2_CheckBus(I2C_DeviceData, &BusState);}
  while(BusState != LDD_I2C_IDLE); 
  if (!I2C_DataState.Sent) {
    return FALSE;
  }  
#else
  if(I2C_DeviceData == NULL)
  {
    printf("please call tmp_init firstly !\n");
    return FALSE;
  }  
  printf("Write ......");  
  i2c_write_polled(I2C_DeviceData,addr,val,len);
  printf("done \n");

  
#endif   
  return TRUE;
}


boolean tmp106_reg_read(uint8_t addr, uint8_t * val,uint8_t len)
{

#if _MQX_LITE_  
  
  int8 i = 0 ; 
  uint16_t myval = 0;
  int8 reg_addr;//TMP106_SLV_ADDR_0;
  uint16_t send_num  = 0 ; 
  LDD_I2C_TBusState BusState;  
  uint8_t reg=0;

  I2C2_TDeviceData *DeviceDataPrv = I2C_DeviceData;
  
  reg_addr = addr;
  I2C_DataState.Sent = FALSE;  
  I2C2_MasterSendBlock(I2C_DeviceData,(LDD_TData *)&addr,(LDD_I2C_TSize)sizeof(uint8_t),LDD_I2C_SEND_STOP);
  while (!I2C_DataState.Sent) {}
  if (!I2C_DataState.Sent) {
    return FALSE;
  }  
  ///app_time_delay(100);
  printf("Start to read the reg [%02x] \n",addr);

  I2C_DataState.Received = FALSE;
  I2C2_MasterReceiveBlock(I2C_DeviceData, (LDD_TData *)val, (LDD_I2C_TSize)len,LDD_I2C_SEND_STOP);  
  while (!I2C_DataState.Received) {}
  do {I2C2_CheckBus(I2C_DeviceData, &BusState);}  while (BusState != LDD_I2C_IDLE);
  if (!I2C_DataState.Received) {
    return FALSE;
  }  
  
  printf("\n Myvalue is %02x \n",*val);
  
#else
  if(I2C_DeviceData == NULL)
  {
    printf("please call tmp_init firstly !\n");
    return FALSE;
  }
  
  i2c_read_polled(I2C_DeviceData,addr,val,len);
  printf("the temp is :%08x \n",*val);
#endif  
  return TRUE;
  
}

boolean tmp106_get_config(TMP106_Config * cfg)
{
  uint8_t reg = REG_TMP106_CFG_REG;
  uint8_t val = 0;
  
  tmp106_reg_read(reg,&val,sizeof(val));

  cfg->ShutDownMode = (val&TMP106_CFG_SD_MASK);
  cfg->Thermostat = (val&TMP106_CFG_TM_MASK);
  cfg->Polarity = (val&TMP106_CFG_POL_MASK);
  cfg->FaultQueue = (val&TMP106_CFG_FQ_MASK);
  cfg->CvtReso = (val&TMP106_CFG_CVT_MASK);
  cfg->OneShot = (val&TMP106_CFG_OS_MASK);
  
  return TRUE;
}

boolean tmp106_set_config(TMP106_Config cfg)
{
  uint8_t val = 0 ; 
  uint8_t reg = REG_TMP106_CFG_REG;

  val = cfg.ShutDownMode|cfg.Thermostat|cfg.Polarity|cfg.FaultQueue|cfg.CvtReso|cfg.OneShot;
  if(!tmp106_reg_write(reg,&val,TMP106_CFG_REG_LEN))
  {
    printf("Call tmp106_set_config failed!\n ");
    return FALSE;
  }
  return TRUE;
}

boolean tmp106_read_temp(uint16_t * pval)
{
  uint8_t tmp_reg = REG_TMP106_TMP_REG ;
  uint8_t shift_val[2] = 0;
  if (!tmp106_reg_read(tmp_reg, (uint8_t*)pval,TMP106_TMP_REG_LEN))
  {
    printf("\nRead tmp error !\n");
    return FALSE;
  }
  shift_val[0] = (((uint8_t*)pval)[1]>>4)|(((uint8_t*)pval)[0]<<4);
  shift_val[1] = ((uint8_t*)pval)[0]>>4;
  *pval = *(uint16_t*)shift_val;
  return TRUE;
}

boolean tmp106_set_threadhold(uint16_t high,uint16_t low)
{
  uint8_t tmp_reg = REG_TMP106_HIGH_REG ;
  if (!tmp106_reg_write(tmp_reg, (uint8_t*)&high,TMP106_HIGH_REG_LEN))
  {
    printf("\nRead tmp error !\n");
    return FALSE;
  }
  
  tmp_reg = REG_TMP106_LOW_REG ;
  if (!tmp106_reg_write(tmp_reg, (uint8_t*)&low,TMP106_LOW_REG_LEN))
  {
    printf("\nRead tmp error !\n");
    return FALSE;
  }  
  return TRUE;
}

boolean tmp106_get_threadhold(uint16_t * high,uint16_t * low)
{
  return TRUE;
}

boolean tmp106_deinit()
{
  TMP106_Config            cfg;
  uint8_t                  tmp_cfg_value = 0;  
  
  if(I2C_DeviceData)
  {
    printf("Try to close the TMP106 sensor \n");

    memset(&cfg,0,sizeof(cfg));
    tmp_cfg_value = 41;
    cfg.ShutDownMode  = (tmp_cfg_value&TMP106_CFG_SD_MASK);
    cfg.Thermostat  = (tmp_cfg_value&TMP106_CFG_TM_MASK);          
    cfg.Polarity  = (tmp_cfg_value&TMP106_CFG_POL_MASK);          
    cfg.FaultQueue  = (tmp_cfg_value&TMP106_CFG_CVT_MASK);          
    cfg.OneShot  = (tmp_cfg_value&TMP106_CFG_OS_MASK); 
    if (!tmp106_set_config(cfg))
    {
        printf("read tmp106 device  failed ! \n");
        return FALSE;
    }
    fclose(I2C_DeviceData);
    I2C_DeviceData = NULL; 
  }else
  {
    printf("tmp106 NOT opened, deinit failed! \n");
  }
  
  ///_io_i2c_polled_uninstall();
  return TRUE;
}

void my_test()
{
  uint8_t  reg ;
  uint16_t tmp_value = 0;
  uint8_t  cfg_value = TMP106_FAULT_QUEUE_2|TMP106_CVT_10_BITS;
  
  tmp106_reg_write(REG_TMP106_CFG_REG,&cfg_value,TMP106_CFG_REG_LEN);

  tmp106_get_config(NULL);
  
  reg = REG_TMP106_TMP_REG;  
  tmp106_read_temp(&tmp_value);
  printf("\nthe tmp value is :%04x \n",tmp_value);

}

#endif 