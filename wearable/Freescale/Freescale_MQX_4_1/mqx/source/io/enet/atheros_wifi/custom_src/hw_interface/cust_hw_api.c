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
#include <a_config.h>
#include <a_types.h>
#include <a_osapi.h>
#include <custom_api.h>
#include <mqx.h>
#include <bsp.h>
#include <common_api.h>
#include <enet.h> //TONY
#include <atheros_wifi.h>


#define POWER_UP_DELAY (1)

#define HW_SPI_CAPS (HW_SPI_FRAME_WIDTH_8| \
					 HW_SPI_NO_DMA | \
					 HW_SPI_INT_EDGE_DETECT)       



A_VOID 
Custom_HW_SetClock(A_VOID *pCxt, A_UINT32 *pClockRate)
{
    UNUSED_ARGUMENT(pCxt);
	UNUSED_ARGUMENT(pClockRate);
}

A_VOID 
Custom_HW_EnableDisableSPIIRQ(A_VOID *pCxt, A_BOOL enable)
{
	if(enable == A_TRUE){
		ioctl(GET_DRIVER_CXT(pCxt)->int_cxt, GPIO_IOCTL_ENABLE_IRQ, NULL);  
	}else{
		ioctl(GET_DRIVER_CXT(pCxt)->int_cxt, GPIO_IOCTL_DISABLE_IRQ, NULL);  
	}
}

A_VOID Custom_HW_UsecDelay(A_VOID *pCxt, A_UINT32 uSeconds)
{
    //avoids division and modulo.
    A_INT32 time = uSeconds>>10; // divide by 1024
    // account for difference between 1024 and 1000 with 
    // acceptable margin for error
    time += (time * 24 > 500)? 1:0;
    // capture round off between usec and msec
    time += (uSeconds - (time * 1000) > 500)? 1:0;
    //input must be sub 500 so we simply wait 1msec
    if(time == 0) time += 1;
    //current implementation relys on msec sleep
    _time_delay(time);
    UNUSED_ARGUMENT(pCxt);
}

A_VOID 
Custom_HW_PowerUpDown(A_VOID *pCxt, A_UINT32 powerUp)
{   
    A_UINT32 cmd = (powerUp)?  GPIO_IOCTL_WRITE_LOG1: GPIO_IOCTL_WRITE_LOG0;
    SIM_MemMapPtr   sim = SIM_BASE_PTR;
    PORT_MemMapPtr  pctl;
    MQX_FILE_PTR                f=NULL;    
    if (IO_OK != ioctl (GET_DRIVER_CXT(pCxt)->pwd_cxt, cmd, NULL))
	{
		A_ASSERT(0);
	}

#if 0
    {
      cmd = (powerUp)?  GPIO_PIN_STATUS_0: GPIO_PIN_STATUS_1;
      const uint_32 output_set[] = {
          BSP_ATHEROS_WIFI_GPIO_FET_PIN | cmd,
          GPIO_LIST_END
      };

      /* Open and set port TC as output to drive LEDs (LED10 - LED13) */
      f = fopen("gpio:write", (char_ptr) &output_set);
      cmd = (powerUp)?  GPIO_IOCTL_WRITE_LOG0: GPIO_IOCTL_WRITE_LOG1;

     if (f)
          ioctl(f, cmd, NULL);              
    
      fclose(f);      
      f=NULL;
      
      cmd = (powerUp)?  GPIO_PIN_STATUS_0: GPIO_PIN_STATUS_1;
      const uint_32 output1_set[] = {
          BSP_LED1 | cmd,
          GPIO_LIST_END
      };  
      
      f = fopen("gpio:write", (char_ptr) &output1_set);
      cmd = (powerUp)?  GPIO_IOCTL_WRITE_LOG0: GPIO_IOCTL_WRITE_LOG1;

       if (f)
            ioctl(f, cmd, NULL);              

      fclose(f);      
      f=NULL;
    }
#endif	

#if 1
    {
      if(!powerUp)
      {
#if 0
          //I2C
          GPIOE_PDOR |= 0x3;
          //A1 input
          GPIO_PDDR_REG(PTA_BASE_PTR) &= 0xfffffffd;
          //B18 output, high
          GPIO_PDDR_REG(PTB_BASE_PTR) |= 0x40000;
          GPIOB_PDOR |= 0x40000;
          //B2,B17
          PORT_PCR_REG(PORTB_BASE_PTR, 2) = PORT_PCR_MUX(0x1) | PORT_PCR_PE_MASK;
          PORT_PCR_REG(PORTB_BASE_PTR, 17) = PORT_PCR_MUX(0x1) | PORT_PCR_PE_MASK;
          GPIOB_PDDR |= 0x20004;
          GPIOB_PDOR &= 0xFFFDFFFB;
          
          //B3,B16
          PORT_PCR_REG(PORTB_BASE_PTR, 3) = PORT_PCR_MUX(0x5) | PORT_PCR_PE_MASK;
          PORT_PCR_REG(PORTB_BASE_PTR, 16) = PORT_PCR_MUX(0x5) | PORT_PCR_PE_MASK;
          //GPIOB_PDDR &= 0xFFFEFFF7;
          //if((PORT_PCR_REG(PORTD_BASE_PTR, 4) && PORT_PCR_PS_MASK ))
          ///  PORT_PCR_REG(PORTD_BASE_PTR, 4) &= ~(PORT_PCR_PS_MASK);
          ///pctl->PCR[4] = PORT_PCR_MUX(1);// | PORT_PCR_PE_MASK ;//| PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA) ;
#endif           
#if PRODUCTION1
            /* Setting Port Data Direction. PTD11 (Chip select) is set as input pin*/
            GPIO_PDDR_REG(PTD_BASE_PTR) |= ~(0xfffff7ff);
            /* Setting Port Data Direction. */
            GPIO_PDOR_REG(PTD_BASE_PTR) &= 0xfffff7ff;
            /* Configure GPIOD for DSPI2 (PRODUCTION1 board) peripheral function     */
            pctl = (PORT_MemMapPtr)PORTD_BASE_PTR;
//            pctl->PCR[13] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;     /* DSPI2.SOUT   */
//            pctl->PCR[12] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;     /* DSPI2.SCK    */
//            pctl->PCR[14] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;     /* DSPI2.SIN    */
//            pctl->PCR[11] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;     /* DSPI2.PCS0   */
            pctl->PCR[13] = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;     /* DSPI2.SOUT   */
            pctl->PCR[12] = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;     /* DSPI2.SCK    */
            pctl->PCR[14] = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;     /* DSPI2.SIN    */
            pctl->PCR[11] = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;     /* DSPI2.PCS0   */

            /* Enable clock gate to DSPI2 module */
            sim->SCGC3 &= ~(SIM_SCGC3_SPI2_MASK);

#elif EVAL_BOARD
            /* Setting Port Data Direction.  0 = input pin, 1 = output pin 
               PTD4 (Chip select) is set as input pin*/
            GPIO_PDDR_REG(PTD_BASE_PTR) |= ~(0xffffffef);
            /* Setting Port Data Output Direction.  Logic level 0 is driven on pin if pin is OUTPUT*/
            GPIO_PDOR_REG(PTD_BASE_PTR) &= 0xffffffef;
            //PORT_PCR_REG(PORTD_BASE_PTR, 4) = PORT_PCR_MUX(0) | PORT_PCR_PE_MASK;
            /* Configure GPIOB for DSPI1 peripheral function     */
            pctl = (PORT_MemMapPtr)PORTD_BASE_PTR;

            /* Turingsense cwati seems like this MUX(6) can be replaced with MUX(1).
             * As long as it's not MUX(2). */
            pctl->PCR[6] = PORT_PCR_MUX(6) | PORT_PCR_DSE_MASK;     /* DSPI1.SOUT   */
            pctl->PCR[5] = PORT_PCR_MUX(6) | PORT_PCR_DSE_MASK;     /* DSPI1.SCK    */
            pctl->PCR[7] = PORT_PCR_MUX(6) | PORT_PCR_DSE_MASK;     /* DSPI1.SIN    */
            pctl->PCR[4] = PORT_PCR_MUX(6) | PORT_PCR_DSE_MASK;     /* DSPI1.PCS0   */

            /* Enable clock gate to DSPI1 module */
            sim->SCGC6 &= ~(SIM_SCGC6_SPI1_MASK);
#else
#error "You must define Board Type!!"
#endif
      }else
      {
#if 0        
          //I2C
          GPIOE_PDOR &= 0xffffffc;
          //A1 input
          GPIO_PDDR_REG(PTA_BASE_PTR) |= 0x2;
          //B18 output, high
          GPIO_PDDR_REG(PTB_BASE_PTR) &= 0xfffbffff;
          GPIOB_PDOR &= 0xfffbffff;
          //B2,B17  UART0
          PORT_PCR_REG(PORTB_BASE_PTR, 2) = PORT_PCR_MUX(0x3) | PORT_PCR_PE_MASK;
          PORT_PCR_REG(PORTB_BASE_PTR, 17) = PORT_PCR_MUX(0x3) | PORT_PCR_PE_MASK;
          GPIOB_PDDR &= ~(0x20004);
          GPIOB_PDOR |= ~(0xFFFDFFFB);
          
          //B3,B16 UART0
          PORT_PCR_REG(PORTB_BASE_PTR, 3) = PORT_PCR_MUX(0x3) | PORT_PCR_PE_MASK;
          PORT_PCR_REG(PORTB_BASE_PTR, 16) = PORT_PCR_MUX(0x3) | PORT_PCR_PE_MASK;
#endif          
          //GPIOB_PDDR &= 0xFFFEFFF7;
#if PRODUCTION1
          pctl = (PORT_MemMapPtr)PORTD_BASE_PTR;
          /* IRQC = 0xA means Interrupt is on falling edge */
          pctl->PCR[11] = PORT_PCR_MUX(0x1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA) ;
          /* Setting PTD11 as input pin */
          GPIO_PDDR_REG(PTD_BASE_PTR) &= 0xfffff7ff;
            /* Configure GPIOB for DSPI1 peripheral function     */
            pctl = (PORT_MemMapPtr)PORTD_BASE_PTR;

            pctl->PCR[13] = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;     /* DSPI2.SOUT   */
            pctl->PCR[12] = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;     /* DSPI2.SCK    */
            pctl->PCR[14] = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;     /* DSPI2.SIN    */
            pctl->PCR[11] = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;     /* DSPI2.PCS0   */

            /* Enable clock gate to DSPI2 module */
            sim->SCGC3 |= SIM_SCGC3_SPI2_MASK;

#elif EVAL_BOARD
          pctl = (PORT_MemMapPtr)PORTD_BASE_PTR;
          pctl->PCR[4] = PORT_PCR_MUX(0x1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA) ;
          GPIO_PDDR_REG(PTD_BASE_PTR) &= 0xffffffef;
          //GPIO_PDOR_REG(PTD_BASE_PTR) |= ~(0xffffffef);
          ///PORT_PCR_REG(PORTD_BASE_PTR, 4) = PORT_PCR_MUX(0x1) | PORT_PCR_PE_MASK;
            /* Configure GPIOB for DSPI1 peripheral function     */
            pctl = (PORT_MemMapPtr)PORTD_BASE_PTR;

            pctl->PCR[6] = PORT_PCR_MUX(7) | PORT_PCR_DSE_MASK;     /* DSPI1.SOUT   */
            pctl->PCR[5] = PORT_PCR_MUX(7) | PORT_PCR_DSE_MASK;     /* DSPI1.SCK    */
            pctl->PCR[7] = PORT_PCR_MUX(7) | PORT_PCR_DSE_MASK;     /* DSPI1.SIN    */
            pctl->PCR[4] = PORT_PCR_MUX(7) | PORT_PCR_DSE_MASK;     /* DSPI1.PCS0   */

            /* Enable clock gate to DSPI1 module */
            sim->SCGC6 |= SIM_SCGC6_SPI1_MASK;
#else
#error "You must define board type!"
#endif
      }
    }
#endif     
}

/*****************************************************************************/
/* Custom_Bus_InOutBuffer - This is the platform specific solution to 
 *  transfer a buffer on the SPI bus.  This solution is always synchronous
 *  regardless of sync param. The function will use the MQX fread and fwrite
 *  as appropriate.
 *      A_VOID * pCxt - the driver context.
 *      A_UINT8 *pBuffer - The buffer to transfer.
 *      A_UINT16 length - the length of the transfer in bytes.
 *      A_UINT8 doRead - 1 if operation is a read else 0.
 *      A_BOOL sync - TRUE is synchronous transfer is required else FALSE.
 *****************************************************************************/
A_STATUS 
Custom_Bus_InOutBuffer(A_VOID *pCxt,
                                    A_UINT8 *pBuffer,
									A_UINT16 length, 									
									A_UINT8 doRead,
                                    A_BOOL sync)
{
	A_STATUS status = A_OK;    
    
    UNUSED_ARGUMENT(sync);
    /* this function takes advantage of the SPI turbo mode which does not toggle the chip select
     * during the transfer.  Rather the chip select is asserted at the beginning of the transfer
     * and de-asserted at the end of the entire transfer via fflush(). */     
#if TURINGSENSE
    _task_stop_preemption();
#endif
    if(doRead){	    
	    if(length != fread(pBuffer, 1, length, GET_DRIVER_CXT(pCxt)->spi_cxt)){
		    status = A_HARDWARE;    	    
	    }	    	
	}else{		
		if(length != fwrite(pBuffer, 1, (A_INT32)length, GET_DRIVER_CXT(pCxt)->spi_cxt)){
			status = A_HARDWARE;        	
		}
	} 
#if TURINGSENSE
	_task_start_preemption();
#endif
    return status;
}				

/*****************************************************************************/
/* Custom_Bus_InOut_Token - This is the platform specific solution to 
 *  transfer 4 or less bytes in both directions. The transfer must be  
 *  synchronous. This solution uses the MQX spi ioctl to complete the request.
 *      A_VOID * pCxt - the driver context.
 *      A_UINT32 OutToken - the out going data.
 *      A_UINT8 DataSize - the length in bytes of the transfer.
 *      A_UINT32 *pInToken - A Buffer to hold the incoming bytes. 
 *****************************************************************************/
A_STATUS 
Custom_Bus_InOutToken(A_VOID *pCxt,
                           A_UINT32        OutToken,
                           A_UINT8         DataSize,
                           A_UINT32    *pInToken) 
{   
    SPI_READ_WRITE_STRUCT  spi_buf;        
    A_STATUS status = A_OK;    
    
    spi_buf.BUFFER_LENGTH = (A_UINT32)DataSize; // data size if really a enum that is 1 less than number of bytes
     
    _DCACHE_FLUSH_MBYTES((pointer)&OutToken, sizeof(A_UINT32));
    
    spi_buf.READ_BUFFER = (char_ptr)pInToken;
    spi_buf.WRITE_BUFFER = (char_ptr)&OutToken;

#if TURINGSENSE
        _task_stop_preemption();
#endif
    do{        
        if (A_OK != Custom_Bus_StartTransfer(pCxt, A_TRUE))
        {        
            status = A_HARDWARE;
            break;
        }
        //IO_IOCTL_SPI_READ_WRITE requires read && write buffers      

        if (SPI_OK != ioctl(GET_DRIVER_CXT(pCxt)->spi_cxt, IO_IOCTL_SPI_READ_WRITE, &spi_buf)) 
        {        
            status = A_HARDWARE;
            break;
        }
        //waits until interrupt based operation completes            
        Custom_Bus_CompleteTransfer(pCxt, A_TRUE);
        _DCACHE_INVALIDATE_MBYTES((pointer)pInToken, sizeof(A_UINT32));
    }while(0);
#if TURINGSENSE
    _task_start_preemption();
#endif
    return status;                                                                        
}


/*****************************************************************************/
/* Custom_Bus_Start_Transfer - This function is called by common layer prior
 *  to starting a new bus transfer. This solution merely sets up the SPI 
 *  mode as a precaution.
 *      A_VOID * pCxt - the driver context.     
 *      A_BOOL sync - TRUE is synchronous transfer is required else FALSE.
 *****************************************************************************/
A_STATUS 
Custom_Bus_StartTransfer(A_VOID *pCxt, A_BOOL sync)
{
    A_UINT32 param;
    A_STATUS status = A_OK;

    UNUSED_ARGUMENT(sync);
    //ensure SPI device is set to proper mode before issuing transfer
    param = SPI_CLK_POL_PHA_MODE3;
    /* set the mode in case the SPI is shared with another device. */
    if (SPI_OK != ioctl(GET_DRIVER_CXT(pCxt)->spi_cxt, IO_IOCTL_SPI_SET_MODE, &param)) 
    {        
        status = A_ERROR;        
    }

    return status;
}


/*****************************************************************************/
/* Custom_Bus_Complete_Transfer - This function is called by common layer prior
 *  to completing a bus transfer. This solution calls fflush to de-assert 
 *  the chipselect.
 *      A_VOID * pCxt - the driver context.     
 *      A_BOOL sync - TRUE is synchronous transfer is required else FALSE.
 *****************************************************************************/
A_STATUS 
Custom_Bus_CompleteTransfer(A_VOID *pCxt, A_BOOL sync)
{
    UNUSED_ARGUMENT(sync);
    fflush(GET_DRIVER_CXT(pCxt)->spi_cxt);
    return A_OK;
}

extern A_VOID *p_Global_Cxt;


/* interrupt service routine that gets mapped to the 
 * SPI INT GPIO */
A_VOID 
Custom_HW_InterruptHandler(A_VOID *pointer)
{
    /* pass global context to common layer */
    HW_InterruptHandler(p_Global_Cxt);
    UNUSED_ARGUMENT(pointer);
}


A_STATUS 
Custom_HW_Init(A_VOID *pCxt)
{
    GPIO_PIN_STRUCT 	pins[2];
    A_CUSTOM_DRIVER_CONTEXT *pCustCxt = GET_DRIVER_CXT(pCxt);
    A_UINT32 baudrate, endian, input;
    ATHEROS_PARAM_WIFI_STRUCT *pParam;
    A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(pCxt);

#ifdef ATH_SPI_DMA
    {
        extern int _bsp_dspi_dma_setup(void);
        static int bsp_dspi_dma_ready = 0;
        if (!bsp_dspi_dma_ready)
        {
            _bsp_dspi_dma_setup();
            bsp_dspi_dma_ready = 1;
        }
    }
#endif
    pParam = GET_DRIVER_CXT(pCxt)->pDriverParam;

    pCustCxt->spi_cxt = fopen(pParam->SPI_DEVICE, (pointer)SPI_FLAG_FULL_DUPLEX);
        
    if(pCustCxt->spi_cxt == NULL){
        A_ASSERT(0);
    }

    ioctl(pCustCxt->spi_cxt, IO_IOCTL_SPI_GET_BAUD, &baudrate);

#if TURINGSENSE
#define MAX_ALLOWED_BAUD_RATE 1000000
#else
#define MAX_ALLOWED_BAUD_RATE 25000000
#endif
	if(baudrate > MAX_ALLOWED_BAUD_RATE){
		baudrate = MAX_ALLOWED_BAUD_RATE;
		ioctl(pCustCxt->spi_cxt, IO_IOCTL_SPI_SET_BAUD, &baudrate);
	}
	
    ioctl(pCustCxt->spi_cxt, IO_IOCTL_SPI_GET_ENDIAN, &endian);
#if PSP_MQX_CPU_IS_KINETIS
	input = 1; //SPI_PUSHR_PCS(1 << 0); 
#elif PSP_MQX_CPU_IS_MCF52
    input = MCF5XXX_QSPI_QDR_QSPI_CS0; 
#else
#error Atheros wifi: unsupported Freescale chip     
#endif    
    if (SPI_OK != ioctl (pCustCxt->spi_cxt, IO_IOCTL_SPI_SET_CS, &input)) 
    {        
        A_ASSERT(0);
    }
    //SPI_DEVICE_LITTLE_ENDIAN or SPI_DEVICE_BIG_ENDIAN
    //IO_IOCTL_SPI_SET_CS
    /* fill the pins structure */
    pins[0] = pParam->INT_PIN;
    pins[1] = GPIO_LIST_END;        

    pCustCxt->int_cxt = fopen(pParam->GPIO_DEVICE, (char_ptr)&pins);

    if(NULL == pCustCxt->int_cxt){            
        A_ASSERT(0);
    }
        
    if (IO_OK != ioctl(pCustCxt->int_cxt, GPIO_IOCTL_SET_IRQ_FUNCTION, (pointer)&Custom_HW_InterruptHandler )){        
        A_ASSERT(0);
    }

#if WLAN_CONFIG_ENABLE_CHIP_PWD_GPIO
/* use gpio pin to reset the wifi chip as needed.
 * this service is required if during operation it
 * is desired to turn off the wifi chip. 
 */
	pins[0] = pParam->PWD_PIN;
    pins[1] = GPIO_LIST_END;     
	pCustCxt->pwd_cxt = fopen(pParam->PWD_DEVICE, (char_ptr)&pins);
	
	if(NULL == pCustCxt->pwd_cxt){            
          A_ASSERT(0);
        }

#if PRODUCTION1
#if EXPERIMENT
	/* In eval board, fopen will pull PWD low.  Then it will be powered on
	 * in about 10ms.  This indicates the chip to come alive.
	 * In production board, fopen will pull PWD high, so we have to manually pull it low.
	 */
    if (IO_OK != ioctl (pCustCxt->pwd_cxt, GPIO_IOCTL_WRITE_LOG0, NULL))
	{
		A_ASSERT(0);
	}

	CUSTOM_HW_USEC_DELAY(pCxt, 2000);

#endif /* EXPERIMENT */
#endif /* PRODUCTION1 */

#endif /* WLAN_CONFIG_ENABLE_CHIP_PWD_GPIO */
        
    /* IT is necessary for the custom code to populate the following parameters
     * so that the common code knows what kind of hardware it is working with. */        
    pDCxt->hcd.OperationalClock = baudrate;
    pDCxt->hcd.PowerUpDelay = POWER_UP_DELAY;            
    pDCxt->hcd.SpiHWCapabilitiesFlags = HW_SPI_CAPS;
    pDCxt->hcd.MiscFlags |= MISC_FLAG_RESET_SPI_IF_SHUTDOWN;   
  
    return A_OK;
}

A_STATUS 
Custom_HW_DeInit(A_VOID *pCxt)
{
#if WLAN_CONFIG_ENABLE_CHIP_PWD_GPIO
	/* NOTE: if the gpio fails to retain its state after fclose
	 * then the wifi device will not remain in the off State.
	 */	
    fclose(GET_DRIVER_CXT(pCxt)->pwd_cxt);
    GET_DRIVER_CXT(pCxt)->pwd_cxt = NULL;
#endif       
	/* clean up resources initialized in Probe() */
    fclose(GET_DRIVER_CXT(pCxt)->spi_cxt);
    GET_DRIVER_CXT(pCxt)->spi_cxt = NULL;
    fclose(GET_DRIVER_CXT(pCxt)->int_cxt);
    GET_DRIVER_CXT(pCxt)->int_cxt = NULL;
    
#if BSPCFG_ENABLE_ADC_SENSOR
    GET_DRIVER_CXT(pCxt)->adc_cxt = NULL;
#endif     
    
    return A_OK;
}
