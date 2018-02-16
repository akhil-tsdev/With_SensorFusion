/*HEADER**********************************************************************
*
* Copyright 2008 Freescale Semiconductor, Inc.
* Copyright 1989-2008 ARC International
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale MQX RTOS License
* distributed with this Material.
* See the MQX_RTOS_LICENSE file distributed for more details.
*
* Brief License Summary:
* This software is provided in source form for you to use free of charge,
* but it is not open source software. You are allowed to use this software
* but you cannot redistribute it or derivative works of it in source form.
* The software may be used only in connection with a product containing
* a Freescale microprocessor, microcontroller, or digital signal processor.
* See license agreement file for full license terms including other
* restrictions.
*****************************************************************************
*
* Comments:
* 
* @brief  The file emulates a USB PORT as RS232 PORT.
* 
*END************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "print_scan.h"
#include "virtual_com.h"

extern void Usb_Cdc_Task(uint32_t param);
//TASK_TEMPLATE_STRUCT  MQX_template_list[] =
//{
//   { MAIN_TASK, Main_Task, 2*3000L, 7L, "Main", MQX_AUTO_START_TASK, 0, 0},
//   { 0L, 0L, 0L, 0L, 0L, 0L, 0, 0}
//};

/*****************************************************************************
 * Constant and Macro's - None
 *****************************************************************************/
 
/*****************************************************************************
 * Global Functions Prototypes
 *****************************************************************************/
void TestApp_Init(void);

/****************************************************************************
 * Global Variables
 ****************************************************************************/              
extern USB_ENDPOINTS usb_desc_ep;
extern DESC_CALLBACK_FUNCTIONS_STRUCT  desc_callback;

CDC_HANDLE   g_app_handle;

/*****************************************************************************
 * Local Types - None
 *****************************************************************************/
 
/*****************************************************************************
 * Local Functions Prototypes
 *****************************************************************************/
void USB_App_Callback(uint8_t event_type, void* val,void *arg);
void USB_Notif_Callback(uint8_t event_type,void* val,void *arg); 
void Virtual_Com_App(void);
/*****************************************************************************
 * Local Variables 
 *****************************************************************************/
static bool start_app = FALSE;
static bool start_transactions = FALSE;
static uint8_t *g_curr_recv_buf;
static uint8_t *g_curr_send_buf;
static uint8_t g_recv_size;
static uint8_t g_send_size;
uint8_t buf_tmp[DIC_BULK_OUT_ENDP_PACKET_SIZE];
uint8_t current_index = 0;
static volatile uint8_t send_complete = 0;

/*****************************************************************************
 * Local Functions
 *****************************************************************************/

/*************Code for debug_printf/scanf*******************************/
int debug_printf(const char  *fmt_s, ...)
{
   va_list  ap;
   int  result;

   va_start(ap, fmt_s);
   result = _doprint(NULL, debug_putc, -1, (char *)fmt_s, ap);
   va_end(ap);

   return result;
}

int debug_putc(int ch, void* stream)
{
    const unsigned char c = (unsigned char) ch;
    VirtualCom_SendDataBlocking(g_app_handle, &c, 1);
    return 0;
}

int debug_scanf(const char  *fmt_ptr, ...)
{
    char    temp_buf[IO_MAXLINE];
    va_list ap;
    uint32_t i;
    char result;

    va_start(ap, fmt_ptr);
    temp_buf[0] = '\0';

    for (i = 0; i < IO_MAXLINE; i++)
    {
        temp_buf[i] = result = debug_getchar();

        if ((result == '\r') || (result == '\n'))
        {
            /* End of Line */
            if (i == 0)
            {
                i = (uint32_t)-1;
            }
            else
            {
                break;
            }
        }

        temp_buf[i + 1] = '\0';
    }

    result = scan_prv(temp_buf, (char *)fmt_ptr, ap);
    va_end(ap);

   return result;
}

int debug_getchar(void)
{
    unsigned char c;
    VirtualCom_ReceiveDataBlocking(g_app_handle, &c, 1);
    return c;
}
/******************************************************************************
 *
 *    @name        VirtualCom_SendDataBlocking
 *
 *    @brief       This function sends data to host
 *
 *    @param       baseAddr  :  pointer to USB callback
 *    @param       buf       :  pointer to the data
 *    @param       count     :  size of the transfer
 *
 *
 *****************************************************************************/
void VirtualCom_SendDataBlocking(uint32_t baseAddr, const uint8_t *buf, uint32_t count)
{
    while (!USB_Check_Start_Transactions());
    USB_Class_CDC_Send_Data(g_app_handle, DIC_BULK_IN_ENDPOINT, (uint8_t *)buf, count);
    while (!send_complete);
    send_complete = 0;
}

/******************************************************************************
 *
 *    @name        VirtualCom_ReceiveDataBlocking
 *
 *    @brief       This function receives data from host
 *
 *    @param       baseAddr  :  pointer to USB callback
 *    @param       buf       :  pointer to the data
 *    @param       count     :  size of the transfer
 *
 *    @return      status
 *                   USB_OK  :  if successful
 *                   else return error
 *
 *****************************************************************************/
uint32_t VirtualCom_ReceiveDataBlocking(uint32_t baseAddr, uint8_t *buf, uint32_t count)
{
    uint32_t error = USB_OK;
    if (g_recv_size <= 0)
    {
        error = USB_Class_CDC_Recv_Data(g_app_handle , DIC_BULK_OUT_ENDPOINT, buf_tmp , DIC_BULK_OUT_ENDP_PACKET_SIZE);
        current_index = 0;
        if(error != USB_OK)
        {
             return error;
        }
    }
    while (g_recv_size <= 0);
    if  (count > g_recv_size ) count = g_recv_size;
    memcpy(buf, buf_tmp + current_index, count);
    g_recv_size -= count;
    current_index += count;
    return error;
}

/******************************************************************************
 *
 *    @name        USB_Check_Start_Transactions
 *
 *    @brief       This function checks whether USB start application and transactions
 *
 *    @return      status
 *                   1  :  if successful
 *                   else return 0
 *
 *
 *****************************************************************************/
uint8_t USB_Check_Start_Transactions()
{
  if ((start_app == TRUE) && (start_transactions == TRUE)) return 1;

  else return 0;
}
 /*****************************************************************************
 *  
 *   @name        TestApp_Init
 * 
 *   @brief       This function is the entry for mouse (or other usuage)
 * 
 *   @param       None
 * 
 *   @return      None
 **                
 *****************************************************************************/
void TestApp_Init(void)
{       
    //uint8_t   error;
    CDC_CONFIG_STRUCT cdc_config;
    USB_CLASS_CDC_ENDPOINT * endPoint_ptr;
    
    g_curr_recv_buf = _mem_alloc_uncached(DATA_BUFF_SIZE);
    g_curr_send_buf = _mem_alloc_uncached(DATA_BUFF_SIZE);
    
    endPoint_ptr = USB_mem_alloc_zero(sizeof(USB_CLASS_CDC_ENDPOINT)*CDC_DESC_ENDPOINT_COUNT);
    cdc_config.comm_feature_data_size = COMM_FEATURE_DATA_SIZE;
    cdc_config.usb_ep_data = &usb_desc_ep;
    cdc_config.desc_endpoint_cnt = CDC_DESC_ENDPOINT_COUNT;
    cdc_config.cdc_class_cb.callback = USB_App_Callback;
    cdc_config.cdc_class_cb.arg = &g_app_handle;
    cdc_config.vendor_req_callback.callback = NULL;
    cdc_config.vendor_req_callback.arg = NULL;
    cdc_config.param_callback.callback = USB_Notif_Callback;
    cdc_config.param_callback.arg = &g_app_handle;
    cdc_config.desc_callback_ptr =  &desc_callback;
    cdc_config.ep = endPoint_ptr;
    cdc_config.cic_send_endpoint = CIC_NOTIF_ENDPOINT;
    /* Always happend in control endpoint hence hard coded in Class layer*/
    //cdc_config.cic_recv_endpoint = 
    cdc_config.dic_send_endpoint = DIC_BULK_IN_ENDPOINT;
    cdc_config.dic_recv_endpoint = DIC_BULK_OUT_ENDPOINT;
    
    if (MQX_OK != _usb_device_driver_install(USBCFG_DEFAULT_DEVICE_CONTROLLER)) {
        printf("Driver could not be installed\n");
        return;
    }

    /* Initialize the USB interface */
    g_app_handle = USB_Class_CDC_Init(&cdc_config);
    g_recv_size = 0;
    g_send_size= 0;     
} 

/******************************************************************************
 * 
 *    @name       Virtual_Com_App
 *    
 *    @brief      
 *                  
 *    @param      None
 * 
 *    @return     None
 *    
 *****************************************************************************/
void Virtual_Com_App(void)
{
    /* User Code */ 
    if(g_recv_size) 
    {
        _mqx_int i;
        
        /* Copy Buffer to Send Buff */
        for (i = 0; i < g_recv_size; i++)
        {
            printf("Copied: %c\n", g_curr_recv_buf[i]);
        	g_curr_send_buf[g_send_size++] = g_curr_recv_buf[i];
        }
        g_recv_size = 0;
    }
    
    if(g_send_size) 
    {
        uint8_t error;
        uint8_t size = g_send_size;
        g_send_size = 0;

        error = USB_Class_CDC_Send_Data(g_app_handle, DIC_BULK_IN_ENDPOINT,
        	g_curr_send_buf, size);
        if (!error && !(size % DIC_BULK_IN_ENDP_PACKET_SIZE)) {
            /* If the last packet is the size of endpoint, then send also zero-ended packet,
            ** meaning that we want to inform the host that we do not have any additional
            ** data, so it can flush the output.
            */
            error = USB_Class_CDC_Send_Data(g_app_handle, DIC_BULK_IN_ENDPOINT, NULL, 0);
        }
        if(error != USB_OK) 
        {
            /* Failure to send Data Handling code here */
        } 
    }
    return;
}

/******************************************************************************
 * 
 *    @name        USB_App_Callback
 *    
 *    @brief       This function handles the callback  
 *                  
 *    @param       handle : handle to Identify the controller
 *    @param       event_type : value of the event
 *    @param       val : gives the configuration value 
 * 
 *    @return      None
 *
 *****************************************************************************/
void USB_App_Callback(uint8_t event_type, void* val,void *arg) 
{
    UNUSED_ARGUMENT (arg)
    UNUSED_ARGUMENT (val)    
    if(event_type == USB_APP_BUS_RESET) 
    {
        start_app=FALSE;    
    }
    else if(event_type == USB_APP_ENUM_COMPLETE) 
    {
        start_app=TRUE;        
    }
    else if(event_type == USB_APP_ERROR)
    {
        /* add user code for error handling */
    }
    
    return;
}

/******************************************************************************
 * 
 *    @name        USB_Notif_Callback
 *    
 *    @brief       This function handles the callback  
 *                  
 *    @param       handle:  handle to Identify the controller
 *    @param       event_type : value of the event
 *    @param       val : gives the configuration value 
 * 
 *    @return      None
 *
 *****************************************************************************/
 
void USB_Notif_Callback(uint8_t event_type,void* val,void *arg) 
{
    uint32_t handle;
    uint8_t index;
    
    handle = *((uint32_t *)arg);
    if(start_app == TRUE) 
    {
        if(event_type == USB_APP_CDC_DTE_ACTIVATED) 
        {
           start_transactions = TRUE;        
        } 
        else if(event_type == USB_APP_CDC_DTE_DEACTIVATED) 
        {
           start_transactions = FALSE;        
        }
        else if((event_type == USB_APP_DATA_RECEIVED) && (start_transactions == TRUE))
        {

            uint32_t BytesToBeCopied;            
            APP_DATA_STRUCT* dp_rcv;
            dp_rcv = (APP_DATA_STRUCT*)val; 
            
            BytesToBeCopied = dp_rcv->data_size;
            for(index = 0; index < BytesToBeCopied; index++) 
            {
                g_curr_recv_buf[index] = dp_rcv->data_ptr[index];
             //   printf("Received: %c\n", g_curr_recv_buf[index]);
            }
            g_recv_size = index;

#if !TURINGSENSE
           /* Schedule buffer for next receive event */
            USB_Class_CDC_Recv_Data(handle, DIC_BULK_OUT_ENDPOINT, g_curr_recv_buf, DIC_BULK_OUT_ENDP_PACKET_SIZE); 
#endif /* !TURINGSENSE */
        }        
        else if((event_type == USB_APP_SEND_COMPLETE)&&
            (start_transactions == TRUE))
        { 
            /* User: add your own code for send complete event */ 
            APP_DATA_STRUCT* dp_rcv;
            dp_rcv = (APP_DATA_STRUCT*)val; 
            uint32_t BytesToBeCopied;            
            BytesToBeCopied = dp_rcv->data_size;
        if (BytesToBeCopied != 0)
        {
            /* If the last packet is the size of endpoint, then send also zero-ended packet,
            ** meaning that we want to inform the host that we do not have any additional
            ** data, so it can flush the output.*/
            if ( !(BytesToBeCopied % DIC_BULK_IN_ENDP_PACKET_SIZE))
            {
                USB_Class_CDC_Send_Data(g_app_handle, DIC_BULK_IN_ENDPOINT, NULL, 0);
            }
            else
            {
              if (BytesToBeCopied != 0) {
                send_complete = 1;
              }
            }
        }
        }
    }
    return;
}

/*FUNCTION*----------------------------------------------------------------
* 
* Function Name  : Main_Task
* Returned Value : None
* Comments       :
*     First function called.  Calls the Test_App
*     callback functions.
* 
*END*--------------------------------------------------------------------*/
void Usb_Cdc_Task
   (
      uint32_t param
   )
{   
    UNUSED_ARGUMENT (param)
    TestApp_Init();  
   
}

/* EOF */
