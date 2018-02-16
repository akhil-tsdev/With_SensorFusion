/*
*/
#include "main.h"
#include "tmp106.h"
//#include "AD1.h"

///LDD_TDeviceData * ADCPtr;
#define POT_SENSOR_ON (GPIO_PORT_A | GPIO_PIN2)
#define LIGHT_SENSOR_ON (GPIO_PORT_A | GPIO_PIN13)
#define MOTION_SENSOR_ON (GPIO_PORT_A | GPIO_PIN12)
#define SP141_GPIO_POWER  (GPIO_PORT_B | GPIO_PIN19)


const ADC_INIT_STRUCT adc_init = {
  ADC_RESOLUTION_DEFAULT,     /* resolution */
};

#define MY_TRIGGER ADC_PDB_TRIGGER    

ADC_INIT_CHANNEL_STRUCT adc_channel_param = 
{
    0 ,  ///ADC0_SOURCE_AD7B,
    ADC_CHANNEL_MEASURE_ONCE | ADC_CHANNEL_START_NOW, /* one sequence is sampled after fopen */
    15,             /* number of samples in one run sequence */
    150000,         /* time offset from trigger point in us */
    600000,         /* period in us (= 0.6 sec) */
    0x10000,        /* scale range of result (not used now) */
    15,             /* circular buffer size (sample count) */
    MY_TRIGGER,     /* logical trigger ID that starts this ADC channel */
#if MQX_USE_LWEVENTS
    NULL
#endif
};  

static MQX_FILE_PTR sns_single_port=NULL;

static void adc_sensor_read(A_INT32 gpio_pin,uint_16 adc_source, uint_32* result)
{
    MQX_FILE_PTR f, f_ch1;
    ADC_RESULT_STRUCT data;
    _mqx_int i;
    
    const uint_32 pot_sns_set[] = {
        gpio_pin | GPIO_PIN_STATUS_1,
        GPIO_LIST_END
    };
    printf("Opening GPIO %08x ... \n",gpio_pin);

    /* Open and set port TC as output to drive LEDs (LED10 - LED13) */
    sns_single_port = fopen("gpio:write", (char_ptr) &pot_sns_set);

    if (sns_single_port)
        ioctl(sns_single_port, GPIO_IOCTL_WRITE_LOG1, NULL);  
    _time_delay(200);
    
    printf("Opening channel %08x ... \n",adc_source);

    f = fopen(BSP_ATHEROS_ADC_SENSOR_DEVICE,(char_ptr)&adc_init);
    if(f != NULL)
    {    
        printf("done\n");
    }
    else
    {    
        printf("failed\n");
    }
    adc_channel_param.source = adc_source ;
    f_ch1  = fopen(BSP_ATHEROS_ADC_SENSOR_DEVICE "first", (const char*)&adc_channel_param);
    if(f_ch1 != NULL)
    {    
        printf("done, prepared to start by trigger\n");
    }
    else
    {    
        printf("failed\n");
    }    

    _time_delay(1000);
    
    printf("Triggering channel #1...");
    ioctl(f, ADC_IOCTL_FIRE_TRIGGER, (pointer) MY_TRIGGER);
    printf("triggered!\n");
    
    ///for(i = 0; ; i++) 
    {    
        /* channel 1 sample ready? */
        _time_delay(100);

        if (read(f_ch1, &data, sizeof(data) ))
            printf("ADC ch 1: %08x ", data.result);
        else
            printf("               ");
        
        _time_delay(100);
        
        if (IO_OK == ioctl (f_ch1, ADC_IOCTL_PAUSE_CHANNEL, NULL))
          printf("stopped!\n");
        else
          printf("failed!\n");

        _time_delay(100);
    }    
    fclose(f_ch1);
    fclose(f);
    if (sns_single_port)
        ioctl(sns_single_port, GPIO_IOCTL_WRITE_LOG0, NULL);      
    fclose(sns_single_port);
    
    *result = data.result;
  
    
    
}

static void adc_motion_read(uint_32* x,uint_32* y,uint_32* z)
{
    MQX_FILE_PTR f, f_ch1,f_ch2,f_ch3;
    ADC_RESULT_STRUCT data;
    _mqx_int i;
    
    const uint_32 pot_sns_set[] = {
        MOTION_SENSOR_ON | GPIO_PIN_STATUS_1,
        GPIO_LIST_END
    };
    /* Open and set port TC as output to drive LEDs (LED10 - LED13) */
    sns_single_port = fopen("gpio:write", (char_ptr) &pot_sns_set);

    if (sns_single_port)
        ioctl(sns_single_port, GPIO_IOCTL_WRITE_LOG1, NULL);  
    _time_delay(200);
    
    f = fopen(BSP_ATHEROS_ADC_SENSOR_DEVICE,(char_ptr)&adc_init);
    if(f != NULL)
    {    
        printf("done\n");
    }
    else
    {    
        printf("failed\n");
    }
    adc_channel_param.source = ADC0_SOURCE_AD0 ;
    f_ch1  = fopen(BSP_ATHEROS_ADC_SENSOR_DEVICE "first", (const char*)&adc_channel_param);
    if(f_ch1 != NULL)
    {    
        printf("X CHANNEL done, prepared to start by trigger\n");
    }
    else
    {    
        printf("x failed\n");
    }    

    _time_delay(1000);

    adc_channel_param.source = ADC0_SOURCE_AD3 ;
    f_ch2  = fopen(BSP_ATHEROS_ADC_SENSOR_DEVICE "second", (const char*)&adc_channel_param);
    if(f_ch2 != NULL)
    {    
        printf("Y channel done, prepared to start by trigger\n");
    }
    else
    {    
        printf("y failed\n");
    }    

    _time_delay(1000);    

    adc_channel_param.source = ADC0_SOURCE_AD23 ;
    f_ch3  = fopen(BSP_ATHEROS_ADC_SENSOR_DEVICE "third", (const char*)&adc_channel_param);
    if(f_ch3 != NULL)
    {    
        printf("Z channel  done, prepared to start by trigger\n");
    }
    else
    {    
        printf("z failed\n");
    }    

    _time_delay(1000);      
    
    printf("Triggering channel #1...");
    ioctl(f, ADC_IOCTL_FIRE_TRIGGER, (pointer) MY_TRIGGER);
    printf("triggered!\n");
    
    ///for(i = 0; ; i++) 
    {    
        /* channel 1 sample ready? */
        _time_delay(100);

        if (read(f_ch1, &data, sizeof(data) ))
            printf("ADC ch 1: X Channel %08x \n", data.result);
        else
            printf("               ");
        memset(&data,0,sizeof(data));
                _time_delay(100);
        if (read(f_ch2, &data, sizeof(data) ))
            printf("ADC ch 1: Y Channel %08x \n", data.result);
        else
            printf("               ");
        memset(&data,0,sizeof(data));
                _time_delay(100);
                
        if (read(f_ch3, &data, sizeof(data) ))
            printf("ADC ch 1: Z Channel %08x \n", data.result);
        else
            printf("               ");
        memset(&data,0,sizeof(data));
        
        _time_delay(100);
        
        if (IO_OK == ioctl (f_ch1, ADC_IOCTL_PAUSE_CHANNEL, NULL))
          printf("X  stopped!\n");
        else
          printf("failed!\n");

        if (IO_OK == ioctl (f_ch2, ADC_IOCTL_PAUSE_CHANNEL, NULL))
          printf("Y  stopped!\n");
        else
          printf("failed!\n");
        
        if (IO_OK == ioctl (f_ch3, ADC_IOCTL_PAUSE_CHANNEL, NULL))
          printf("Z  stopped!\n");
        else
          printf("failed!\n");        
        _time_delay(100);
    }    
    fclose(f_ch1);
    fclose(f_ch2);
    fclose(f_ch3);
    
    fclose(f);
    //if (sns_single_port)
    //    ioctl(sns_single_port, GPIO_IOCTL_WRITE_LOG0, NULL);      
    fclose(sns_single_port);
    
 
    
    
}


A_INT32 sensor_handle(A_INT32 argc, char* argv[] )
{
    A_BOOL                 shorthelp = FALSE;
    A_UINT8 		    printhelp = 0;
    A_INT32                 return_code = A_OK;
    A_UINT32                 index = 1;
    A_UINT8                  reg ;
    A_UINT16                   tmp_value = 0;
    A_UINT8                    tmp_cfg_value = 0;
    A_UINT32               result = 0 ;
    MQX_FILE_PTR                f=NULL;    
    TMP106_Config             cfg;
    //TMP106_Config            cfg;
    
    
#define ADC_SAMPLE_CHANNEL_POTENTTIOMETER  0
#define ADC_SAMPLE_CHANNEL_LIGHT           1
#define ADC_SAMPLE_CHANNEL_ACCELEROMETER_X 2      
#define ADC_SAMPLE_CHANNEL_ACCELEROMETER_Y 3
#define ADC_SAMPLE_CHANNEL_ACCELEROMETER_Z 4    
    

    if (argc > index)
    {
        if (ATH_STRCMP(argv[index], "--help") == 0)
        {

            printf ("Usage: %s [<command>]\n", argv[0]);
            printf ("  Commands:\n");
            printf ("    --tmprd         = Read the current temperature \n");
            printf ("    --tmpinit       = Init  temperature sensor\n");            
            printf ("    --tmpcfgrd      = Read  temperature sensor configuration\n");     
            printf ("    --tmpcfgwr      = Write temperature sensor configuration values\n"); 
            printf ("    --adcinit       = Init the ADC bus channel \n");                    
            printf ("    --potentiometer = Read potentiometer sensor values\n");            
            printf ("    --light         = Read Light sensor values\n");            
            printf ("    --accelerometer = Read accelerometer sensor values\n");            
            printf ("    --sp141down     = shutdown SP141 \n");            
            printf ("    --sp141UP       = POWERUP SP141 \n");                        

            
            return return_code;
            
        }else  if (ATH_STRCMP(argv[index], "--tmprd") == 0)
        {
            tmp106_read_temp(&tmp_value);
            printf("the temperature of TMP106 is : %04x \n",tmp_value);
        }else  if (ATH_STRCMP(argv[index], "--tmpinit") == 0)
        {
            if(!tmp106_init())
            {
              printf("tmp106 device init failed ! \n");
              return_code = A_ERROR;
            }
        }
        else  if (ATH_STRCMP(argv[index], "--tmpcfgrd") == 0)
        {
            if (!tmp106_get_config(&cfg))
            {
              printf("read tmp106 device  failed ! \n");
              return_code = A_ERROR;
            }
            
            printf("TMP106 config : \n");
            printf("ShutDownMode = %d: \n",cfg.ShutDownMode);            
            printf("Thermostat = %d: \n",cfg.Thermostat);            
            printf("CvtReso = %d: \n",cfg.CvtReso);
            printf("FaultQueue = %d: \n",cfg.FaultQueue);            
            printf("OneShot = %d: \n",cfg.OneShot);                        
            printf("Polarity = %d: \n",cfg.Polarity);                                   
              
        }
        else  if (ATH_STRCMP(argv[index], "--tmpcfgwr") == 0)
        {
          memset(&cfg,0,sizeof(cfg));
          tmp_cfg_value = atoi(argv[index+1]);
          cfg.ShutDownMode  = (tmp_cfg_value&TMP106_CFG_SD_MASK);
          cfg.Thermostat  = (tmp_cfg_value&TMP106_CFG_TM_MASK);          
          cfg.Polarity  = (tmp_cfg_value&TMP106_CFG_POL_MASK);          
          cfg.FaultQueue  = (tmp_cfg_value&TMP106_CFG_CVT_MASK);          
          cfg.OneShot  = (tmp_cfg_value&TMP106_CFG_OS_MASK); 
          if (!tmp106_set_config(cfg))
          {
            printf("read tmp106 device  failed ! \n");
            return_code = A_ERROR;
          }
        }else  if (ATH_STRCMP(argv[index], "--adcinit") == 0)
        {
/*          
          ADCPtr = NULL ; 
          ADCPtr = AD1_Init(NULL);
          if (NULL == ADCPtr)
          {
            printf("call AD1_Init failedd \n");
            return_code = A_ERROR;
          }
  */        
        }else  if (ATH_STRCMP(argv[index], "--potentiometer") == 0)
        {
#if (_MQX_LITE_ && _DB142_)
          AD1_TResultData StoreData = 0;
          if(ADCPtr != NULL)
          {
            if (AD1_SelectSampleGroup(ADCPtr,ADC_SAMPLE_CHANNEL_POTENTTIOMETER) != ERR_OK )
            {
              printf("call AD1_SelectSampleGroup failed !\n");                
              return A_ERROR;
            }
            
            AD1_StartSingleMeasurement(ADCPtr);
            return_code = AD1_GetMeasuredValues(ADCPtr,(LDD_TData*)&StoreData);
            printf("the potentiometer = %08x  \n",StoreData);
            return_code = A_OK;
          }else {
            printf("please Init the ADC device firstly !\n");
            return_code = A_ERROR;
          }
#else 
          printf("Begin to read the ADC7b \n");
          adc_sensor_read(POT_SENSOR_ON,ADC0_SOURCE_AD7B,&result);
          printf("### result = %08x ##### \n",result);
#endif           
                        

        }

        else if (ATH_STRCMP(argv[index], "--light") == 0)
        {
          printf("Begin to read the ADC6b \n");
          adc_sensor_read(LIGHT_SENSOR_ON,ADC0_SOURCE_AD6B,&result);
          printf("### result = %08x ##### \n",result);
        }
        else if (ATH_STRCMP(argv[index], "--accelerometer") == 0)
        {
          //adc_motion_read(NULL,NULL,NULL);
          adc_sensor_read(MOTION_SENSOR_ON,ADC0_SOURCE_AD0,&result);
          printf("### X result = %08x ##### \n",result);
          adc_sensor_read(MOTION_SENSOR_ON,ADC0_SOURCE_AD3,&result);
          printf("### Y result = %08x ##### \n",result);
          adc_sensor_read(MOTION_SENSOR_ON,ADC0_SOURCE_AD23,&result);
          printf("### Z result = %08x ##### \n",result);
        }else if (ATH_STRCMP(argv[index], "--sp141down") == 0)
        {
                    
            const uint_32 output_set[] = {
                SP141_GPIO_POWER | GPIO_PIN_STATUS_1,
                GPIO_LIST_END
            };

            /* Open and set port TC as output to drive LEDs (LED10 - LED13) */
            f = fopen("gpio:write", (char_ptr) &output_set);

            if (f)
                ioctl(f, GPIO_IOCTL_WRITE_LOG1, NULL);              
          
            fclose(f);
        }else if (ATH_STRCMP(argv[index], "--sp141up") == 0)
        {
            const uint_32 output_set[] = {
                SP141_GPIO_POWER | GPIO_PIN_STATUS_0,
                GPIO_LIST_END
            };

            /* Open and set port TC as output to drive LEDs (LED10 - LED13) */
            f = fopen("gpio:write", (char_ptr) &output_set);

            if (f)
                ioctl(f, GPIO_IOCTL_WRITE_LOG0, NULL);              
          
            fclose(f);          
        } else if (ATH_STRCMP(argv[index], "--temp_off") == 0)
        {
            tmp106_get_config(&cfg);
            cfg.ShutDownMode =  TRUE;
            tmp106_set_config(cfg);
        } else if (ATH_STRCMP(argv[index], "--temp_on") == 0)
        {
            tmp106_get_config(&cfg);
            cfg.ShutDownMode =  FALSE;
            tmp106_set_config(cfg);

        } else if (ATH_STRCMP(argv[index], "--tmplowpower") == 0)
        {
            //if(!tmp106_init())
            //{
            //    printf("tmp106 device init failed ! \n");
            //    return_code = A_ERROR;
            //}
            if(!tmp106_deinit())
            {
                printf("tmp106 device de-init failed ! \n");
                return_code = A_ERROR;
            }
        }
      }

    return return_code;
  
}