/*
*/
#include "main.h"
//#include "tmp106.h"
//#include "AD1.h"

///LDD_TDeviceData * ADCPtr;

A_INT32 sensor_handle(A_INT32 argc, char* argv[] )
{
    A_BOOL                 shorthelp = FALSE;
    A_UINT8 		    printhelp = 0;
    A_INT32                 return_code = A_OK;
    A_UINT32                 index = 1;
    A_UINT8                  reg ;
    A_UINT16                   tmp_value = 0;
    A_UINT8                    tmp_cfg_value = 0;
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

            return return_code;
            
        }
#if 0      
        
        else  if (ATH_STRCMP(argv[index], "--tmprd") == 0)
        {
            reg = REG_TMP106_TMP_REG;  
            tmp106_read_temp(&tmp_value);
            printf("the temperature of TMP106 is : %04x \n",tmp_value);
        }else  if (ATH_STRCMP(argv[index], "--tmpinit") == 0)
        {
            if(!tmp106_init())
            {
              printf("tmp106 device init failed ! \n");
              return_code = A_ERROR;
            }
        }else  if (ATH_STRCMP(argv[index], "--tmpcfgrd") == 0)
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
              
        }else  if (ATH_STRCMP(argv[index], "--tmpcfgwr") == 0)
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
        }else  
#endif 
          if (ATH_STRCMP(argv[index], "--adcinit") == 0)
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
#if  BSPCFG_ENABLE_ADC_SENSOR
    MQX_FILE_PTR f, f_ch1;
    ADC_RESULT_STRUCT data;
    _mqx_int i;
    const ADC_INIT_STRUCT adc_init = {
      ADC_RESOLUTION_DEFAULT,     /* resolution */
    };
#if 1    
#define MY_TRIGGER ADC_PDB_TRIGGER    
    const ADC_INIT_CHANNEL_STRUCT adc_channel_param1 = 
    {
        ADC0_SOURCE_AD7B,
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
#endif     
    printf("Opening channel #7b ...");

    f = fopen(BSP_ATHEROS_ADC_SENSOR_DEVICE,(char_ptr)&adc_init);
    if(f != NULL)
    {    
        printf("done\n");
    }
    else
    {    
        printf("failed\n");
    }
    
    f_ch1  = fopen(BSP_ATHEROS_ADC_SENSOR_DEVICE "first", (const char*)&adc_channel_param1);
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
    
    for(i = 0; ; i++) 
    {    
        /* channel 1 sample ready? */
        if (read(f_ch1, &data, sizeof(data) ))
            printf("ADC ch 1: %4d ", data.result);
        else
            printf("               ");

#if ADC_MAX_MODULES > 1
#if 0
        /* channel 2 sample ready? */
        if (read(f_ch2, &data, sizeof(data) ))
            printf("ADC ch 2: %d\n", data.result);
        else
#endif           
            printf("\n");
#else
        printf("\n");
#endif
        if (i == 37) {
            printf("Pausing channel #1...");
            if (IO_OK == ioctl (f_ch1, ADC_IOCTL_PAUSE_CHANNEL, NULL))
              printf("stopped!\n");
            else
              printf("failed!\n");
            
            break;
        }
        if (i == 53) {
            printf("Resuming channel #1...");
            if (IO_OK == ioctl (f_ch1, ADC_IOCTL_RESUME_CHANNEL, NULL))
              printf("resumed!\n");
            else
              printf("failed!\n");
        }

        _time_delay(100);
    }    
    fclose(f_ch1);
    fclose(f);
    
#endif   
#endif           
                        

        }
/*
        else if (ATH_STRCMP(argv[index], "--light") == 0)
        {
          AD1_TResultData StoreData = 0;
          if(ADCPtr != NULL)
          {
            if (AD1_SelectSampleGroup(ADCPtr,ADC_SAMPLE_CHANNEL_LIGHT) != ERR_OK )
            {
              printf("call AD1_SelectSampleGroup failed !\n");                
              return A_ERROR;
            }
            
            AD1_StartSingleMeasurement(ADCPtr);
            return_code = AD1_GetMeasuredValues(ADCPtr,(LDD_TData*)&StoreData);
            printf("the Light = %08x  return_code = %08x\n",StoreData,return_code);
            return_code = A_OK;
          }else {
            printf("please Init the ADC device firstly !\n");
            return_code = A_ERROR;
          }
        }else if (ATH_STRCMP(argv[index], "--accelerometer") == 0)
        {
          AD1_TResultData StoreData = 0;
          if(ADCPtr != NULL)
          {
            if (AD1_SelectSampleGroup(ADCPtr,ADC_SAMPLE_CHANNEL_ACCELEROMETER_X) != ERR_OK )
            {
              printf("call AD1_SelectSampleGroup failed !\n");                
              return A_ERROR;
            }
            
            AD1_StartSingleMeasurement(ADCPtr);
            return_code = AD1_GetMeasuredValues(ADCPtr,(LDD_TData*)&StoreData);
            printf("the ACCELEROMETER_X = %08x  \n",StoreData);
            
            StoreData = 0 ; 
            if (AD1_SelectSampleGroup(ADCPtr,ADC_SAMPLE_CHANNEL_ACCELEROMETER_Y) != ERR_OK )
            {
              printf("call AD1_SelectSampleGroup failed !\n");                
              return A_ERROR;
            }
            
            AD1_StartSingleMeasurement(ADCPtr);
            return_code = AD1_GetMeasuredValues(ADCPtr,(LDD_TData*)&StoreData);
            printf("the ACCELEROMETER_Y = %08x  \n",StoreData);
            
            StoreData = 0 ;             
            if (AD1_SelectSampleGroup(ADCPtr,ADC_SAMPLE_CHANNEL_ACCELEROMETER_Z) != ERR_OK )
            {
              printf("call AD1_SelectSampleGroup failed !\n");                
              return A_ERROR;
            }
            
            AD1_StartSingleMeasurement(ADCPtr);
            return_code = AD1_GetMeasuredValues(ADCPtr,(LDD_TData*)&StoreData);
            printf("the ACCELEROMETER_Z = %08x  \n",StoreData);
            return_code = A_OK;
          }else {
            printf("please Init the ADC device firstly !\n");
            return_code = A_ERROR;
          }          
        }
      }

*/        
    }
    return return_code;
  
}