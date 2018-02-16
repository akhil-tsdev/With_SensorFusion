/*
 * Copyright TuringSense, Inc © 2015
 * hub_22f.c
 *
 *  Created on: May 15, 2015
 *      Author: cwati
 *
 *
 */

#include <mqx.h>
#include <mutex.h>

#include "atheros_main_loop.h"
#include "hub_main_loop.h"
#include "hub_to_nordic.h"
#include "main.h"
#include "ts_hub_flash.h"
#include "throughput.h"
#include "wmiconfig_ts.h"
#include "adc_ts.h"
#include "ts_io_led.h"
 
/* PIVOT 3.0 POC */
#if PIVOT_3_0
#include "pivot3_0_poc.h"
#endif

/*************************** GLOBAL ************************/

const uint8_t enable_stats = 1;

cloud_to_hub_t cloud_to_hub;
hub_to_nordic_t hub_to_nordic;
nordic_to_hub_t nordic_to_hub;
hub_to_cloud_t hub_to_cloud;

uint32_t current_state;
cbuf_t cbuf[MAX_SENSORS];

/* Current count */
uint32_t cur_num_of_sat = 6;
uint32_t cur_sat_ids[MAX_SATELLITES]/* = {313,0x1b,0x2b,0x3b,330,327}*/;


uint32_t cur_hub_id = DEFAULT_HUB_ID;
uint32_t cur_hub_lcp = INITIAL_LCP;
uint32_t hubType_comPort = HUB_TYPE_DIRECT;     /* If we use Wifi Direct, or HUB_TYPE_DIRECT=0 */
uint32_t hubType_IPAdress = 0;

static bool hub_can_send = false;
  
static TIME_STRUCT last_bat_read; /* Last time reading battery */

/***********************************************************/
uint32_t hub_active (uint32_t c) {
	return (hub_can_send == true);
}
uint32_t hub_not_active (uint32_t c) {
	return (hub_can_send == false);
}

void set_hub_active(uint32_t* c) {
        hub_can_send = true;
		hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_COMMA] = SAT_COMM_START_DUMP;
}

void set_hub_not_active(uint32_t* c) {
        hub_can_send = false;
		hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_COMMA] = SAT_COMM_STOP_DUMPI;
}


/* Function: clr_atheros_GPIO
 * Parameters: none
 *
 * This function will turn on Green LED
 * */
void clr_ath_GPIO(MQX_FILE_PTR pins_fd) {
	const GPIO_PIN_STRUCT pins[] = {
	GPIO_GREEN_LED,
	GPIO_LIST_END };

	pins_fd = fopen("gpio:output", (char *) &pins);

	if (NULL == pins_fd) {
		printf("ERROR: Failed to open GPIO char device to turn on green LED!\n");
	}

	if (ioctl(pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
//		printf("OK write log0\n");
	}
	fclose(pins_fd);
}

/* Function: clr_atheros_GPIO
 * Parameters: none
 *
 * This function will clear GPIO pin.
 * Will turn off Green LED.
 */
void set_ath_GPIO(MQX_FILE_PTR pins_fd) {
	const GPIO_PIN_STRUCT pins[] = {
	GPIO_GREEN_LED,
	GPIO_LIST_END };

	pins_fd = fopen("gpio:output", (char *) &pins);

	if (NULL == pins_fd) {
		printf(
				"ERROR: Failed to open GPIO char device to turn off green LED!\n");
	}

	if (ioctl(pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
//		printf("OK write log1\n");
	}
	fclose(pins_fd);
}

/* Function: assert_nordic_reset
 * Parameters: none
 *
 * This function will assert nordic soft reset pin.
 *
 */
void assert_nordic_reset() {
  MQX_FILE_PTR pins_fd;
  const GPIO_PIN_STRUCT pins[] = {NORDIC_RST, GPIO_LIST_END };
  uint8_t rgb[3] = {0};
  
  pins_fd = fopen("gpio:output", (char *) &pins);
  if (NULL == pins_fd) {
#if EVAL_BOARD
    printf("ERROR: Failed to open GPIO char device to reset Nordic!\n");
#endif
  }

  if (ioctl(pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK) {
  //		printf("OK write log1\n");
  }
  /* Turn off the LEDs */
  rgb[redLed] = ts_get_red_LED();
  rgb[greenLed] = ts_get_green_LED();
  rgb[blueLed] = ts_get_blue_LED();
  ts_turn_on_LED(offLed);

  app_time_delay(300); /* Wait in millisecond.  Minimum 100-150ms. */

  if (ioctl(pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK) {
  //		printf("OK write log1\n");
  }
  
    app_time_delay(NORDIC_BL_WAIT_MS + NORDIC_BL_WAIT2_MS); /* Wait for Nordic to get out of BL mode. */

  /* Turn on the LEDs.  Why do we flip the value you ask?
   * Because ts_get_<color>_LED returns the GPIO pin value, and
   * ts_turnOnOneLED accepts "on" or "off" state according to human eye.
   * These two are opposite of each other, ie if GPIO pin value is 0, then it turns on. */
  ts_turnOnOneLED(redLed,!rgb[redLed]);
  ts_turnOnOneLED(greenLed,!rgb[greenLed]);
  ts_turnOnOneLED(blueLed,!rgb[blueLed]);

  fclose(pins_fd);

}


/* Returns
 * 1	: if any cbuf has fill
 * 0	: if no cbuf has fill.  All cbufs are empty.
 */
uint8_t cbufs_not_empty(void) {
	uint8_t has_fill = 0;

	for (uint8_t i = 0; i < cur_num_of_sat; i++) {
		if (!cbufIsEmpty(&(cbuf[i]))) {
			has_fill = 1;
			return 1;
		}
	}

	return 0;
}

/* Returns
 * 1	: if all cbufs have entry
 * 0	: if any cbuf doesn't have entry
 */
uint8_t cbufs_all_has_entry(void) {
	uint8_t has_entry = 1;

	for (uint8_t i = 0; i < cur_num_of_sat; i++) {
		if (cbufIsEmpty(&(cbuf[i]))) {
			has_entry = 0;
			return 0;
		}
	}

	return 1;
}

/* Returns:
 * 	1: if all cbuf has at least space to read one more nordic spi
 * 	0: if any cbuf is still kind of empty
 */
static uint32_t all_cbuf_fill_pct(void) {
	uint32_t 		fill_pct, max_fill;
	uint8_t			empty_cbuf = 0;

	for (uint8_t i = 0; i < cur_num_of_sat; i++) {
		if (cbufIsEmpty(&cbuf[i])) {
			return 0;
		}
		fill_pct = cbufNum(&(cbuf[i]));
                max_fill = CB_SIZE - MAX_SENSOR_RECORDS_PER_PACKET;
		if (fill_pct < max_fill) {
			return 0;
		}
	}

	return 1;
}

/* Returns:
 * 	cbuf with the highest fill, in percentage, rounded to integer.
 */
uint32_t max_cbuf_fill_pct() {
    uint32_t 		max_pct = 0;
    uint32_t 		fill_pct;

    for (uint8_t i = 0; i < cur_num_of_sat; i++) {
            if (cbufIsEmpty(&cbuf[i])) {
                    continue;
            }
            fill_pct = cbufNum(&(cbuf[i])) * 100 / CB_SIZE;
            if (fill_pct > max_pct) {
                    max_pct = fill_pct;
            }
    }
    return (max_pct);
}

void empty_cbuf(void) {
  for (uint8_t sat_idx = 0; sat_idx < MAX_SENSORS; sat_idx++) {
          cbufInit(&cbuf[sat_idx]);
  } 
}
/*TASK*-------------------------------------------------------------------
 *
 * Task Name : hub_quit_loop
 * Comments  : Called when user types in "quittx"
 *
 *END*----------------------------------------------------------------------*/
static A_INT32 hub_quit_loop(void) {
	deinit_nordic();

	return A_OK;
}

uint32_t readBlockFromFlash(int mPos, uint32_t defaultValue)
{
#if USE_FLASH
    err_t           flash_ret;
    uint32_t    temp;
        
    // Read hub ID 
    flash_ret = hub_flash_read32(&temp, mPos); 
    if (flash_ret != E_OK) {
        printf("\nERROR! Could not read from flash. Exiting...");
        ts_blinkLEDerror(redLed, 4);
    }
    if (temp != 0xFFFFFFFF)
      return temp;
    else
      return defaultValue;
    
#endif
}

uint32_t readHubIdFromFlash(void)
{
#if USE_FLASH
    err_t           flash_ret;
    uint32_t    temp_hub_id;
        
    // Read hub ID 
    flash_ret = hub_flash_read32(&temp_hub_id, HUB_ID_OFF); 
    if (flash_ret != E_OK) {
        printf("\nERROR! Could not read from flash. Exiting...");
        ts_blinkLEDerror(redLed, 4);
    }
    if (temp_hub_id != 0xFFFFFFFF)
      return temp_hub_id;
    else
      return DEFAULT_HUB_ID;
    
#endif
}

uint32_t readHubLcpFromFlash(void)
{
#if USE_FLASH
    
    err_t flash_ret;
    uint32_t temp_hub_lcp;
          
    // Read hub LCP
    flash_ret = hub_flash_read32(&temp_hub_lcp, HUB_LCP_OFF); 
    if (flash_ret != E_OK) {
        printf("\nERROR! Could not read from flash. Exiting...");
        ts_blinkLEDerror(redLed, 4);
    }
    if ((temp_hub_lcp != 0xFFFFFFFF)&&(temp_hub_lcp >= 0)&&(temp_hub_lcp <=MAX_LCP))
      return temp_hub_lcp;
    else
      return INITIAL_LCP;
    
#endif
}

// 15 satellites -> satn from 1 to 15;
uint32_t readSatIdList(int satn)
{
  #if USE_FLASH
    
    err_t flash_ret;
    
    uint32_t temp_sat_id = 0;

    // make sure we are not reading outside the sat ID list area
    if (((satn+2) > TS_SATID_LIST_SIZE) || ((satn+2) < 2)) 
      return 0xFFFFFFFF;
    
    flash_ret = hub_flash_read32(&temp_sat_id, TS_FLASH_OFFSET(satn+1)); 
    if (flash_ret != E_OK) {
        printf("\nERROR! Could not read from flash. Exiting...");
        ts_blinkLEDerror(redLed, 4);
    }

	return temp_sat_id;
    

#endif
}
void readAllSatIdList()
{
        cur_num_of_sat = 0;
        cur_num_of_sensors = 0;
        
	for (uint8_t cnt1 = 0; cnt1 < MAX_SATELLITES; cnt1++)
        {
           cur_sat_ids[cnt1] = 0;
        }
	for (uint8_t cnt1 = 0; cnt1 < MAX_SATELLITES; cnt1++) {
            uint32_t satTMP = readSatIdList(cnt1+1);
            if ((satTMP != 0xFFFFFFFF) && (satTMP != 0))
            {
                cur_sat_ids[cnt1] = satTMP;
		
		cur_num_of_sat++;
                /* cwati TODO TODO TODO Yoga POC.  Num of sensors per sat should be stored in flash!!! */
                cur_num_of_sensors += cur_sensors_per_sat[cnt1];
            }
	}
}

void sendSatIdToNordic(uint32_t pos, uint32_t satIDToSend, uint32_t curSatNum, uint32_t curSatNumSens)
{
	hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_COMMA] = SAT_COMM_WIF_SETSAT;
        hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALX1] = pos;
        hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALY2] = satIDToSend; 
        hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALZ3] = curSatNum; 
        hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALT4] = curSatNumSens; 
        
}

void sendHubIdToNordic(void)
{
#if USE_FLASH
        // Send hub id to the nordic
        hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_COMMA] = SAT_COMM_SET_HUB_ID;
        hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_SATID] = 0;
        hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALX1] = cur_hub_id;

#endif
}

void sendHubLcpToNordic(void)
{
#if USE_FLASH
        // Send hub id to the nordic
        hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_COMMA] = SAT_COMM_SET_HUBLCP;
        hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_SATID] = 0;
        hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_VALX1] = cur_hub_lcp;
#endif
}

/* Read Information from Hub Flash */
void readHubFlash()
{
    /* Read HUB ID */
    cur_hub_id = readHubIdFromFlash();
    
    /* Read HUB LCP */
    cur_hub_lcp = readHubLcpFromFlash();

    /* Read SAT ID LIST */
    readAllSatIdList();
}

/* Send Hub Flash Information to Nordic */
void sendHubFlashInfoToNordic() {
    /* Send Hub ID to Nordic */
    sendHubIdToNordic();
    talk_to_nordic(&nordic_to_hub);
    app_time_delay(20);

    /* Send HUB LCP to Nordic */
    sendHubLcpToNordic();
    talk_to_nordic(&nordic_to_hub);
    app_time_delay(5);

    /* Send SAT ID LIST */
    for (uint8_t cnt1 = 0; cnt1 < cur_num_of_sat; cnt1++) {
      sendSatIdToNordic(cnt1,cur_sat_ids[cnt1],cur_num_of_sat,cur_sensors_per_sat[cnt1]); 
      talk_to_nordic(&nordic_to_hub);
      app_time_delay(2);
    }  
}

/* 
* value:   value to be written to flash
* par:     field to write to flash, where
*   0:     N/A
*   1:     hub ID
*   2:     sat ID 0
*   3:     sat ID 1
*...
*   n:     sat ID n-1
*...
*  15:     sat ID 13
*  16:     sat ID 14
*  17:     sat ID 15
*  26:     LCP
* others:  ignored
*/
void writeHubFlash(uint32_t par, uint32_t value1, uint32_t value2)
{
#if USE_FLASH

    err_t       flash_ret;
    
    uint32_t lcp_tmp, hubid_tmp, ip_tmp, hubType_tmp, hubIP_tmp;
    uint32_t cur_sat_ids_tmp[TS_SATID_LIST_SIZE] = {0};

    // Store current values in temporary variables
    hubid_tmp = readHubIdFromFlash();
     
    lcp_tmp = readHubLcpFromFlash();
    
    hubType_tmp = readBlockFromFlash(HUB_TYP_OFF, 0);
    
    hubIP_tmp = readBlockFromFlash(HUB_IPA_OFF, 0);
    
    for(int count = 0; count < MAX_SENSORS; count++)
      cur_sat_ids_tmp[count] = readSatIdList(count+1);
        
    // Erase the whole sector
    flash_ret =  hub_flash_erase_ts_sector();
    if (flash_ret != E_OK) {
        printf("\nERROR! Could not erase the flash. Exiting...");
        ts_blinkLEDerror(redLed, 4);
    }
    
    // Write in the temp (local) variables
    switch (par)
    {
      case 1:
        hubid_tmp = value1;
        break;

      case 26:
          if ((value1 >= 0) && (value1 <= MAX_LCP))
            lcp_tmp = value1;
          else
            onlyTest(true);
          break;
        

        lcp_tmp = value1;
        break;
      
      case 27:
        hubType_tmp = value1;
        break;
      
      case 28:
        uint32_t ipA,ipMA,ipMB,ipB;
        ipB=(value2%1000);
        ipMB=((value2-ipB)/1000)*256;
        ipMA=(value1%1000)*(256*256);
        ipA=((value1-(value1%1000))/1000)*(256*256*256);
        hubIP_tmp = ipB+ipMB+ipMA+ipA;
        break;
      
      default:
        if ((par > 1) || (par <= (TS_SATID_LIST_SIZE + 2)))
          if (value1 != 0)
            cur_sat_ids_tmp[par-2] = value1;
          else
            onlyTest(true);
    }
    
    // Write back in the flash
    // HUB ID
    if (hubid_tmp != DEFAULT_HUB_ID)
    {
      flash_ret = hub_flash_write32(&hubid_tmp, HUB_ID_OFF); 
      if (flash_ret != E_OK) 
      {
        printf("\nERROR! Could not write the flash. Exiting...");
        ts_blinkLEDerror(redLed, 4);
       }
    }
    
    //SAT ID LIST
    for (int count = 0; count < TS_SATID_LIST_SIZE; count++)
    {
      flash_ret = hub_flash_write32(cur_sat_ids_tmp+count, TS_FLASH_OFFSET(count+2));
      if (flash_ret != E_OK) {
        printf("\nERROR! Could not write the flash. Exiting...");
        ts_blinkLEDerror(redLed, 4);
      }
    }
    
    // HUB LCP
    if (lcp_tmp != INITIAL_LCP)
    {  
      flash_ret = hub_flash_write32(&lcp_tmp, HUB_LCP_OFF); 
      if (flash_ret != E_OK) {
        printf("\nERROR! Could not write the flash. Exiting...");
        ts_blinkLEDerror(redLed, 4);
      }
    }
      // HUB TYPE
      flash_ret = hub_flash_write32(&hubType_tmp, HUB_TYP_OFF); 
      if (flash_ret != E_OK) {
          printf("\nERROR! Could not write the flash. Exiting...");
          ts_blinkLEDerror(redLed, 4);
      }
      
      // IP ADRESS
      flash_ret = hub_flash_write32(&hubIP_tmp, HUB_IPA_OFF); 
      if (flash_ret != E_OK) {
          printf("\nERROR! Could not write the flash. Exiting...");
          ts_blinkLEDerror(redLed, 4);
      }

    
 
#endif 
}

/* This function checks the battery status and updates via LED */
static void update_battery_led(void) {
    TIME_STRUCT             now, diff;
    static bool             first_time = true;
    uint32_t                diff_ms;
    float                   bat_level;
    uint8_t                 rgb[3] = {0};
 
    /* Only update via LED if hub is not in streaming-mode.
     * If you blink LED during streaming, it will affect the
     * streaming.  Unless you want to create a new task to
     * blink LED only, and do fancy task switch. 
     */
    if (hub_active(current_state)) {
      return;
    }
    
    if (first_time) {
      _time_get_elapsed(&last_bat_read);
      first_time = false;
    }
    
    _time_get_elapsed(&now);
    _time_diff(&last_bat_read, &now, &diff);
    diff_ms = diff.MILLISECONDS + diff.SECONDS * 1000;

    if ((diff_ms > BATTERY_READ_MS)) {
      /* Get battery level */
      bat_level = readBatteryApplyFilter(adc_read_1(0));
      
      /* Save LED info */
      rgb[redLed] = ts_get_red_LED();
      rgb[greenLed] = ts_get_green_LED();
      rgb[blueLed] = ts_get_blue_LED();

      /* If fully charged or enough battery then we don't need to blink during streaming */
      if (bat_level >= 4) {
        /* White for fully charged */  
        ts_doubleBlinkLED(whiteLed);
      } else if (((bat_level >= 3.6) && (bat_level < 4))) {
          /* Blue for normal, working level */
          ts_doubleBlinkLED(blueLed);
      } else {
        /* If running out of battery then have to blink. */
        /* Red means you should charge now. */
        ts_doubleBlinkLED(redLed);
      }
      
      /* Turn on the LEDs.  Why do we flip the value you ask?
       * Because ts_get_<color>_LED returns the GPIO pin value, and
       * ts_turnOnOneLED accepts "on" or "off" state according to human eye.
       * These two are opposite of each other, ie if GPIO pin value is 0, then it turns on. */
      ts_turnOnOneLED(redLed,!rgb[redLed]);
      ts_turnOnOneLED(greenLed,!rgb[greenLed]);
      ts_turnOnOneLED(blueLed,!rgb[blueLed]);

      _time_get_elapsed(&last_bat_read);
    }

}

/*TASK*-------------------------------------------------------------------
 *
 * Task Name : hub_main_loop
 *
 *END*----------------------------------------------------------------------*/
void hub_main_loop(uint32_t dummy) {
	A_INT32 		return_code, timeout;
	A_UINT8 		i;
	MQX_FILE_PTR 	pins_fd;
	GPIO_PIN_STRUCT pins[2];
	err_t			ret;
	uint32_t		ret32;
	uint8_t			cbuf_full = FALSE;
	uint8_t			sat_idx;
	static uint16_t cnt_print = 1;//todo remove me

	printf("Hi! Welcome to hub main loop%s\n", ENABLE_MAG ? "! (MAG enabled)" : "!");

	_int_install_unexpected_isr();

	//assert_nordic_reset();
	init_nordic();

	for (sat_idx = 0; sat_idx < MAX_SENSORS; sat_idx++) {
          cbufInit(&cbuf[sat_idx]);
	}

	ts_init_LED();
	ts_turn_on_LED(greenLed);
        
        /* Flash operation and Initialize state to Nordic */
        //initialize_ids();
        hub_flash_init();

        readHubFlash();
        sendHubFlashInfoToNordic();

        /* Set Hub SSID */
#if PIVOT_3_0
        sprintf(&ssid_name[0], "PIVOT_3_0_%u",cur_hub_id);
#else
        sprintf(&ssid_name[0], "PIVOT_%u",cur_hub_id);
#endif    
        // Read HUB Connection Type (If Wifi Direct, port==0)
        hubType_comPort = readBlockFromFlash(HUB_TYP_OFF, HUB_TYPE_DIRECT);
        
        hubType_IPAdress = readBlockFromFlash(HUB_IPA_OFF, HUB_TYPE_DIRECT);
        
	/* Other init */
#if ENABLE_WIFI
	set_hub_not_active(&current_state);
#else
	set_hub_active(&current_state);
#endif

	while (1) {
                for (uint8_t qq = 0; qq < nordic_repeat ; qq++) {
                  talk_to_nordic(&nordic_to_hub);
                  app_time_delay2(16);
                }
                update_battery_led();

		_lwevent_set(&atheros_task_event, 0x01);
		_sched_yield();

#if DEBUG_DELAY
		app_time_delay(3000); /* Wait in millisecond */
#endif
	}

	SHUTDOWN: hub_quit_loop();

	return;
}

