/*******************************************************************************
 * Standard C Include Files
 ******************************************************************************/
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * SDK Include Files
 ******************************************************************************/
#include <mqx.h>
#include "atheros_stack_offload.h"
#include "dtm_common_types.h" /* Turing Sense specific */
#include "hub_main_loop_dtm.h"
#include "main.h"
#include "throughput.h"
#include "wmiconfig_ts.h"
#include "dtmTestutilities.h"
#include "common_types.h"
#include "hub_to_nordic_dtm.h"
#include "virtual_com.h"

#include <a_config.h>
#include <a_types.h>
#include <a_osapi.h>
#include <driver_cxt.h>
#include <custom_api.h>
#include <common_api.h>
#include <wmi_api.h>
//#include <wmi_host.h>
#include <targaddrs.h>
#include <spi_hcd_if.h>
#include "AR6002/hw2.0/hw/mbox_host_reg.h"
#include <atheros_wifi_api.h>

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
static hub_to_nordic_dtm_t spi_tx;
static nordic_to_hub_dtm_t spi_rx;
static uint8_t spi_tx_arr[SPI_COMM_LENGTH] = {0};
static uint8_t spi_rx_arr[SPI_COMM_LENGTH] = {0};

/* SPI related */
typedef union {
	sat_mosi_packet_t packet;
	uint8_t bytes[/*sizeof(sat_mosi_packet_t)*/SPI_COMM_LENGTH];
} receivedCommand_t;
typedef union dataToSend_ {
	sat_miso_packet_t data;
	uint8_t bytes[sizeof(sat_miso_packet_t)];
} dataToSend_t;

dataToSend_t dataToSend;

static uint32_t sRandomSeed;
uint16_t user_input;

void initRandom()
{
	sRandomSeed = 0x02;
}


uint32_t getRandom(uint8_t is_prbs9)
{
	uint8_t a = (is_prbs9 ? 6 : 14);
	uint32_t new_v = (((sRandomSeed < a) ^ (sRandomSeed >> (a-1))) & 0x01);
	sRandomSeed = ((sRandomSeed << 16) | new_v) | sRandomSeed >> 16 ;
	return sRandomSeed;
}

int atherosSpiTest(parameters_t parameters) {
  
    A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(p_Global_Cxt);
    A_NETBUF_DECLARE req;
    A_VOID *pReq = (A_VOID*)&req;  
    A_UINT32 mac_word;
    A_UINT8 *ptr_mac_word = (A_UINT8*)&mac_word;
    A_STATUS status = A_ERROR;

    A_NETBUF_CONFIGURE(pReq, &mac_word, 0, sizeof(A_UINT32), sizeof(A_UINT32));
    uint32_t nIter = parameters.spiSensors_numberOfTests;
    uint32_t nGood = 0;
    uint32_t tmpRandVar = 0;
    
    initRandom(); 
    
    debug_printf("\r\n************ RESULTS SPI TO ATHEROS TESTING ************");

     // Loopback test
    for (uint32_t ii = 0; ii < nIter; ii++) {
      
      // Write a random variable
      sRandomSeed = _time_get_nanoseconds();

      tmpRandVar = getRandom(false);
      mac_word = tmpRandVar;
      ATH_SET_PIO_EXTERNAL_WRITE_OPERATION(pReq, SCRATCH_ADDRESS, A_TRUE, sizeof(A_UINT32));

      if(A_OK != (status = Hcd_DoPioExternalAccess(p_Global_Cxt, pReq))){
        debug_printf("\r\nFAIL writing to address 0x%x on Atheros! Going into ASSERT!\n", \
          SCRATCH_ADDRESS);
        A_ASSERT(0);
      }
    
      // clear the container variable
      mac_word = 0;
    
      // Read from the scratch register
      ATH_SET_PIO_EXTERNAL_READ_OPERATION(pReq, SCRATCH_ADDRESS, A_TRUE, sizeof(A_UINT32));
      
      if(A_OK != (status = Hcd_DoPioExternalAccess(p_Global_Cxt, pReq))){
        debug_printf("\r\nFAIL reading from address 0x%x on Atheros! Going into ASSERT\n", \
          SCRATCH_ADDRESS);
        A_ASSERT(0);
      }   
       
      // check the result
      if (mac_word == tmpRandVar) {
        nGood++;
      }
    }
    
    
    if(parameters.log_enableDetail>0) 
      debug_printf("\r\nAtheros SPI test, Good transmissions = %d",nGood);
    
    if (parameters.spiSensors_numberOfTests - nGood <= parameters.spiSensor_max_WR_fail)
      debug_printf("\r\n[SPI2AT:PASSED]");
    else
      debug_printf("\r\n[SPI2AT:FAILED]");
    
    return A_OK;
}

// ************************************************************ //
// ***************** TEST SPI COMMUNICATION ******************* //
int spiTest(parameters_t parameters){
  
    uint32_t results[NR_STATISTIC_ELEMENTS]={0};
    bool is_sleeping = false, retbool;
    dspi_status_t dspiResult;
 
    static uint16_t last_crc16_sent = 0;
    uint16_t generated_crc16, temp_crc16;
    bool need_spi_resend = false, first=true;
 
    //dspi_status_t Error;
    uint32_t nBusyState = 0;
    
    results[POS_NR_WRREADCICLE]=parameters.spi_numberOfTests;

    		
     assert_nordic_reset();
     init_nordic();
     app_time_delay(100);
                
             
     debug_printf("\r\n ************ RESULTS SPI 22f TO NORDIC TESTING ************");
	do {
                
                initRandom(); 
		/* Generate CRC16 */
                sRandomSeed = _time_get_nanoseconds();
                results[POS_NR_ATTEMPTCICL]=0;
                
		spi_tx.randomFrom22F=getRandom(false);//getRandom(true); //generate random number
                spi_tx.crc16 = 0;
                generated_crc16 = crc16_compute((uint8_t*)&spi_tx, sizeof(spi_tx), 0);
		spi_tx.crc16 = last_crc16_sent = generated_crc16;

		results[POS_NR_PACKAGESENT]++;

		bool firstCicle=true;
		
                
              do {
		// ************* TRASFER DATA WITH SPI **************************** //
		// **************************************************************** //
                nBusyState = talk_to_nordic_dtm(spi_tx, &spi_rx);
                
                //nBusyState = talk_to_nordic_dtm(spi_tx, &spi_rx);
                results[POS_NR_SPBUSYSTATE]+= nBusyState;  
                results[POS_NR_ITERACTIONS]++;
		// **************************************************************** //
		// **************************************************************** //
		//results initialization
                if(!firstCicle)
                  results[POS_NR_ATTEMPTCICL]++;
                
                temp_crc16 = spi_rx.crc16;
		spi_rx.crc16 = 0;
                generated_crc16 = crc16_compute ((uint8_t*) &spi_rx, sizeof(spi_rx), 0);
			
                if (generated_crc16 == temp_crc16) {
                  // debug_printf("\r\nGot clean crc16 from Nordic DEBUG: 0x%x\n", generated_crc16rated_crc16);
                  
                   // Check last CRC16 received by Nordic 
                   if (last_crc16_sent != spi_rx.last_crc16_recvd) {
			if(!firstCicle)
                          results[POS_NR_CRC16LASTFL_TOT]++;

			if(parameters.log_enableDetail>1)
                          if(!firstCicle)
                            debug_printf("\r\n LAST CRC CK FAILED value=%d RECEIVED value=%d",last_crc16_sent,spi_rx.last_crc16_recvd);
                        
                        need_spi_resend = true;
                    }
                    else if(spi_tx.randomFrom22F == spi_rx.randomFrom22F)
                    {
                        // debug_printf("\r\nGot Right crc16 from Nordic DEBUG: 0x%x Exiting...\n", spi_rx.last_crc16_recvd);
			// debug_printf("\r\nCommand has been successfully requested to the RF Chip. RF chip will continue transmitting unless you enter new command...\n");
			//if(parameters.log_enableDetail>0)
                          //  debug_printf("Sent %d - Rcvd %d \r\n",spi_tx.randomFrom22F,spi_rx.randomFrom22F);
                              need_spi_resend = false;

                              results[POS_NR_ITERACTIONS_TOT] += results[POS_NR_ITERACTIONS];
                              results[POS_NR_SPBUSYSTATE_TOT] += results[POS_NR_SPBUSYSTATE];
                              results[POS_NR_PACKAGERECE]++;
                              results[POS_NR_ITERACTIONS]=0;
                              results[POS_NR_SPBUSYSTATE]=0;
				
                              if(parameters.log_enableDetail>1)
				debug_printf("\r\n SENT value=%d RECEIVED value=%d [%d|%d]",spi_tx.randomFrom22F,spi_rx.randomFrom22F,results[POS_NR_ITERACTIONS],results[POS_NR_SPBUSYSTATE]);

                    }
                    else 
			results[POS_NR_ERRORTRANSM]++;
                    
		}  
                else 
                     if(results[POS_NR_ATTEMPTCICL] > parameters.spi_attemptLimit){
                          results[POS_NR_PACKAGELOST]++;
                          break;
                      }
                      else {
                          //debug_printf("\r\nGot bad crc16 from Nordic DEBUG: 0x%x Expected:0x%x\n", temp_crc16, generated_crc16);
                          results[POS_NR_CRC16FAILED]++;
                          need_spi_resend = true;
                      }

                //debug_printf("\r\nGot bad crc16 from Nordic DEBUG: 0x%x Expected:0x%x\n", temp_crc16, generated_crc16);
                
		if (!need_spi_resend) {
			break;
		}

		firstCicle=false;
                
              } while(1);
		parameters.spi_numberOfTests--;
                //debug_printf("%u\r\n",parameters.spi_numberOfTests);
	} while(parameters.spi_numberOfTests>0);
        
	if(parameters.log_enableDetail>0){
		debug_printf("\r\n ************ RESULTS SPI TESTING ************");
		debug_printf("\r\n * Number of packets sent: =%d", results[POS_NR_PACKAGESENT] );
		debug_printf("\r\n * Number of packets received =%d", results[POS_NR_PACKAGERECE] );
		debug_printf("\r\n * Number of lost packets: =%d", results[POS_NR_PACKAGELOST] );
		debug_printf("\r\n * Number of CRC failed =%d", results[POS_NR_CRC16FAILED] );
		debug_printf("\r\n * Number of LAST CRC failed =%d", results[POS_NR_CRC16LASTFL_TOT] );
		debug_printf("\r\n * Number of compromise package: =%d", results[POS_NR_ERRORTRANSM] );
		debug_printf("\r\n * Average number attempts per shipment (consider 2 cycle for packet): =%d", (uint32_t)((float)results[POS_NR_ITERACTIONS_TOT]/(float)results[POS_NR_WRREADCICLE]));
		debug_printf("\r\n * Average busy state per shipment: =%d", (uint32_t)((float)results[POS_NR_SPBUSYSTATE_TOT]/(float)results[POS_NR_WRREADCICLE]));
	}
        
	if(results[POS_NR_CRC16FAILED]>parameters.spi_max_crc_fail)
		debug_printf("\r\n[SPI2TN:FAILED]");
	else
		debug_printf("\r\n[SPI2TN:OK]");
        
  return 0;
}

// ************************************************************ //
// ************************* OPTIONS ************************** //
void setNordicParameters()
{
            /******************* Setting Nordic DTM *****************************/
              debug_printf("\r\nDTM frequency in MHz (0-83): ");
              debug_scanf("%u", &user_input);

              if (user_input > DTM_FREQ_MAX) {
                      spi_tx.dtm_tx_freq = DTM_FREQ_MAX;
                      debug_printf("\r\nYou entered %u, defaulting to %u", user_input, spi_tx.dtm_tx_freq);
              } else {
                      spi_tx.dtm_tx_freq = user_input;
                      debug_printf("\r\n%u", spi_tx.dtm_tx_freq);
              }

              debug_printf("\r\nDTM packet length in bytes (1-252): ");
              debug_scanf("%u", &user_input);

              if (user_input > DTM_PKT_LEN_MAX) {
                      spi_tx.dtm_tx_pkt_len = DTM_PKT_LEN_MAX;
                      debug_printf("\r\nYou entered %u, defaulting to %u", user_input, spi_tx.dtm_tx_pkt_len);
              } else if (user_input < DTM_PKT_LEN_MIN) {
                      spi_tx.dtm_tx_pkt_len = DTM_PKT_LEN_MIN;
                      debug_printf("\r\nYou entered %u, defaulting to %u", user_input, spi_tx.dtm_tx_pkt_len);
              } else {
                      spi_tx.dtm_tx_pkt_len = user_input;
                      debug_printf("\r\n%u", spi_tx.dtm_tx_pkt_len);
              }

              debug_printf("\r\nDTM transmit interval in ms (0 to 65535): ");
              debug_scanf("%u", &user_input);

              if (user_input > UINT16_MAX) {
                      spi_tx.tx_interval = UINT16_MAX;
                      debug_printf("\r\nYou entered %u, defaulting to %u", user_input, spi_tx.dtm_tx_pkt_len);
              } else {
                      spi_tx.tx_interval = user_input;
                      debug_printf("\r\n%u", spi_tx.tx_interval);
              }

              debug_printf("\r\nTransmit power (dBm) "
                                      "\r\n 0: +4dBm   DEFAULT"
                                      "\r\n 1: 0dBm"
                                      "\r\n 2: -4dBm"
                                      "\r\n 3: -8dBm"
                                      "\r\n 4: -12dBm"
                                      "\r\n 5: -16dBm"
                                      "\r\n 6: -20dBm"
                                      "\r\n 7: -30dBm"
                                      "\r\nDTM power: ");
              debug_scanf("%u", &user_input);

              if (user_input > DTM_ENUM_POWER_MAX) {
                      spi_tx.dtm_power = DTM_ENUM_POWER_MAX;
                      debug_printf("\r\nYou entered %u, defaulting to %u", user_input, spi_tx.dtm_power);
              } else {
                      spi_tx.dtm_power = user_input;
                      debug_printf("\r\n%u", spi_tx.dtm_power);
              }

              debug_printf("\r\nPacket Type (0-3) "
                                      "\r\n 0: prbs9"
                                      "\r\n 1: prbs15"
                                      "\r\n 2: 11110000"
                                      "\r\n 3: 01010101"
                                      "\r\n 4: unmodulated"
                                      "\r\nPacket Type: ");
              debug_scanf("%u", &user_input);

              if (user_input > DTM_PKT_TYPE_MAX) {
                      spi_tx.dtm_pkt_type = DTM_PKT_TYPE_MAX;
                      debug_printf("\r\nYou entered %u, defaulting to %u", user_input, spi_tx.dtm_pkt_type);
              } else {
                      spi_tx.dtm_pkt_type = user_input;
                      debug_printf("\r\n%u", spi_tx.dtm_pkt_type);
              }

}


// ************************************************************ //
// ************** TEST SPIsensor COMMUNICATION **************** //


static uint8_t mpu9250_read_reg(uint8_t reg_addr)
{
	uint8_t data;

	mpu9250_read_regs(reg_addr, 1, & data);
	return data;
}

void showResultTag(){
	debug_printf("\r\nOUTPUT CODES **************************");
	debug_printf("\r\nSENSCR: sensor screening test");
	debug_printf("\r\nSPI2TN: 22F<>Nordic SPI test");
        debug_printf("\r\nSPI2AT: 22F<>Atheros SPI test");
	debug_printf("\r\nSPI2TI: 22F<>InverSense SPI test");
	debug_printf("\r\nSATIDX: calculated SAT ID");

}


