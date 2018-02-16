/*HEADER**********************************************************************
*
* Copyright 2015 Turingsense Inc
*
* cwati
*
* Comments:
* This code is for hub code.
* It aims to talk to Nordic and send it to Wifi.
*
*END************************************************************************/
#include <mqx.h>
#include <mutex.h>

//#include "atheros_main_loop.h"
#include "crc16.h"
#include "hub_to_nordic_dtm.h"
#include "hub_main_loop_dtm.h"

/* DTM */
#include "dtm_common_types.h"

#define 					PRINT_DEBUG	0
#define 					PRINT_DEBUG1	0
#define 					CWATI_TMP   0

#if DEBUG_DIAG
uint32_t 					cw_ave_pkt_size = 0;
uint32_t 					frame_rcvd, total_nordic_pkt_recv = 0;
#endif

#if (HUB_22F && EVAL_BOARD)
/* SPI 0 */
const uint32_t 					nordic_spi_instance = 0;
dspi_device_t 				        spi0_device;
dspi_master_state_t 		                spi0_master_state;
dspi_master_state_t 		                *spi0_master_st_p = &spi0_master_state;
dspi_master_user_config_t 	                spi0_m_user_cfg;
#elif ((HUB_22F && PRODUCTION1) || (SAT_22F && PRODUCTION1))
/* SPI 1 */
const uint32_t 					nordic_spi_instance = 1;
dspi_device_t 				        spi1_device;
dspi_master_state_t 		                spi1_master_state;
dspi_master_state_t 		                *spi1_master_st_p = &spi1_master_state;
dspi_master_user_config_t 	                spi1_m_user_cfg;
#endif
const uint32_t					ts_diff_cutoff = 10000;	/* If timestamp jumps by this much (absolute value), discard it */

static uint8_t spi_tx_arr[SPI_COMM_LENGTH] = {0};
static uint8_t spi_rx_arr[SPI_COMM_LENGTH] = {0};

uint32_t talk_to_nordic_dtm(hub_to_nordic_dtm_t spi_tx, nordic_to_hub_dtm_t *spi_rx)
{
  dspi_status_t Error;
  uint32_t framesTransferred = 0;
  uint32_t nBusyState = 0;
  

  /* Read and parse from Nordic */
  _task_stop_preemption();
  
  memcpy(&spi_tx_arr, &spi_tx, sizeof(hub_to_nordic_dtm_t));
  
#if EVAL_BOARD
	Error = DSPI_DRV_MasterTransferBlocking(nordic_spi_instance, &spi0_device,
#elif PRODUCTION1
	Error = DSPI_DRV_MasterTransferBlocking(nordic_spi_instance, &spi1_device,
#endif
                  (uint8_t *) &spi_tx_arr, (uint8_t *) &spi_rx_arr,
                  SPI_COMM_LENGTH, 1);
                                                
    while (DSPI_DRV_MasterGetTransferStatus(nordic_spi_instance,&framesTransferred) == kStatus_DSPI_Busy)
    {
       nBusyState++;
       _time_delay(1);
       //debug_printf("\r\n DSPI Busy...");
     }
                  
    memcpy(spi_rx, &spi_rx_arr, sizeof(nordic_to_hub_dtm_t));                                        
                                                
    _task_start_preemption();
                
    return nBusyState;
}
                     
#if (HUB_22F && EVAL_BOARD)
void nordic_hardware_init_spi0(void) {
	/* Enable port clock */
    SIM_MemMapPtr   sim = SIM_BASE_PTR;

    /* Enable clock gate to DSPI0 module */
    sim->SCGC6 |= SIM_SCGC6_SPI0_MASK;		/* Enable SPI0 clock */

    /* Setting GPIO pins for Nordic*/
    PORT_MemMapPtr  pctl;

    /* SPI 0 pins */
    /* PORT_PCR_DSE_MASK is setting for high drive strength.
     * Not using DSE for now */
    pctl = (PORT_MemMapPtr)PORTD_BASE_PTR;
    pctl->PCR[0] = PORT_PCR_MUX(2);     /* PTD0, alt2, DSPI0.PCS0   */
    pctl->PCR[1] = PORT_PCR_MUX(2);     /* PTD1, alt2, DSPI0.SCK    */
    pctl->PCR[2] = PORT_PCR_MUX(2);     /* PTD2, alt2, DSPI0.SOUT   */
    pctl->PCR[3] = PORT_PCR_MUX(2);     /* PTD3, alt2, DSPI0.SIN    */
}
#elif ((HUB_22F && PRODUCTION1) || (SAT_22F && PRODUCTION1))

/* Nordic is on SPI1 on production hub */
void nordic_hardware_init_spi1(void) {
	/* Enable port clock */
    SIM_MemMapPtr   sim = SIM_BASE_PTR;

    /* Enable clock gate to DSPI1 module */
    sim->SCGC6 |= SIM_SCGC6_SPI1_MASK;		/* Enable SPI1 clock */

    /* Setting GPIO pins for Nordic*/
    PORT_MemMapPtr  pctl;

    /* SPI 1 pins */
    /* PORT_PCR_DSE_MASK is setting for high drive strength.
     * Not using DSE for now */
    pctl = (PORT_MemMapPtr)PORTB_BASE_PTR;
    pctl->PCR[10] = PORT_PCR_MUX(2);     /* PTB10, alt2, DSPI1.PCS0   */
    pctl->PCR[11] = PORT_PCR_MUX(2);     /* PTB11, alt2, DSPI1.SCK    */
    pctl->PCR[16] = PORT_PCR_MUX(2);     /* PTB16, alt2, DSPI1.SOUT   */
    pctl->PCR[17] = PORT_PCR_MUX(2);     /* PTB16, alt2, DSPI1.SIN   */
}
#endif /* PRODUCTION1 */


#if (HUB_22F && EVAL_BOARD)
static void init_spi0_nordic(void)
{
    uint32_t		baudRateHz = 4000000;
    uint32_t		transferSizeBits = 8;
    uint32_t 		baudRate;
	dspi_status_t 	status;
	uint32_t 		calculatedPcsToSck, calculatedLastSckToPcs, calculatedAfterTransfer;

    spi0_m_user_cfg.isChipSelectContinuous = true;
    spi0_m_user_cfg.isSckContinuous = false;
    spi0_m_user_cfg.pcsPolarity = kDspiPcs_ActiveLow;
    spi0_m_user_cfg.whichCtar = kDspiCtar0;
    spi0_m_user_cfg.whichPcs = kDspiPcs0; /* Nordic on 22F hub uses SPI0_PCS0 */

    DSPI_DRV_MasterInit(nordic_spi_instance, spi0_master_st_p, &spi0_m_user_cfg);

    spi0_device.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge; //CPHA=0
    spi0_device.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    spi0_device.dataBusConfig.direction = kDspiMsbFirst;
    spi0_device.bitsPerSec = baudRateHz;
    spi0_device.dataBusConfig.bitsPerFrame = transferSizeBits;
    DSPI_DRV_MasterConfigureBus(nordic_spi_instance, &spi0_device, &baudRate);


    status = DSPI_DRV_MasterSetDelay(nordic_spi_instance,
    								 kDspiPcsToSck,
    								 500, // delayInNanoSec
//    								 100, // delayInNanoSec
    								 & calculatedPcsToSck);
	if (status != kStatus_DSPI_Success)
		printf("error %s:%d\n",__FUNCTION__,__LINE__);

    status = DSPI_DRV_MasterSetDelay(nordic_spi_instance,
    								 kDspiLastSckToPcs,
    								 500, // delayInNanoSec
//    								 100, // delayInNanoSec
    								 & calculatedLastSckToPcs);
	if (status != kStatus_DSPI_Success)
		printf("error %s:%d\n",__FUNCTION__,__LINE__);

	status = DSPI_DRV_MasterSetDelay(nordic_spi_instance,
    								 kDspiAfterTransfer,
    								 500, // delayInNanoSec
//    								 100, // delayInNanoSec
    								 & calculatedAfterTransfer);
	if (status != kStatus_DSPI_Success)
		printf("error %s:%d\n",__FUNCTION__,__LINE__);
}
#elif ((HUB_22F && PRODUCTION1) || (SAT_22F && PRODUCTION1))
static void init_spi1_nordic(void)
{
    uint32_t		baudRateHz = 1000000; /* cwati max is 4M per spec */
    uint32_t		transferSizeBits = 8;
    uint32_t 		baudRate;
	dspi_status_t 	status;
	uint32_t 		calculatedPcsToSck, calculatedLastSckToPcs, calculatedAfterTransfer;

    spi1_m_user_cfg.isChipSelectContinuous = true;
    spi1_m_user_cfg.isSckContinuous = false;
    spi1_m_user_cfg.pcsPolarity = kDspiPcs_ActiveLow;
    spi1_m_user_cfg.whichCtar = kDspiCtar0;
    spi1_m_user_cfg.whichPcs = kDspiPcs0; /* Nordic on 22F hub uses spi1_PCS0 */

    DSPI_DRV_MasterInit(nordic_spi_instance, spi1_master_st_p, &spi1_m_user_cfg);

    spi1_device.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge; //CPHA=0
    spi1_device.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    spi1_device.dataBusConfig.direction = kDspiMsbFirst;
    spi1_device.bitsPerSec = baudRateHz;
    spi1_device.dataBusConfig.bitsPerFrame = transferSizeBits;
    DSPI_DRV_MasterConfigureBus(nordic_spi_instance, &spi1_device, &baudRate);


    status = DSPI_DRV_MasterSetDelay(nordic_spi_instance,
    								 kDspiPcsToSck,
    								 500, // delayInNanoSec
//    								 100, // delayInNanoSec
    								 & calculatedPcsToSck);
	if (status != kStatus_DSPI_Success)
		printf("error %s:%d\n",__FUNCTION__,__LINE__);

    status = DSPI_DRV_MasterSetDelay(nordic_spi_instance,
    								 kDspiLastSckToPcs,
    								 500, // delayInNanoSec
//    								 100, // delayInNanoSec
    								 & calculatedLastSckToPcs);
	if (status != kStatus_DSPI_Success)
		printf("error %s:%d\n",__FUNCTION__,__LINE__);

	status = DSPI_DRV_MasterSetDelay(nordic_spi_instance,
    								 kDspiAfterTransfer,
    								 500, // delayInNanoSec
//    								 100, // delayInNanoSec
    								 & calculatedAfterTransfer);
	if (status != kStatus_DSPI_Success)
		printf("error %s:%d\n",__FUNCTION__,__LINE__);
}
#endif

void deinit_nordic (void) {
	DSPI_DRV_MasterDeinit(nordic_spi_instance);
}

/*TASK*-------------------------------------------------------------------
*
* Task Name : init_nordic
* Comments  : Initializing Nordic
*
*END*----------------------------------------------------------------------*/
void init_nordic (void)
{
    MQX_FILE_PTR           spifd;
    uint32_t               param, result, i = 0;

    //assert_nordic_reset();
    /* Initializing pins */

    /* Initializing SPI to Nordic */
#if (HUB_22F && EVAL_BOARD)
    nordic_hardware_init_spi0();
    printf ("\n-------------- Initializing SPI0 to Nordic --------------\n\n");
    init_spi0_nordic();
#elif ((HUB_22F && PRODUCTION1) || (SAT_22F && PRODUCTION1))
    nordic_hardware_init_spi1();
    printf ("\n-------------- Initializing SPI1 to Nordic --------------\n\n");
    init_spi1_nordic();
#endif

#if PRINT_DEBUG
    printf("\r\nFinished initializing Nordic interface\r\n");
#endif

}

/* This reinit_nordic is called when the data received is not as expected */
void reinit_nordic (void) {

  deinit_nordic();
  init_nordic();
}
