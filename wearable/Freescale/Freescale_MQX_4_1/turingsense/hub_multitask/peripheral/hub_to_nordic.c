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

#include "atheros_main_loop.h"
#include "crc16.h"
#include "hub_to_nordic.h"
#include "hub_main_loop.h"

#define 					PRINT_DEBUG	0
#define 					PRINT_DEBUG1	0
#define 					CWATI_TMP   0

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

uint32_t reset_ts_rtc0[MAX_SENSORS] = {0};			/* Reset timestamp after RTC is reset to 0 */
uint32_t last_seen_timestamp[MAX_SENSORS] = {0};
uint32_t last_seen_tsarr[MAX_SENSORS][500] = {0};	//cw debug
uint32_t debug_cnt[MAX_SENSORS] = {0};

int32_t timestamp_adjustment[MAX_SENSORS] = {0};

uint32_t temp_timestamp[MAX_SENSORS] = {0};
//static uint32_t communicationTestCounter_fromNordic[MAX_SENSORS]={0}; //CF: communication analysis
//static uint32_t communicationTestCounter_toClient[MAX_SENSORS]={0}; //CF: communication analysis
//static uint32_t communicationTestCounter_lastSAT22F[MAX_SENSORS]={0}; //CF: communication analysis

padded_nordic_to_hub_t	padded_rx;
static nordic_to_hub_t 	rx_from_nordic;

static uint32_t	nordic_reinit = 0;
const uint32_t nordic_reinit_max = 1000; /* Make it longer to wait for Nordic bootloader to initialize */

typedef struct quaternion_ {
	uint32_t timestamp;
	float quat_w;		// Quarternion's scalar component, rotation degree around the vector
	float quat_x;		// Quarternion's x vector component
	float quat_y;		// Quarternion's y vector component
	float quat_z;		// Quarternion's z vector component
} quaternion_t;
static void print_quat_data(quaternion_t *quat_data);

/*
 * Adding sensor record into the cbuf
 */
err_t add_rx_per_sensor_record(sat_sensor_record_t rx_from_nordic) {
	err_t 		ret = E_OK;
	uint8_t		idx, sensor_idx = 0xff;
	CB_TYPE		new_cbuf_entry;
        static uint32_t discarded[MAX_SENSORS] = {0};
        
        if (rx_from_nordic.sensor_idx > (MAX_SENSORS - 1)) {
          return E_INVALID_DATA;
        }
        
	/* First find the index of this satellite ID */
	sensor_idx = rx_from_nordic.sensor_idx;

	/* If cbuf is full, discard older data */
	if (cbufIsFull(&cbuf[sensor_idx])) {
		cbufPopDiscard(&cbuf[sensor_idx]);
                discarded[sensor_idx]++;
	}

	new_cbuf_entry.timestamp = rx_from_nordic.timestamp;
	new_cbuf_entry.record = rx_from_nordic.data;
	_mutex_lock(&cbuf_mutex);
	ret = cbufPush(&cbuf[sensor_idx], new_cbuf_entry);
	_mutex_unlock(&cbuf_mutex);

	if (ret != E_OK) {
#if PRINT_DEBUG
		printf("!!!!ERROR: adding to cbuf!\n");
#endif
		}


#if PRINT_DEBUG
	printf("cbuf[%d] now has %d elements\n", sat_idx, cbufNum(&cbuf[idx]));
#endif

	return ret;
}
/*
 * Function: 	add_rx_from_nordic
 * Parameter:	nordic_to_hub_t
 *
 * Comments: 	For each data received from Nordic, we categorize them into its satellite ID.
 *
 *
 */
err_t add_rx_from_nordic(nordic_to_hub_t rx_data) {
	err_t 		ret = E_OK;
	uint8_t		cnt;

#if DEBUG_DIAG
	frame_rcvd = 0;
#endif

	for (cnt = 0; cnt < MAX_SENSOR_RECORDS_PER_PACKET; cnt++) {

		if(rx_data.record[cnt].sensor_idx > (MAX_SENSORS - 1)) {
			continue;
		}
#if PRINT_DEBUG
		quaternion_t			quat, *quat_data = &quat;
	/* For now only get the quaternion "fake"/"simulated" data from Nordic.
	*/
		printf("\r\nReceiving satellite ID%d: 0x%x",cnt, rx_data.record[cnt].satellite_id);
		quat_data->timestamp	= rx_data.record[cnt].timestamp;
		quat_data->quat_w		= rx_data.record[cnt].data.quat_w;
		quat_data->quat_x		= rx_data.record[cnt].data.quat_x;
		quat_data->quat_y		= rx_data.record[cnt].data.quat_y;
		quat_data->quat_z		= rx_data.record[cnt].data.quat_z;

		print_quat_data(quat_data);
#endif /* PRINT_DEBUG */

		if (rx_data.record[cnt].timestamp != INVALID_TIMESTAMP) {

#if DEBUG_DIAG
			frame_rcvd++;
#endif

#if PRINT_DEBUG
			printf("adding record cnt %d\n", cnt);
#endif

			ret = add_rx_per_sensor_record(rx_data.record[cnt]);

			if (ret != E_OK) {
				//todo ERROR processing
			}
		}
	}

	return ret;
}

static uint32_t spi_checksum_valid() {
  uint16_t          expected_crc16;
  static uint32_t	total_err = 0, total_spi_transaction = 0;

  total_spi_transaction++;

  uint32_t test = sizeof (padded_rx.packet);
  uint32_t test1 = sizeof(sensor_record_t);
  uint32_t test2 = sizeof(hub_spi_data_packet_t);
  
  expected_crc16 = crc16_compute((uint8_t*)(&padded_rx.packet),sizeof(padded_rx.packet),0);
  /* cwati 12/17/15 an issue in SPI from Nordic hub to 22F hub where the first byte receives always
   * has bit 7 set. TODO fixme */
  expected_crc16 |= 0x80;

  if (padded_rx.crc16 != expected_crc16) {
    total_err++;
    return false;
  }

  return true;
}

/* Returns true if no sensor info in this packet */
static bool packet_is_empty(nordic_to_hub_t packet) {
	for (uint8_t i = 0; i < MAX_SENSOR_RECORDS_PER_PACKET; i++) {
		if (packet.record[i].timestamp != INVALID_TIMESTAMP) {
			return false;
		}
	}

	return true;
}

uint32_t talk_to_nordic(nordic_to_hub_t *data)
{
	uint8_t mts_buffer[2], stm_buffer[2];
	dspi_status_t 			Error;
	err_t					ret;
        static uint32_t        no_pkt_cnt = 0;
        const uint32_t         max_no_pkt_cnt = 5000;
        
#if PRINT_DEBUG
	printf("\r\n\r\n\r\nTransmitting to Nordic:");
	printf( "\r\ncommand: 		0x%02x"
			"\r\nRTC: 			0x%02x",
			hub_to_nordic.command, hub_to_nordic.payload.rtc_value);
	printf("\n");

#endif

	memset(&padded_rx, 0, sizeof(padded_rx));

        hub_to_nordic.crc = 0;
        hub_to_nordic.crc = crc16_compute((uint8_t*)&hub_to_nordic, sizeof(hub_to_nordic), 0);

	/* Read and parse from Nordic */
        _task_stop_preemption();
#if EVAL_BOARD
	Error = DSPI_DRV_MasterTransferBlocking(nordic_spi_instance, &spi0_device,
#elif PRODUCTION1
	Error = DSPI_DRV_MasterTransferBlocking(nordic_spi_instance, &spi1_device,
#endif
			(uint8_t *) &hub_to_nordic, (uint8_t *) &padded_rx,
			sizeof(padded_rx), 1);
        _task_start_preemption();
        
        if (Error == kStatus_DSPI_Success) {
            /* Clear Nordic command */
            hub_to_nordic.payload.satellite_ids[SAT_COMMSTRUCT_COMMA] = SAT_COMM_IDLE_CYCLE;
        }
        
	if ((spi_checksum_valid() == FALSE) || (Error != kStatus_DSPI_Success)) {
		nordic_reinit++;

		/* After asserting Nordic, we should resend satellite list to Nordic */
		if (nordic_reinit % nordic_reinit_max == 0) {
                        reinit_nordic();
			sendHubFlashInfoToNordic();
		}
		return 1;
	}

	/* If state is active/sending, but we don't get any packet, keep track of it.
	 */
	if (hub_active(current_state)) {

          if (packet_is_empty(padded_rx.packet)) {
		/* Although packet is empty, we still want to retrieve the ack */
		if (padded_rx.packet.ack & HUB_RTC_ACK) {
			ack_to_client |= HUB_RTC_ACK;
		}

		no_pkt_cnt++;
		if (no_pkt_cnt % max_no_pkt_cnt == 0) {
			assert_nordic_reset();
			reinit_nordic();
			/* After asserting Nordic, we should resend satellite list to Nordic */
			sendHubFlashInfoToNordic();
		}
		return 1;
          } else {
            /* After receiving packet, reset this count. */
            no_pkt_cnt = 0;
          }
	}


	if (padded_rx.packet.ack & HUB_RTC_ACK) {
                _mutex_lock(&ack_to_client_mutex);
		ack_to_client |= HUB_RTC_ACK;
                _mutex_unlock(&ack_to_client_mutex);
	}

	/* There's an issue with SPI communication from hub to Nordic via MQX SPI.
	 * The first byte from Nordic is always lost.  To compensate for that we
	 * send additional 4 bytes from Nordic and then 1 byte will be lost, and
	 * we manually throw away the remaining 3 bytes.
	 */
	rx_from_nordic = padded_rx.packet;

	/* Add data to cbuf */
        for(int i = 0;i<MAX_SENSOR_RECORDS_PER_PACKET;i++){//cf: packet counter
          int16_t tmp = rx_from_nordic.record[i].data.quat_w/CPI_QUATERNION16_BIAS_MULTIPLIER;
          if(tmp==CPI_QUATERNION16_COUNTER_COMMAND){
                  if(communicationTestCounter_1[rx_from_nordic.record[i].sensor_idx]<rx_from_nordic.record[i].data.accel_x){
                    communicationTestCounter_1[rx_from_nordic.record[i].sensor_idx]=rx_from_nordic.record[i].data.accel_x;
                    communicationTestCounter_fromNordic[rx_from_nordic.record[i].sensor_idx]++;
                  }else if(rx_from_nordic.record[i].data.accel_x <= 0){
                  }else if(communicationTestCounter_1[rx_from_nordic.record[i].sensor_idx]>rx_from_nordic.record[i].data.accel_x){
                    communicationTestCounter_1[rx_from_nordic.record[i].sensor_idx] = rx_from_nordic.record[i].data.accel_x;
                  }
#if ENABLE_MAG
                  rx_from_nordic.record[i].data.mag_x = (int16_t)(communicationTestCounter_fromNordic[rx_from_nordic.record[i].sensor_idx]/CPI_QUATERNION16_COUNTER_MAXVALU);
                  rx_from_nordic.record[i].data.mag_y = (int16_t)(communicationTestCounter_fromNordic[rx_from_nordic.record[i].sensor_idx]%CPI_QUATERNION16_COUNTER_MAXVALU);
                  rx_from_nordic.record[i].data.quat_x = communicationTestCounter_fromNordic[rx_from_nordic.record[i].sensor_idx];
#endif
          }
         }
        
#if FAST_DRAIN
        /* If it's not NOT_ACTIVE, do not push the sensor data */
        if (hub_active(current_state)) {
          ret = add_rx_from_nordic(rx_from_nordic);
        }
#else
	ret = add_rx_from_nordic(rx_from_nordic);
#endif
	return true;
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

static void put_float(float f)
{
	int32_t q;

	q = (int32_t) (f * (1<<30));  // Python demo uses fixed point +-1.30 bits

	putchar((q >> 24) & 0xff);
	putchar((q >> 16) & 0xff);
	putchar((q >> 8)  & 0xff);
	putchar(q & 0xff);
}

static void print_quat_data(quaternion_t *quat_data)
{
	printf("\r\nTime 0x%x  \n", quat_data->timestamp);
	printf("W: %f\n",quat_data->quat_w);
	printf("X: %f\n",quat_data->quat_x);
	printf("Y: %f\n",quat_data->quat_y);
	printf("Z: %f\n",quat_data->quat_z);
//	printf("\r\n");

}
static void output_quaternion_packet_local(quaternion_t quat_data)
{
	putchar('$');
	putchar(0x02);
	putchar(0x00);
	put_float(quat_data.quat_w);
	put_float(quat_data.quat_x);
	put_float(quat_data.quat_y);
	put_float(quat_data.quat_z);
	putchar(0x00);
	putchar(0x00);
	putchar('\r');
	putchar('\n');
}

#if (HUB_22F && EVAL_BOARD)
static void init_spi0_nordic(void)
{
    uint32_t		baudRateHz = 1000000;
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
	quaternion_t 			quat_data;

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
//#if (HUB_22F && EVAL_BOARD)
//    init_spi0_nordic();
//#elif ((HUB_22F && PRODUCTION1) || (SAT_22F && PRODUCTION1))
//    init_spi1_nordic();
//#endif
}
