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

#include "hub_to_nordic.h"
#include "hub_main_loop.h"


#define 					PRINT_DEBUG		1

/* SPI 0 */
uint32_t 					nordic_instance = 0;
dspi_device_t 				spi0_device;
dspi_master_state_t 		spi0_master_state;
dspi_master_state_t 		*spi0_master_st_p = &spi0_master_state;
dspi_master_user_config_t 	spi0_m_user_cfg;

typedef struct padded_nord {
	uint32_t 		buf[1];
	nordic_to_hub_t packet;
} padded_nordic_to_hub_t;

padded_nordic_to_hub_t	padded_rx;
static nordic_to_hub_t 	rx_from_nordic;

typedef struct quaternion_ {
	uint32_t timestamp;
	float quat_w;		// Quarternion's scalar component, rotation degree around the vector
	float quat_x;		// Quarternion's x vector component
	float quat_y;		// Quarternion's y vector component
	float quat_z;		// Quarternion's z vector component
} quaternion_t;
static void print_quat_data(quaternion_t *quat_data);

//static void enable_nordic_spi0cs1(void) {
//    PORT_MemMapPtr  pctl;
//    pctl = (PORT_MemMapPtr)PORTD_BASE_PTR;
//	pctl->PCR[4] = PORT_PCR_MUX(2);     				/* PTD4, alt2, DSPI0.PCS1   */
//}
//
//static void enable_ath_spi1cs0(void) {
//    PORT_MemMapPtr  pctl;
//    pctl = (PORT_MemMapPtr)PORTD_BASE_PTR;
//    pctl->PCR[4] = PORT_PCR_MUX(7) | PORT_PCR_DSE_MASK;     /* DSPI1.PCS0   */
//}

/*
 * Function: cbufFindTimestamp
 * Comments: Locally modified cbuf find.  Will return index to cbuf element that has it.
 *
 */
err_t cbufFindTimestamp(cbuf_t *cb, uint32_t keyval, uint32_t *index_found)
{
	if (cb == NULL) {
		return E_NO_RESOURCE;
	}
   /* Exit if queue is empty */
   if(cbufIsEmpty(cb) == TRUE)
   {
   	return E_NOT_FOUND;
   }

   /* traverse from head to tail to look for the value */
   uint32_t index = cb->head;
   do {
	 if (cb->A[index].timestamp == keyval)
   	 {
	   /* If found match, return the index */
	   *index_found = index;
   	   return E_OK;
   	 }
   	 index = (index + 1) % CB_SIZE;  /* wrap the queue */
   } while(index != cb->tail);

   return E_NOT_FOUND;  /*Keyval is not present in the array */
}

/*
 * Function: 	add_rx_from_nordic
 * Parameter:	nordic_to_hub_t
 *
 * Comments: 	For each data received from Nordic, we categorize them into its timestamp.
 *
 * cwati TODO bucket into 10ms window??
 *
 */
err_t add_rx_from_nordic(nordic_to_hub_t rx_from_nordic) {
	err_t 		ret = E_OK;
	uint8_t		idx, sat_idx = 0xff;
	uint32_t	cbuf_idx;
	CB_TYPE		new_cbuf_entry;

	/* First find the index of this satellite ID */
	for (idx = 0; idx < cur_num_of_sat; idx++) {
		if (cur_sat_ids[idx] == rx_from_nordic.satellite_id) {
			sat_idx = idx;
			break;
		} else {
			return E_NOT_FOUND;
		}
	}

	/* Error check.  Should never arrive here. */
	if (sat_idx > (MAX_SENSORS - 1)) {
		return E_UNKNOWN;
	}

	ret = cbufFindTimestamp(&cbuf, rx_from_nordic.timestamp, &cbuf_idx);
	if (ret == E_OK) {
		cbuf.A[cbuf_idx].bitmap |= (1 << sat_idx);
		cbuf.A[cbuf_idx].data[sat_idx].satellite_id = rx_from_nordic.satellite_id;
		cbuf.A[cbuf_idx].data[sat_idx].record = rx_from_nordic.data;
	} else {
		if (cbufIsFull(&cbuf)) {
			return E_NO_RESOURCE;
		} else {
			new_cbuf_entry.bitmap = (1 << sat_idx);
			new_cbuf_entry.timestamp = rx_from_nordic.timestamp;
			new_cbuf_entry.data[sat_idx].satellite_id = rx_from_nordic.satellite_id;
			new_cbuf_entry.data[sat_idx].record = rx_from_nordic.data;

			ret = cbufPush(&cbuf, new_cbuf_entry);
		}
	}

#if PRINT_DEBUG
	("cbuf now has %d elements\n", cbufNum(&cbuf));
#endif

	return ret;
}


uint32_t talk_to_nordic(nordic_to_hub_t *data)
{
	uint8_t mts_buffer[2], stm_buffer[2];
	dspi_status_t 			Error;
	quaternion_t			quat, *quat_data = &quat;
	err_t					ret;

	/* If cbuf is nearly full, set wait */
	if (cbufNum(&cbuf) == (CB_SIZE - 2)) {
		/* Tell Nordic to stop sending, hub is full. */
		hub_to_nordic.command |= NORDIC_WAIT;
	} else {
		hub_to_nordic.command &= ~NORDIC_WAIT;
	}

#if PRINT_DEBUG
	printf("\r\n\r\n\r\nTransmitting to Nordic:");
	printf( "\r\ncommand: 		0x%02x"
			"\r\nRTC: 			0x%02x",
			hub_to_nordic.command, hub_to_nordic.payload.rtc_value);
	printf("\n");
#endif

	/* Read and parse from Nordic */
	Error = DSPI_DRV_MasterTransferDataBlocking(nordic_instance, &spi0_device,
			(uint8_t *) &hub_to_nordic, (uint8_t *) &padded_rx,
			sizeof(padded_rx), 1);

	/* There's an issue with SPI communication from hub to Nordic via MQX SPI.
	 * The first byte from Nordic is always lost.  To compensate for that we
	 * send additional 4 bytes from Nordic and then 1 byte will be lost, and
	 * we manually throw away the remaining 3 bytes.
	 */
	uint8_t *ptr = (uint8_t *) &padded_rx;
	ptr += 3;

	memcpy (&rx_from_nordic, ptr, sizeof(nordic_to_hub_t));

#if PRINT_DEBUG
/* For now only get the quaternion "fake"/"simulated" data from Nordic.
*/
	printf("Receiving satellite ID: 0x%x\n",rx_from_nordic.satellite_id);
	quat_data->timestamp	= rx_from_nordic.timestamp;
	quat_data->quat_w		= rx_from_nordic.data.quat_w;
	quat_data->quat_x		= rx_from_nordic.data.quat_x;
	quat_data->quat_y		= rx_from_nordic.data.quat_y;
	quat_data->quat_z		= rx_from_nordic.data.quat_z;

	print_quat_data(quat_data);
#endif /* PRINT_DEBUG */

	/* Add data to cbuf */
	ret = add_rx_from_nordic(rx_from_nordic);
	if (ret != E_OK) {
		printf("!!!!ERROR: adding to cbuf!\n");
	}

	/* Clear Nordic command */
	hub_to_nordic.command &= ~(NORDIC_CALIBRATE | NORDIC_SET_RTC | NORDIC_SET_SATELLITES);

	return true;
}

void nordic_hardware_init(void) {
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

/*
 * This is more for diagnostics and to verify that the program has been loaded successfully.
 */
void turn_on_LED(void) {
	/* Green LED on PTA2 */
    GPIO_PIN_STRUCT 	pins[2];
    uint32_t 			read_pin_table[2];
    MQX_FILE_PTR	pins_fd;

    pins[0] = (GPIO_PORT_A|GPIO_PIN2);
    pins[1] = GPIO_LIST_END;

    pins_fd = fopen( "gpio:output",(char *)&pins);

    if(NULL == pins_fd){
        printf("ERROR: Failed to open GPIO char device for GPIO!\n");
    }
    if(ioctl(pins_fd, GPIO_IOCTL_READ, &read_pin_table) == IO_OK)
    {
		if((read_pin_table[0] & GPIO_PIN_STATUS) == GPIO_PIN_STATUS_1)
		{
		/* first pin in the table is set */
		}
    }
    if(ioctl(pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK)
    {
//		printf("OK write log1\n");
    }
    if(ioctl(pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK)
    {
//		printf("OK write log0\n");
    }

}

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
	printf("\r\n");

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

    DSPI_DRV_MasterInit(nordic_instance, spi0_master_st_p, &spi0_m_user_cfg);

    spi0_device.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge; //CPHA=0
    spi0_device.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    spi0_device.dataBusConfig.direction = kDspiMsbFirst;
    spi0_device.bitsPerSec = baudRateHz;
    spi0_device.dataBusConfig.bitsPerFrame = transferSizeBits;
    DSPI_DRV_MasterConfigureBus(nordic_instance, &spi0_device, &baudRate);


    status = DSPI_DRV_MasterSetDelay(nordic_instance,
    								 kDspiPcsToSck,
    								 500, // delayInNanoSec
    								 & calculatedPcsToSck);
	if (status != kStatus_DSPI_Success)
		printf("error %s:%d\n",__FUNCTION__,__LINE__);

    status = DSPI_DRV_MasterSetDelay(nordic_instance,
    								 kDspiLastSckToPcs,
    								 500, // delayInNanoSec
    								 & calculatedLastSckToPcs);
	if (status != kStatus_DSPI_Success)
		printf("error %s:%d\n",__FUNCTION__,__LINE__);

	status = DSPI_DRV_MasterSetDelay(nordic_instance,
    								 kDspiAfterTransfer,
    								 500, // delayInNanoSec
    								 & calculatedAfterTransfer);
	if (status != kStatus_DSPI_Success)
		printf("error %s:%d\n",__FUNCTION__,__LINE__);
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

    /* Initializing pins */
//    _int_install_unexpected_isr();

    nordic_hardware_init();
    turn_on_LED();

#if PRINT_DEBUG
    printf ("\n-------------- Initializing SPI0 to Nordic --------------\n\n");
#endif

    /* Initializing SPI to Nordic */
    init_spi0_nordic();

    hub_to_nordic.command = 0;

#if PRINT_DEBUG
    printf("\r\nFinished initializing Nordic interface\r\n");
#endif

}
