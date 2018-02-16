/*HEADER**********************************************************************
*
* Copyright 2015 Turingsense Inc
*
* cwati
*
* Comments:
* This code is for hub code.
* It aims to talk to Nordic and receive data to be output into Python for 3D.
*

*END************************************************************************/


#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <mqx.h>
#include <bsp.h>
#include <spi.h>
//#include "spi_memory.h"

#include "fsl_dspi_master_driver.h"

#include "mpu9250.h"
#include "common_types.h"
#include "ossf/build.h"
#include "ossf/tasks.h"

#define 		PRINT_DEBUG		1
/* SPI 0 */
uint32_t 					nordic_instance = 0;
dspi_device_t 				spi0_device;
dspi_master_state_t 		spi0_master_state;
dspi_master_state_t 		*spi0_master_st_p = &spi0_master_state;
dspi_master_user_config_t 	spi0_m_user_cfg;

typedef sat_mosi_packet_t 		hub_to_nordic_t;
typedef sat_sensor_record_t		nordic_to_hub_t;

typedef struct padded_nord {
	uint32_t 		buf[1];
	nordic_to_hub_t packet;
} padded_nordic_to_hub_t;

padded_nordic_to_hub_t	padded_rx;
static nordic_to_hub_t 	*rx_from_nordic;

static hub_to_nordic_t 	tx_to_nordic;

typedef struct quaternion_ {
	uint32_t timestamp;
	float quat_w;		// Quarternion's scalar component, rotation degree around the vector
	float quat_x;		// Quarternion's x vector component
	float quat_y;		// Quarternion's y vector component
	float quat_z;		// Quarternion's z vector component
} quaternion_t;

extern void main_task (uint32_t);

const TASK_TEMPLATE_STRUCT  MQX_template_list[] =
{
    /* Task Index,   Function,   Stack,  Priority,   Name,   Attributes,          Param, Time Slice */
    { 10L,          main_task,  1500L,  8L,         "Main", MQX_AUTO_START_TASK, 0,     0  },
    { 0 }
};

static bool get_raw_data_sample_quat(quaternion_t *quat_data)
{
	uint8_t mts_buffer[2], stm_buffer[2];
	dspi_status_t 			Error;
	static uint32_t			dummy_rtc = 0xABCDEF12;

	tx_to_nordic.command = SAT_SPI_START;
	tx_to_nordic.rtc_value = 0xE1E2E3e4;

#if PRINT_DEBUG
	printf("\r\n\r\n\r\nTransmitting:");
	printf( "\r\ncommand: 		0x%02x"
			"\r\nRTC: 			0x%02x",
			tx_to_nordic.command,tx_to_nordic.rtc_value);
	printf("\n");
#endif

	/* Read and parse from Nordic */
	Error = DSPI_DRV_MasterTransferDataBlocking(nordic_instance, &spi0_device,
			(uint8_t *) &tx_to_nordic, (uint8_t *) &padded_rx,
			sizeof(padded_rx), 1);

	uint8_t *ptr = (uint8_t *) &padded_rx;
	rx_from_nordic = (nordic_to_hub_t*)(ptr + 3);

#if PRINT_DEBUG
	printf("\r\nReceiving:");

	printf( "\r\nTimestamp: 0x%02x"
			"\r\nfloat[0]: 	0x%f"
			"\r\nfloat[1]: 	0x%f"
			"\r\nfloat[2]: 	0x%f"
			"\r\nfloat[3]: 	0x%f",
			rx_from_nordic->timestamp,rx_from_nordic->data.quat_w,
			rx_from_nordic->data.quat_x,rx_from_nordic->data.quat_y,rx_from_nordic->data.quat_z);
	printf("\n");
#endif

/* For now only get the quaternion "fake"/"simulated" data from Nordic.
*/
	quat_data->timestamp	= rx_from_nordic->timestamp;
	quat_data->quat_w		= rx_from_nordic->data.quat_w;
	quat_data->quat_x		= rx_from_nordic->data.quat_x;
	quat_data->quat_y		= rx_from_nordic->data.quat_y;
	quat_data->quat_z		= rx_from_nordic->data.quat_z;

	return true;
}

void hardware_init(void) {
	/* Enable port clock */
    SIM_MemMapPtr   sim = SIM_BASE_PTR;

    sim->SCGC5 |= SIM_SCGC5_PORTA_MASK;		/* Enable PTA clock */
    sim->SCGC5 |= SIM_SCGC5_PORTB_MASK;		/* Enable PTB clock */
    sim->SCGC5 |= SIM_SCGC5_PORTC_MASK;		/* Enable PTC clock */
    sim->SCGC5 |= SIM_SCGC5_PORTD_MASK;		/* Enable PTD clock */
    sim->SCGC5 |= SIM_SCGC5_PORTE_MASK;		/* Enable PTE clock */

    /* Enable clock gate to DSPI0 module */
    sim->SCGC6 |= SIM_SCGC6_SPI0_MASK;		/* Enable SPI0 clock */

    /* Setting GPIO pins for Invensense*/
    PORT_MemMapPtr  pctl;

    /* port number    Alt choice (1 == GPIO) */
    pctl = (PORT_MemMapPtr)PORTA_BASE_PTR;
    pctl->PCR[1] = PORT_PCR_MUX(1);  					   /* PTA1 gpio*/
    pctl->PCR[2] = PORT_PCR_MUX(1);  					   /* PTA2 gpio*/

    pctl = (PORT_MemMapPtr)PORTB_BASE_PTR;
    pctl->PCR[17] = PORT_PCR_MUX(1);  					   /* PTB17 gpio*/

    pctl = (PORT_MemMapPtr)PORTC_BASE_PTR;
    pctl->PCR[0] = PORT_PCR_MUX(1);  					   /* PTC0 gpio*/
    pctl->PCR[1] = PORT_PCR_MUX(1);  					   /* PTC1 gpio*/
    pctl->PCR[3] = PORT_PCR_MUX(1);  					   /* PTC3 gpio*/
    pctl->PCR[4] = PORT_PCR_MUX(1);  					   /* PTC4 gpio*/
    pctl->PCR[7] = PORT_PCR_MUX(1);  					   /* PTC7 gpio*/
    pctl->PCR[11] = PORT_PCR_MUX(1);  					   /* PTC11 gpio*/

    pctl = (PORT_MemMapPtr)PORTD_BASE_PTR;
    pctl->PCR[0] = PORT_PCR_MUX(1);  					   /* PTD0 gpio*/
    pctl->PCR[1] = PORT_PCR_MUX(1);  					   /* PTD1 gpio*/
    pctl->PCR[4] = PORT_PCR_MUX(1);  					   /* PTD4 gpio*/
    pctl->PCR[5] = PORT_PCR_MUX(1);  					   /* PTD5 gpio*/

    pctl = (PORT_MemMapPtr)PORTE_BASE_PTR;
    pctl->PCR[0] = PORT_PCR_MUX(1);  					   /* PTE0 gpio*/
    pctl->PCR[1] = PORT_PCR_MUX(1);  					   /* PTE1 gpio*/

    /* I2C 1 pins */
    pctl = (PORT_MemMapPtr)PORTB_BASE_PTR;
    pctl->PCR[2] = PORT_PCR_MUX(2) | PORT_PCR_ODE_MASK;  	/* PTB2 alt2, set Open Drain*/
    pctl->PCR[3] = PORT_PCR_MUX(2) | PORT_PCR_ODE_MASK;  	/* PTB3 alt2, set Open Drain*/

    /* I2C 2 pins */
    pctl = (PORT_MemMapPtr)PORTC_BASE_PTR;
    pctl->PCR[10] = PORT_PCR_MUX(2);  					   /* PTC10 alt2, set Open Drain*/
    pctl->PCR[11] = PORT_PCR_MUX(3);  					   /* PTC11 alt2, set Open Drain*/

    /* UART 1 pins */
    pctl = (PORT_MemMapPtr)PORTE_BASE_PTR;
    pctl->PCR[0] = PORT_PCR_MUX(3);  					   /* PTE0 alt3*/
    pctl->PCR[1] = PORT_PCR_MUX(3);  					   /* PTE1 alt3*/

    /* SPI 0 pins */
    /* PORT_PCR_DSE_MASK is setting for high drive strength.
     * Not using DSE for now */
    pctl = (PORT_MemMapPtr)PORTD_BASE_PTR;
    pctl->PCR[1] = PORT_PCR_MUX(2);     /* PTD1, alt2, DSPI0.SCK    */
    pctl->PCR[2] = PORT_PCR_MUX(2);     /* PTD2, alt2, DSPI0.SOUT   */
    pctl->PCR[3] = PORT_PCR_MUX(2);     /* PTD3, alt2, DSPI0.SIN    */
    pctl->PCR[4] = PORT_PCR_MUX(2);     /* PTD4, alt2, DSPI0.PCS1   */
}

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
		// first pin in the table is set
		}
    }
    if(ioctl(pins_fd, GPIO_IOCTL_WRITE_LOG1, NULL) == IO_OK)
    {
		printf("OK write log1\n");
    }
    if(ioctl(pins_fd, GPIO_IOCTL_WRITE_LOG0, NULL) == IO_OK)
    {
		printf("OK write log0\n");
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
    spi0_m_user_cfg.whichPcs = kDspiPcs1;   /* Nordic on hub uses SPI0_PCS1 */

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
* Task Name : main_task
* Comments  :
*
*END*----------------------------------------------------------------------*/
void main_task
   (
      uint32_t dummy
   )
{
    MQX_FILE_PTR           spifd;
    uint32_t               param, result, i = 0;
	quaternion_t 			quat_data;

    /* Initializing pins */
    _int_install_unexpected_isr();
    hardware_init();
    turn_on_LED();

    printf ("\n-------------- SPI driver example --------------\n\n");
    printf ("\n-------------- SPI0 to Nordic --------------\n\n");

    /* Initializing SPI to Nordic */
    init_spi0_nordic();

    printf("\r\nOpen Source Sensor Fusion with Invensense MPU-9250\r\n");

	while (true)
	{

	   get_raw_data_sample_quat(&quat_data);


#if PRINT_DEBUG
	   	print_quat_data(&quat_data);
	   		    _time_delay(5000); /* cwati TODO delay 1s to debug */

#else
		output_quaternion_packet_local(quat_data);
#endif

	}


}
