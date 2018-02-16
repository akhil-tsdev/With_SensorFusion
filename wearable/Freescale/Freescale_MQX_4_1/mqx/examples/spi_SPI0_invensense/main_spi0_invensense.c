/*HEADER**********************************************************************
*
* Copyright 2012 Freescale Semiconductor, Inc.
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
*   This file contains the source for a simple example of an
*   application that writes and reads the SPI memory using the SPI driver.
*   It's already configured for onboard SPI flash where available.
*
*
*END************************************************************************/


#include <string.h>
#include <mqx.h>
#include <bsp.h>
#include <spi.h>
#include "spi_memory.h"

#if ! TURINGSENSE
#error Please define Turingsense for code to work as expected
#endif

#if ! BSPCFG_ENABLE_IO_SUBSYSTEM
#error This application requires BSPCFG_ENABLE_IO_SUBSYSTEM defined non-zero in user_config.h. Please recompile BSP with this option.
#endif


#ifndef BSP_DEFAULT_IO_CHANNEL_DEFINED
#error This application requires BSP_DEFAULT_IO_CHANNEL to be not NULL. Please set corresponding BSPCFG_ENABLE_TTYx to non-zero in user_config.h and recompile BSP with this option.
#endif


#ifndef BSP_SPI_MEMORY_CHANNEL
#error This application requires BSP_SPI_MEMORY_CHANNEL to be defined. Please set it to appropriate SPI channel number in user_config.h and recompile BSP with this option.
#endif

const char *device_mode[] =
{
    "SPI_DEVICE_MASTER_MODE",
    "SPI_DEVICE_SLAVE_MODE",
};

const char *clock_mode[] =
{
    "SPI_CLK_POL_PHA_MODE0",
    "SPI_CLK_POL_PHA_MODE1",
    "SPI_CLK_POL_PHA_MODE2",
    "SPI_CLK_POL_PHA_MODE3"
};


#ifdef BSP_SPI_MEMORY_GPIO_CS

/*FUNCTION*---------------------------------------------------------------
*
* Function Name : set_CS
* Comments  : This function sets chip select signal to enable/disable memory.
*             It's used only on platforms with manual CS handling.
*END*----------------------------------------------------------------------*/
#define BSP_SPI_MEMORY_SPI_CS     (SPI_PUSHR_PCS(1 << 1)) //PCS1

_mqx_int set_CS (uint32_t cs_mask, void *user_data)
{
    LWGPIO_STRUCT_PTR spigpio = (LWGPIO_STRUCT_PTR)user_data;

    if (cs_mask & BSP_SPI_MEMORY_SPI_CS)
    {
        lwgpio_set_value(spigpio, LWGPIO_VALUE_LOW);
    }
    else
    {
        lwgpio_set_value(spigpio, LWGPIO_VALUE_HIGH);
    }
    return MQX_OK;
}
#endif /* BSP_SPI_MEMORY_GPIO_CS */


extern void main_task (uint32_t);


const TASK_TEMPLATE_STRUCT  MQX_template_list[] =
{
    /* Task Index,   Function,   Stack,  Priority,   Name,   Attributes,          Param, Time Slice */
    { 10L,          main_task,  1500L,  8L,         "Main", MQX_AUTO_START_TASK, 0,     0  },
    { 0 }
};


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

//    /* Must select PTD4 as output for CS to be captured */
//    GPIO_PIN_STRUCT 	ss_pins[2];
//    uint32_t 			read_pin_table[2];
//    MQX_FILE_PTR	    ss_pins_fd;
//
//    ss_pins[0] = (GPIO_PORT_D|GPIO_PIN4);	/* Slave Select / CS / chip select */
//    ss_pins[1] = GPIO_LIST_END;
//
//    ss_pins_fd = fopen( "gpio:output",(char *)&ss_pins);
//
//    if(NULL == ss_pins_fd){
//        printf("ERROR: Failed to open GPIO char device for Slave Select!\n");
//    }

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
        printf("ERROR: Failed to open GPIO char device for Interrupt!\n");
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
    SPI_STATISTICS_STRUCT  stats;
    SPI_READ_WRITE_STRUCT  rw;

#ifdef BSP_SPI_MEMORY_GPIO_CS
    LWGPIO_STRUCT          spigpio;
    SPI_CS_CALLBACK_STRUCT callback;
#endif

    printf ("\n-------------- SPI driver example --------------\n\n");
    printf ("\n-------------- SPI0 to Invensense --------------\n\n");

    /* Initializing pins */
    hardware_init();

    turn_on_LED();

    /* Open the SPI driver */
    spifd = fopen ("spi0:2", NULL);

    if (NULL == spifd)
    {
        printf ("Error opening SPI driver! GAAAAHHHHHH\n");
        _time_delay (200L);
        _task_block ();
    }

    /* Set transfer mode */
    param = SPI_DEVICE_MASTER_MODE;
    printf ("Setting transfer mode to %s ... ", device_mode[param]);
    if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_SET_TRANSFER_MODE, &param))
    {
        printf ("OK\n");
    }
    else
    {
        printf ("ERROR\n");
    }

    /* Set endian */
    param = SPI_DEVICE_BIG_ENDIAN;
    printf ("Setting endian to %s ... ", param == SPI_DEVICE_BIG_ENDIAN ? "SPI_DEVICE_BIG_ENDIAN" : "SPI_DEVICE_LITTLE_ENDIAN");
    if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_SET_ENDIAN, &param))
    {
        printf ("OK\n");
    }
    else
    {
        printf ("ERROR\n");
    }

#define MPU9250_READ_FLAG   	    0x80
    

    unsigned char send_buffer[128];
    unsigned char recv_buffer[128];

    /* Test simultaneous write and read */
    memset (send_buffer, 0, sizeof (send_buffer));
    memset (recv_buffer, 0, sizeof (recv_buffer));
    uint16_t	length = 128;

//	param = SPI_FLAG_FULL_DUPLEX;
//	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_SET_FLAGS, &param))
//	{
//		printf ("OK\n");
//	} else {
//		printf ("Failed to set FULL DUPLEX!\n");
//	}
//
//	for (i = 0; i < 128; i++) {
//		send_buffer[i] = i | MPU9250_READ_FLAG;
//	}

    while (1) {
	for (int loop = 0; loop < 128; loop++) {

		send_buffer[0] = 0 |  MPU9250_READ_FLAG;
		send_buffer[1] = 0;
		recv_buffer[0] = 0;
		recv_buffer[1] = 0;

    	rw.BUFFER_LENGTH = 2;
    	rw.WRITE_BUFFER = (char *)send_buffer;
    	rw.READ_BUFFER = (char *)recv_buffer;

    	printf ("IO_IOCTL_SPI_READ_WRITE ... ");
    	if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_READ_WRITE, &rw))
    	{
    		printf ("OK\n");
    	}
    	else
    	{
    		printf ("ERROR\n");
    	}
    	printf ("Write 0x%02x", (unsigned char)rw.WRITE_BUFFER[0]);
    	printf ("Read 0x%02x 0x%02x\n", (unsigned char)rw.READ_BUFFER[0], (unsigned char)rw.READ_BUFFER[1]);

    	fflush(spifd);

    	_time_delay (500L); /* Time delay in MS */
	}
    }

//	length = 1;
//	while (1) {
//		for (i = 0; i < 128; i++) {
//			send_buffer[0] = 0 |  MPU9250_READ_FLAG;
//			send_buffer[1] = 0;
//			recv_buffer[0] = 0;
//			recv_buffer[1] = 0;
//
//			if(length != fread(send_buffer, 2, length, spifd)){
//				printf("Error reading SPI!\n");
//			}
//			printf ("Read recv 0x%02x 0x%02x \n", send_buffer[0], send_buffer[1]);
//
//			_time_delay (500L); /* Time delay in MS */

//			fflush(spifd);
//		}
//	}

#define BSPCFG_ENABLE_SPI_STATS			1

	/* Get statistics */
	printf ("Getting statistics: ");
	result = ioctl (spifd, IO_IOCTL_SPI_GET_STATS, &stats);
	if (SPI_OK == result)
	{
		printf ("\n");
		printf ("Rx packets:   %d\n", stats.RX_PACKETS);
		printf ("Tx packets:   %d\n", stats.TX_PACKETS);
	}
	else if (MQX_IO_OPERATION_NOT_AVAILABLE == result)
	{
		printf ("not available, define BSPCFG_ENABLE_SPI_STATS\n");
	}
	else
	{
		printf ("ERROR\n");
	}
	printf ("\n");

	fflush (spifd);


    /* Close the SPI */
    result = (uint32_t)fclose (spifd);
    if (result)
    {
        printf ("Error closing SPI, returned: 0x%08x\n", result);
    }

    printf ("\n-------------- End of example --------------\n\n");
    while (1) {
    	//do nothing;
    	;
    }
}
