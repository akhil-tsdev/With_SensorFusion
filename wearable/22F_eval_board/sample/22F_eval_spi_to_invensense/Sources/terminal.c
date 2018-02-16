/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*******************************************************************************
 * Standard C Included Files
 ******************************************************************************/   
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
/*******************************************************************************
 * SDK Included Files
 ******************************************************************************/
#include "fsl_os_abstraction.h"
/*******************************************************************************
 * Application Included Files
 ******************************************************************************/
#include "terminal.h"
/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
void print_welcome(void)
{
    printf("\r\n*************************************");
    printf("\r\nDSPI eDMA Demo");
    printf("\r\n-------------------------------------");
    printf("\r\nDSPI Transfers using eDMA");
    printf("\r\n*************************************");
    printf("\r\n\r\n");
}

void print_configuration(void)
{
    printf("\r\n*************************************");  
    printf("\r\nConfiguration");
    printf("\r\n");
    printf("\r\n  Terminal Baud: 115200 bps");
    printf("\r\n  DSPI Bit rate: User defined (3 kHz to 2 MHz)");
    printf("\r\n  DSPI Master:   DSPI0");
    printf("\r\n  DSPI Slave:    DSPI1");
    printf("\r\n");
}

void print_pin_setup(void)
{
	printf ("cwati");
	printf("\r\n       SPI1 to Nordic");
	printf("\r\n-------------------------------------");
	printf("\r\n     CS0 PTD4 ---------- ");
	printf("\r\n     SCK PTD5 ---------- ");
	printf("\r\n  MstOut PTD6 ----------  ");
	printf("\r\n   MstIn PTD7 ---------- ");
	printf("\r\n*************************************");
	printf("\r\n\r\n");

	return;
#if defined(TWR_K64F120M) || defined(TWR_K22F120M) || defined(TWR_K22F120M128R) || defined(TWR_K22F120M256R)
    printf("\r\n       Master            Slave");
    printf("\r\n-------------------------------------");
    printf("\r\n      SS PTD0 ---------- PTB10");
    printf("\r\n     SCK PTD1 ---------- PTB11");
    printf("\r\n  MstOut PTD2 ---------- PTB17 SlvIn");
    printf("\r\n   MstIn PTD3 ---------- PTB16 SlvOut");
    printf("\r\n*************************************");
    printf("\r\n\r\n");
#elif defined(FRDM_K64F120M) || defined(FRDM_K22F120M)
    printf("\r\n       Master            Slave");
    printf("\r\n-------------------------------------");
    printf("\r\n      SS PTD0 ---------- PTD4");
    printf("\r\n     SCK PTD1 ---------- PTD5");
    printf("\r\n  MstOut PTD2 ---------- PTD7 SlvIn");
    printf("\r\n   MstIn PTD3 ---------- PTD6 SlvOut");
    printf("\r\n*************************************");
    printf("\r\n\r\n");
#elif defined(TWR_KV31F120M) || defined(TWR_KV31F120M128R) || defined(TWR_KV31F120M256R)
    printf("\r\n       Master             Slave");
    printf("\r\n-------------------------------------");
    printf("\r\n      SS PTE16 ---------- PTB10");
    printf("\r\n     SCK PTE17 ---------- PTB11");
    printf("\r\n  MstOut PTE18 ---------- PTE3 SlvIn");
    printf("\r\n   MstIn PTE19 ---------- PTE1 SlvOut");
    printf("\r\n*************************************");
    printf("\r\n\r\n");
#endif
}

void print_edma_tcd(DMA_MemMapPtr edma, uint8_t index)
{
    uint32_t dataAddr;

    printf("\r\n*************************************");
    printf("\r\nTCD_%d", index);
    printf("\r\n*************************************");

    dataAddr = (uint32_t)edma->TCD[index].SADDR;
    printf("\r\nsrcAddress:     %08X", (unsigned int)dataAddr);

    dataAddr = (uint32_t)edma->TCD[index].SOFF;
    printf("\r\nsrcOffset:      %08X", (unsigned int)dataAddr);

    dataAddr = (uint32_t)edma->TCD[index].SLAST;
    printf("\r\nsrcLastOffset:  %08X", (unsigned int)dataAddr);
    printf("\r\n-------------------------------------");

    dataAddr = (uint32_t)edma->TCD[index].DADDR;
    printf("\r\ndestAddress:    %08X", (unsigned int)dataAddr);

    dataAddr = (uint32_t)edma->TCD[index].DOFF;
    printf("\r\ndestOffset:     %08X", (unsigned int)dataAddr);

    dataAddr = (uint32_t)edma->TCD[index].DLAST_SGA;
    printf("\r\ndestLastOffset: %08X", (unsigned int)dataAddr);
    printf("\r\n-------------------------------------");

    dataAddr = (uint32_t)edma->TCD[index].ATTR;
    printf("\r\nAttributes:                %08X", (unsigned int)dataAddr);

    dataAddr = (uint32_t)edma->TCD[index].NBYTES_MLNO;
    printf("\r\nMinor Loop Byte Count:     %08X", (unsigned int)dataAddr);

    dataAddr = (uint32_t)edma->TCD[index].CITER_ELINKNO;
    printf("\r\nCurrent Major Loop Count:  %08X", (unsigned int)dataAddr);

    dataAddr = (uint32_t)edma->TCD[index].BITER_ELINKNO;
    printf("\r\nStarting Major Loop Count: %08X", (unsigned int)dataAddr);

    dataAddr = (uint32_t)edma->TCD[index].CSR;
    printf("\r\nControl & Satus:           %08X", (unsigned int)dataAddr);

    printf("\r\n*************************************");
    printf("\r\n");
}

void print_edma_ch_erq(DMA_MemMapPtr edma, uint8_t channel)
{
    uint32_t dataAddr;

    printf("\r\n*************************************");
    printf("\r\nDMA ERQ Channel  %d", channel);
    printf("\r\n*************************************");

    dataAddr = (uint32_t)edma->ERQ;

    if(dataAddr & channel)
    {
        printf("\r\nEnabled.");
    }
    else
    {
        printf("\r\nDisabled.");
    }

    printf("\r\n*************************************");
    printf("\r\n");

}

void print_dmamux_ch(DMAMUX_MemMapPtr dmamux, uint8_t channel)
{
    uint8_t *temp = OSA_MemAlloc(sizeof(channel));
    uint32_t dataAddr;

    printf("\r\n*************************************");
    printf("\r\nDMAMUX Settings Channel %d", channel);
    printf("\r\n*************************************");

    dataAddr = (uint32_t)dmamux->CHCFG[channel];

    if(dataAddr & DMAMUX_CHCFG_ENBL_MASK)
    {
        printf("\r\nChannel Enabled.");
    }
    else
    {
        printf("\r\nChannel Disabled.");
    }

    printf("\r\n-------------------------------------");

    if(dataAddr & DMAMUX_CHCFG_TRIG_MASK)
    {
        printf("\r\nTriggering Enabled.");
    }
    else
    {
        printf("\r\nTriggering Disabled.");
    }

    printf("\r\n-------------------------------------");

    *temp = dataAddr & 0x3F;
    printf("\r\nChannel Source: %d", *temp);

    printf("\r\n*************************************");
    printf("\r\n"); 

    OSA_MemFree(temp);
}

uint32_t get_spi_freq(void)
{
    int8_t index;
    uint8_t count;
    volatile uint8_t isValid = 0;
    uint8_t userIn[7];
    volatile uint32_t freq = 0;

    while(!isValid)
    {
        printf("\r\nEnter a SPI clock frequency between 3,000 Hz & 2,000,000 Hz below.\r\nNOTE: If the number is less than 1,000,000 press enter after typing the number.\r\n\r\n->");
        scanf("%7s", userIn);
        printf("%s\n", userIn);

        count = 0;
        for(index = 7; index > 0; index--)
        {
            if((userIn[index - 1] >= 0x30) && (userIn[index - 1] <= 0x3A))
            {

                freq += (userIn[index - 1] - 0x30) * (pow(10, count));
                count++;
            }
        }

        printf("\r\nThe number received was: %d Hz\r\n", (unsigned int)freq);

        /* Check if valid entry exists. */
        if((freq >= 3000) && (freq <= 2000000))
        {
            isValid = 1;
        }
        else
        {
            freq = 0;
        }

        /* Clear input buffer. */
        for(count = 0; count < 7; count++)
        {
            userIn[count] = ' ';
        }
    }

    OSA_TimeDelay(750);

    return freq;
}

/******************************************************************************
 * EOF
 ******************************************************************************/
