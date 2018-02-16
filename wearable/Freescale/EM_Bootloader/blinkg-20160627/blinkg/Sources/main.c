/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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
 */

#include "MK22F25612.h"

#define PORT_RD_PCR(base, index) (PORT_PCR_REG(base, index))
#define PORT_WR_PCR(base, index, value) (PORT_PCR_REG(base, index) = (value))
#define PORT_WR_PCR_MUX(base, index, value) (PORT_WR_PCR(base, index, (PORT_RD_PCR(base, index) & ~(PORT_PCR_MUX_MASK | PORT_PCR_ISF_MASK)) | PORT_PCR_MUX(value)))
#define PORT_BWR_PCR_MUX(base, index, value) (PORT_WR_PCR_MUX(base, index, value))

#define RED	    12   /* K22F's GPIOA pin number for red LED */
#define GREEN	13   /* K22F's GPIOA pin number for green LED */
#define BLUE	14   /* K22F's GPIOA pin number for blue LED */

static int i = 0;

void led_init(int led)
{
	PORT_BWR_PCR_MUX(PORTA, led, 1);
	PTA->PDDR |= (1 << led);
	PTA->PDOR |= (1 << led);
}

void led_set(int led, int value)
{
	if (value)
		PTA->PDOR &= ~ (1 << led);
	else
		PTA->PDOR |= (1 << led);
}

void delay(void)
{
	volatile uint32_t i = 1000000;

	while (i--)
	{
		__ASM("nop");
	}
}

void led_flash(int led)
{
	led_set(led, 1);
	delay();
	led_set(led, 0);
	delay();
}

#define FORCE_BOOTLOADER_LOC 0x1FFFC000
#define FORCE_BOOTLOADER_VALUE 0x424f4f54  /* arbitrarily chosen */
void force_bootloader(void)
{
	uint32_t *magic_loc = (uint32_t *) FORCE_BOOTLOADER_LOC;

	*magic_loc = FORCE_BOOTLOADER_VALUE;
	NVIC_SystemReset();
	// never returns
}

int main(void)
{
	int i;

	SIM->SCGC5 |= ( SIM_SCGC5_PORTA_MASK
			      | SIM_SCGC5_PORTB_MASK
			      | SIM_SCGC5_PORTC_MASK
			      | SIM_SCGC5_PORTD_MASK
			      | SIM_SCGC5_PORTE_MASK);

	SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;

	led_init (RED);
	led_init (GREEN);
	led_init (BLUE);

	for (i = 0; i < 2; i++)
		led_flash (BLUE);

	for (i = 0; i < 20; i++)
		led_flash (GREEN);

	force_bootloader();  // NEVER RETURNS

    return 0;
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
