/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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

#include "bootloader_common.h"
#include "bootloader/context.h"
#include "property/property.h"
#include "device/fsl_device_registers.h"
#include <assert.h>

////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////
// Clock mode types
typedef enum _target_clock_mode
{
    kClockMode_FEI = 0,
    kClockMode_FEE = 1,
    kClockMode_FBI = 2,
    kClockMode_FBE = 3,
    kClockMode_PEE = 4,
    kClockMode_PBE = 5,
    kClockMode_Default = kClockMode_FEI,
}target_clock_mode_t;


////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

// This function implements clock mode switch between FEI and PEE mode used in this bootloader
void clock_mode_switch(const target_clock_mode_t currentMode, const target_clock_mode_t expectedMode);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See bootloader_common.h for documentation on this function.
void configure_clocks(bootloader_clock_option_t option)
{
#if BL_TARGET_FLASH
    
    static target_clock_mode_t s_currentClockMode = kClockMode_FEI;
    static uint32_t s_defaultClockDivider;
    
    if (option == kClockOption_EnterBootloader)
    {
        s_defaultClockDivider = SIM->CLKDIV1;
        // General procedure to be implemented:
        // 1. Read clock flags and divider from bootloader config in property store
        bootloader_configuration_data_t * config = &g_bootloaderContext.propertyInterface->store->configurationData;
        uint8_t options = config->clockFlags;

        // 2. If NOT High Speed, do nothing (use reset clock config)
        if (options & kClockFlag_HighSpeed)
        {
            // High speed flag is set (meaning disabled), so just use default clocks.
            SystemCoreClock = kDefaultClock / (SIM_BRD_CLKDIV1_OUTDIV1(SIM) + 1);
            return;
        }

        // 3. Set OUTDIV1 based on divider in config. OUTDIV4 starts out at /1.
        // The divider values are masked by the maximum bits per divider.
        uint32_t div1 = ((~config->clockDivider) & (SIM_CLKDIV1_OUTDIV1_MASK >> SIM_CLKDIV1_OUTDIV1_SHIFT)) + 1;
        uint32_t div2 = 1;
        uint32_t div4 = 1;

        // 4. Get MCGOUTCLK
        uint32_t McgOutClk = kHIRC;
        
        // 5. Update SystemCoreClock global.
        SystemCoreClock = kHIRC / div1;

        // 6. Keep bus freq below max.
        //
        // The bus clock is divided by OUTDIV2:
        //      MCGOUTCLK -> OUTDIV2 -> bus_clk
        uint32_t freq = McgOutClk;
        while ((freq / div2) > kMaxBusClock)
        {
            // Increase bus clock divider.
            ++div2;
        }
        assert((div2 >= kDivider_Min) && (div2 <= kDivider_Max));


        // 7. Keep flash freq below max.
        //
        // The flash clock is diveded by OUTDIV4:
        //      MCGOUTCLK -> OUTDIV4 ->flash_clk
        freq = McgOutClk;
        while ((freq / div4) > kMaxFlashClock)
        {
            // Increase bus/flash clock divider.
            ++div4;
        }
        assert((div4 >= kDivider_Min) && (div4 <= kDivider_Max));

       if((div1 == 1) && ((div2 > 8) || (div4 > 8)))
        {
            return;
        }
        
        // 8. Now set the dividers
        SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(div1 - 1) |
                      SIM_CLKDIV1_OUTDIV2(div2 - 1) |
                      SIM_CLKDIV1_OUTDIV4(div4 - 1); /* Update system prescalers */

        
        // 9. Switch to FEE mode.
        clock_mode_switch(s_currentClockMode, kClockMode_FEE);
        s_currentClockMode = kClockMode_FEE;
    }
    else if (option == kClockOption_ExitBootloader)
    {
        // Restore from FEE mode to FEI mode
        clock_mode_switch(s_currentClockMode, kClockMode_FEI);
        
        // Restore clock divider
        SIM_CLKDIV1 = s_defaultClockDivider;
    }
#endif // BL_TARGET_FLASH
}

void clock_mode_switch(const target_clock_mode_t currentMode, const target_clock_mode_t expectedMode)
{
    // Note: here only implements clock switch between FEI and FEE, 
    // The other modes are not supported.
    assert (currentMode == kClockMode_FEE || currentMode == kClockMode_FEI);
    assert (expectedMode == kClockMode_FEE || expectedMode == kClockMode_FEI);
    
    if (currentMode == expectedMode)
    {
        return;
    }
    
    if (expectedMode == kClockMode_FEE)
    {
        /* Switch to FEE mode */
        MCG_BWR_C2_RANGE(MCG, 2); /* MCG_C2: RANGE = 2 */
        MCG_BWR_C1_FRDIV(MCG, 6); // FRDIV=6, RANGE=2, divide IRC48M with 1280
        MCG_BWR_C4_DRST_DRS(MCG, 1); // Multiply with 1280, MCGOUTCLK is 48Mhz
        MCG_BWR_C1_IREFS(MCG, 0); // Switch to external reference clock.
        
        MCG_BWR_C7_OSCSEL(MCG, 2); // Select IRC48M as Oscillator.
        
        while (MCG_BRD_S_IREFST(MCG)); // Wait until external reference clock is ready.
    }
    else if (expectedMode == kClockMode_FEI)
    {
        MCG_BWR_C1_IREFS(MCG, 1); // Switch to internal reference clock.
        while (!MCG_BRD_S_IREFST(MCG)); // Wait until internal reference clock is ready.
        
        // Restore registers to default value out of reset.
        MCG_WR_C1(MCG, 0x04);
        MCG_WR_C2(MCG, 0x80);
        MCG_BWR_C4_DRST_DRS(MCG, 0);   
        MCG_WR_C7(MCG, 0); 
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

