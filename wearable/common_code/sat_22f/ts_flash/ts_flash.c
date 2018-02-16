/*
 * Copyright (c) 2016 TuringSense
 *
 * ts_flash.c
 *
 */

#include "ts_flash.h"

static UINT32 gCallBackCnt; /* global counter in callback(). */
UINT32 margin_read_level;   /* 0=normal, 1=user - margin read for reading 1's */
static UINT32 FailAddr;

pFLASHCOMMANDSEQUENCE g_FlashLaunchCommand = (pFLASHCOMMANDSEQUENCE)0xFFFFFFFF;
/* array to copy __Launch_Command func to RAM */
UINT16 __ram_func[LAUNCH_CMD_SIZE/2];
UINT16 __ram_for_callback[CALLBACK_SIZE/2]; /* length of this array depends on total size of the functions need to be copied to RAM*/

static bool init_done = false;

/************************************************************************************************/
/************************************************************************************************/
/*                      Flash Standard Software Driver Structure                                */
/************************************************************************************************/
/************************************************************************************************/
FLASH_SSD_CONFIG flashSSDConfig =
{
    FTFx_REG_BASE,          /* FTFx control register base */
    PFLASH_BLOCK_BASE,      /* base address of PFlash block */
    PBLOCK_SIZE,            /* size of PFlash block */
    DEFLASH_BLOCK_BASE,     /* base address of DFlash block */
    0,                      /* size of DFlash block */
    EERAM_BLOCK_BASE,       /* base address of EERAM block */
    0,                      /* size of EEE block */
    DEBUGENABLE,            /* background debug mode enable bit */
    NULL_CALLBACK           /* pointer to callback function */
};

/*********************************************************************
*
*  Function Name    : callback
*  Description      : callback function for flash operations
*  Arguments        : none
*  Return Value     :
*
*********************************************************************/
static void ts_flash_callback(void)
{
    /* just increase this variable to observer that this callback() func has been invoked */
    gCallBackCnt++;
}


err_t ts_flash_init() {
    /**************************************************************************
      * Set CallBack to callback function
    ***************************************************************************/
    flashSSDConfig.CallBack = (PCALLBACK)RelocateFunction((UINT32)__ram_for_callback , CALLBACK_SIZE , (UINT32) ts_flash_callback);
    g_FlashLaunchCommand = (pFLASHCOMMANDSEQUENCE)RelocateFunction((UINT32)__ram_func , LAUNCH_CMD_SIZE ,(UINT32)FlashCommandSequence);

    init_done = true;
}

/*
 * Flash writing can only change bit 1 to 0.  You must make sure the flash area is properly
 * erased (all FF's) before you try writing into it.
 *
 * Notice that LSB goes first.
 * For example, you want to write 4-byte of satellite ID 305419896.
 * 305419896d = 0x12345678
 * Normally you read it as 0x12345678
 *
 * So in your program buffer you should put {0x78, 0x56, 0x34, 0x12}
 * Then when you read
 * uint32_t sat_id = ts_read_from_flash(addr);
 * You will get sat_id = 0x12345678
 *
 */
err_t ts_write_to_flash(uint32_t destination, uint32_t size, uint8_t* program_buffer) {
	uint32_t ret;

    if (!init_done) {
    	ts_flash_init();
    }

    ret = FlashProgram(&flashSSDConfig, destination, size, \
                                   program_buffer, g_FlashLaunchCommand);


    if (FTFx_OK != ret)
    {
    	return E_FLASH_ERR;
    }

    /* Program Check user margin levels*/
    for (margin_read_level = 1; margin_read_level < 0x2; margin_read_level++)
    {
        ret = FlashProgramCheck(&flashSSDConfig, destination, size, program_buffer, \
                                    &FailAddr, margin_read_level, g_FlashLaunchCommand);
        if (FTFx_OK != ret)
        {
        	return E_FLASH_ERR;
        }
    }

    return E_OK;
}

/*
 * This operation will wipe out the whole 2Kb (0x800) flash to all 0xFF's.
 * You must erase before you write to a location of flash.
 * The minimum erase is the whole sector.
 */
err_t ts_erase_flash_sector() {
    uint32_t destination = TS_FLASH_STARTING_ADDR;
	uint32_t size = FTFx_PSECTOR_SIZE;	/* Minimum sector erase */
	uint32_t ret;

    if (!init_done) {
    	ts_flash_init();
    }

	ret = FlashEraseSector(&flashSSDConfig, destination, size, g_FlashLaunchCommand);
	if (FTFx_OK != ret)
	{
    	return E_FLASH_ERR;
	}

	/* Verify section for several sector of PFLASH */
	uint32_t number = FTFx_PSECTOR_SIZE/PRD1SEC_ALIGN_SIZE;
	for(margin_read_level = 0; margin_read_level < 0x2; margin_read_level++)
	{
		ret = FlashVerifySection(&flashSSDConfig, destination, number, margin_read_level, g_FlashLaunchCommand);
		if (FTFx_OK != ret)
		{
        	return E_FLASH_ERR;
		}
	}

    return E_OK;
}

uint32_t ts_read_from_flash(uint32_t dest) {
	return (READ32(dest));
}
