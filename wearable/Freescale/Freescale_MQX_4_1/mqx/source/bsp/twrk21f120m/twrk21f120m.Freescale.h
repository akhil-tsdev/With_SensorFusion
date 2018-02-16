/*HEADER**********************************************************************
*
* Copyright 2011 Freescale Semiconductor, Inc.
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
*   This include file is used to provide information needed by
*   an application program using the kernel running on the
*   Freescale TWR-K20D72M Evaluation board.
*
*
*END************************************************************************/

#ifndef _twrk21f120m_h_
    #define _twrk21f120m_h_  1


/*----------------------------------------------------------------------
**                  HARDWARE INITIALIZATION DEFINITIONS
*/

/*
** Define the board type
*/
#define BSP_TWR_K21F120M                     1
#define BSP_NAME                            "TWRK21F120M"

/*
** PROCESSOR MEMORY BOUNDS
*/
#define BSP_PERIPH_BASE                 (CORTEX_PERIPH_BASE)

typedef void (*vector_entry)(void);

#define BSP_INTERNAL_FLASH_BASE    0x00000000
#define BSP_INTERNAL_FLASH_SIZE    0x00100000
#define BSP_INTERNAL_FLASH_SECTOR_SIZE  0x1000

#ifdef __CC_ARM
/* Keil compiler */
#define __EXTERNAL_MRAM_ROM_BASE 0x70000000
#define __EXTERNAL_MRAM_ROM_SIZE 0x00000000
#define __EXTERNAL_MRAM_RAM_BASE 0x70000000
#define __EXTERNAL_MRAM_RAM_SIZE 0x00080000
#define __EXTERNAL_LCD_BASE      0x60010000
#define __EXTERNAL_LCD_SIZE      0x0001FFFF
#define __EXTERNAL_LCD_DC_BASE   0x60000000

extern unsigned char Image$$USEDFLASH_END$$Base[];
#define __FLASHX_START_ADDR     ((void *)Image$$USEDFLASH_END$$Base)
#define __FLASHX_END_ADDR       ((BSP_INTERNAL_FLASH_BASE) + (BSP_INTERNAL_FLASH_SIZE))
#define __FLASHX_SECT_SIZE      BSP_INTERNAL_FLASH_SECTOR_SIZE

#define __INTERNAL_SRAM_BASE    0x1FFF0000
#define __INTERNAL_SRAM_SIZE    0x00020000
#define __SRAM_POOL             (void *)0x1FFF0000

/* Uncached area is maped to sram */
#define __UNCACHED_DATA_START   (0x1FFF0000)
#define __UNCACHED_DATA_SIZE    (0x00020000)
#define __UNCACHED_DATA_END     (0x2000FFF0)

#define __DEFAULT_PROCESSOR_NUMBER 1
#define __DEFAULT_INTERRUPT_STACK_SIZE 1024

extern unsigned char Image$$VECTORS$$Base[];
#define __VECTOR_TABLE_ROM_START ((void *)Image$$VECTORS$$Base)

extern unsigned char Image$$RAM_VECTORS$$Base[];
#define __VECTOR_TABLE_RAM_START ((void *)Image$$RAM_VECTORS$$Base)

#else /* __CC_ARM */

extern unsigned char __EXTERNAL_MRAM_ROM_BASE[],    __EXTERNAL_MRAM_ROM_SIZE[];
extern unsigned char __EXTERNAL_MRAM_RAM_BASE[],    __EXTERNAL_MRAM_RAM_SIZE[];
extern unsigned char __EXTERNAL_LCD_BASE[],         __EXTERNAL_LCD_SIZE[];
extern unsigned char __EXTERNAL_LCD_DC_BASE[];
extern const unsigned char __FLASHX_START_ADDR[];
extern const unsigned char __FLASHX_END_ADDR[];
extern const unsigned char __FLASHX_SECT_SIZE[];
extern unsigned char __INTERNAL_FLEXNVM_BASE[];
extern unsigned char __INTERNAL_FLEXNVM_SIZE[];

extern vector_entry __VECTOR_TABLE_RAM_START[]; // defined in linker command file
extern vector_entry __VECTOR_TABLE_ROM_START[]; // defined in linker command file

extern unsigned char __DEFAULT_PROCESSOR_NUMBER[];
extern unsigned char __DEFAULT_INTERRUPT_STACK_SIZE[];

#endif /* __CC_ARM */
#define BSP_EXTERNAL_MRAM_ROM_BASE          ((void *)__EXTERNAL_MRAM_ROM_BASE)
#define BSP_EXTERNAL_MRAM_ROM_SIZE          ((uint32_t)__EXTERNAL_MRAM_ROM_SIZE)
#define BSP_EXTERNAL_MRAM_RAM_BASE          ((void *)__EXTERNAL_MRAM_RAM_BASE)
#define BSP_EXTERNAL_MRAM_RAM_SIZE          ((uint32_t)__EXTERNAL_MRAM_RAM_SIZE)

#define BSP_EXTERNAL_LCD_BASE               ((void *)__EXTERNAL_LCD_BASE)
#define BSP_EXTERNAL_LCD_SIZE               ((uint32_t)__EXTERNAL_LCD_SIZE)
#define BSP_EXTERNAL_LCD_DC                 ((void *)__EXTERNAL_LCD_DC_BASE)

/* Enable modification of flash configuration bytes during loading for flash targets */
#ifndef BSPCFG_ENABLE_CFMPROTECT
    #define BSPCFG_ENABLE_CFMPROTECT        1
#endif
#if !BSPCFG_ENABLE_CFMPROTECT && defined(__ICCARM__)
    #error Cannot disable CFMPROTECT field on IAR compiler. Please define BSPCFG_ENABLE_CFMPROTECT to 1.
#endif

#if BSPCFG_ENABLE_SPI0 || BSPCFG_ENABLE_ISPI0
     #warning Flexbus is sharing pin with SPI0 moudle, hence enabling SPI0 that cause some demo use flexbus (mfs_ramdisk) do not run properly
#endif

#if BSPCFG_ENABLE_SAI
     #warning Flexbus is sharing pin with SAI moudle, hence enabling SAI that cause some demo use flexbus (mfs_ramdisk) do not run properly
#endif
/*
 * The clock configuration settings
 * Remove old definitions of "BSP_CLOCKS" in drivers and replace
 * by runtime clock checking. Its assumed that BSP_CLOCK_CONFIGURATION_1
 * sets PLL to full speed 48MHz to be compatible with old drivers.
 */

#ifndef BSP_CLOCK_CONFIGURATION_STARTUP
    #define BSP_CLOCK_CONFIGURATION_STARTUP (BSP_CLOCK_CONFIGURATION_DEFAULT)
#endif /* BSP_CLOCK_CONFIGURATION_STARTUP */

/* Init startup clock configuration is CPU_CLOCK_CONFIG_0 */
#define BSP_CLOCK_SRC                   (CPU_XTAL_CLK_HZ)
#define BSP_CORE_CLOCK                  (CPU_CORE_CLK_HZ_CONFIG_0)
#define BSP_SYSTEM_CLOCK                (CPU_CORE_CLK_HZ_CONFIG_0)
#define BSP_CLOCK                       (CPU_BUS_CLK_HZ_CONFIG_0)
#define BSP_BUS_CLOCK                   (CPU_BUS_CLK_HZ_CONFIG_0)
#define BSP_FLEXBUS_CLOCK               (CPU_FLEXBUS_CLK_HZ_CONFIG_0)
#define BSP_FLASH_CLOCK                 (CPU_FLASH_CLK_HZ_CONFIG_0)

/** MGCT: <category name="BSP Hardware Options"> */

/*
** The clock tick rate in miliseconds - be cautious to keep this value such
** that it divides 1000 well
**
** MGCT: <option type="number" min="1" max="1000"/>
*/
#ifndef BSP_ALARM_FREQUENCY
    #define BSP_ALARM_FREQUENCY             (200)
#endif

/*
** System timer definitions
*/
#define BSP_SYSTIMER_DEV          systick_devif
#define BSP_SYSTIMER_ID           0
#define BSP_SYSTIMER_SRC_CLK      CM_CLOCK_SOURCE_CORE
#define BSP_SYSTIMER_ISR_PRIOR    2
/* We need to keep BSP_TIMER_INTERRUPT_VECTOR macro for tests and watchdog.
 * Will be removed after hwtimer expand to all platforms */
#define BSP_TIMER_INTERRUPT_VECTOR INT_SysTick

/** MGCT: </category> */
/*
** Old clock rate definition in MS per tick, kept for compatibility
*/
#define BSP_ALARM_RESOLUTION                (1000 / BSP_ALARM_FREQUENCY)

/*
** Define the location of the hardware interrupt vector table
*/
#if MQX_ROM_VECTORS
    #define BSP_INTERRUPT_VECTOR_TABLE              ((uint32_t)__VECTOR_TABLE_ROM_START)
#else
    #define BSP_INTERRUPT_VECTOR_TABLE              ((uint32_t)__VECTOR_TABLE_RAM_START)
#endif

#ifndef BSP_FIRST_INTERRUPT_VECTOR_USED
    #define BSP_FIRST_INTERRUPT_VECTOR_USED     (0)
#endif

#ifndef BSP_LAST_INTERRUPT_VECTOR_USED
    #define BSP_LAST_INTERRUPT_VECTOR_USED      (250)
#endif

#define BSP_TIMER_INTERRUPT_VECTOR              INT_SysTick
#define BSP_TIMER_INTERRUPT_PRIORITY            2

/*
** EDMA interrupt level
*/
#define BSP_EDMA_INT_LEVEL                     (3)

/*
** RTC interrupt level
*/
#define BSP_RTC_INT_LEVEL                      (4)

/*
** LPM related
*/
#define BSP_LLWU_INTERRUPT_VECTOR              INT_LLW
#define BSP_LLWU_INTERRUPT_PRIORITY            3

#define BSP_LPM_DEPENDENCY_LEVEL_SERIAL_POLLED (30)
#define BSP_LPM_DEPENDENCY_LEVEL_SERIAL_INT    (31)

/*
** GPIO board specifications
*/
#define BSP_LED1                    (GPIO_PORT_D | GPIO_PIN4)
#define BSP_LED2                    (GPIO_PORT_D | GPIO_PIN5)
#define BSP_LED3                    (GPIO_PORT_D | GPIO_PIN6)
#define BSP_LED4                    (GPIO_PORT_D | GPIO_PIN7)

#define BSP_SW1                     (GPIO_PORT_C | GPIO_PIN6)
#define BSP_SW2                     (GPIO_PORT_C | GPIO_PIN7)
#define BSP_ACCEL_IRQ               (GPIO_PORT_B | GPIO_PIN0) /* onboard accelerometer IRQ pin */

#define BSP_LED1_MUX_GPIO           (LWGPIO_MUX_D4_GPIO)
#define BSP_LED2_MUX_GPIO           (LWGPIO_MUX_D5_GPIO)
#define BSP_LED3_MUX_GPIO           (LWGPIO_MUX_D6_GPIO)
#define BSP_LED4_MUX_GPIO           (LWGPIO_MUX_D7_GPIO)
#define BSP_SW1_MUX_GPIO            (LWGPIO_MUX_C6_GPIO)
#define BSP_SW2_MUX_GPIO            (LWGPIO_MUX_C7_GPIO)
#define BSP_ACCEL_MUX_GPIO          (LWGPIO_MUX_B0_GPIO)

/* definitions for user applications */
#define BSP_BUTTON1                 BSP_SW1
#define BSP_BUTTON2                 BSP_SW2

#define BSP_BUTTON1_MUX_GPIO        BSP_SW1_MUX_GPIO
#define BSP_BUTTON2_MUX_GPIO        BSP_SW2_MUX_GPIO
#define BSP_BUTTON1_MUX_IRQ         BSP_SW1_MUX_GPIO

/*
** SAI/audio specifications
*/
#define BSP_HAS_AUDIO_CODEC         (1)

/* HWTIMER definitions for user applications */
#define BSP_HWTIMER1_DEV            pit_devif
#define BSP_HWTIMER1_SOURCE_CLK     (CM_CLOCK_SOURCE_BUS)
#define BSP_HWTIMER1_ID             (0)
#define BSP_HWTIMER2_DEV            lpt_devif
#define BSP_HWTIMER2_SOURCE_CLK     (CM_CLOCK_SOURCE_LPO)
#define BSP_HWTIMER2_ID             (0)
/* Port IRQ levels */
#define BSP_PORTA_INT_LEVEL         (3)
#define BSP_PORTB_INT_LEVEL         (3)
#define BSP_PORTC_INT_LEVEL         (3)
#define BSP_PORTD_INT_LEVEL         (3)
#define BSP_PORTE_INT_LEVEL         (3)

/* GPIO settings for resistive touch screen on LCD board */
#define BSP_LCD_TCHRES_X_PLUS       (GPIO_PORT_B | GPIO_PIN3) /* open J7 7-8 */
#define BSP_LCD_TCHRES_X_MINUS      (GPIO_PORT_B | GPIO_PIN1) /* open J7 5-6 */
#define BSP_LCD_TCHRES_Y_PLUS       (GPIO_PORT_B | GPIO_PIN0) /* open J7 3-4 */
#define BSP_LCD_TCHRES_Y_MINUS      (GPIO_PORT_B | GPIO_PIN2) /* open J7 7-8 */

#define BSP_LCD_X_PLUS_ADC_CHANNEL  (ADC0_SOURCE_AD13)
#define BSP_LCD_Y_PLUS_ADC_CHANNEL  (ADC0_SOURCE_AD8)

/* ADC tchres device init struct */
#define BSP_TCHRES_ADC_DEVICE "adc0:"
#define BSP_TCHRES_X_TRIGGER ADC_PDB_TRIGGER
#define BSP_TCHRES_Y_TRIGGER ADC_PDB_TRIGGER

/* Turingsense, adding Atheros */
//TONY
/*-----------------------------------------------------------------------------
**                      Ethernet Info
*/
#if !(BSPCFG_DB142_IOT_MODE_SUPPORT)
#define BSP_ENET_DEVICE_COUNT               2 ///(MACNET_DEVICE_COUNT +(BSPCFG_ENABLE_ATHEROS_WIFI?1:0))
#else 
#define BSP_ENET_DEVICE_COUNT               1 ///(MACNET_DEVICE_COUNT +(BSPCFG_ENABLE_ATHEROS_WIFI?1:0))
#endif 

/*
** MACNET interrupt levels and vectors
*/
#define BSP_MACNET0_INT_TX_LEVEL           (4)
#define BSP_MACNET0_INT_RX_LEVEL           (3)

#define BSP_DEFAULT_ENET_DEVICE             0
#define ATHEROS_WIFI_DEFAULT_ENET_DEVICE    1
#define BSP_DEFAULT_ENET_OUI                { 0x00, 0x00, 0x5E, 0, 0, 0 }
/*
** The Ethernet PHY device number 0..31
*/
#ifndef BSP_ENET0_PHY_ADDR
    #define BSP_ENET0_PHY_ADDR                 0
#endif
/*
** PHY MII Speed (MDC - Management Data Clock)
*/
#define BSP_ENET0_PHY_MII_SPEED             (2500000L)

#ifndef BSPCFG_RX_RING_LEN
    #define BSPCFG_RX_RING_LEN              4 /* Number of receive BD's */
#endif

#ifndef BSPCFG_TX_RING_LEN
    #define BSPCFG_TX_RING_LEN              4 /* Number of transmit BD's */
#endif
#ifndef BSPCFG_ENABLE_ENET_MULTICAST
    #define BSPCFG_ENABLE_ENET_MULTICAST    1
#endif

#ifndef BSPCFG_ENABLE_ENET_STATS
    #define BSPCFG_ENABLE_ENET_STATS        1
#endif

#ifndef BSPCFG_ENET_RESTORE
    #define BSPCFG_ENET_RESTORE             1
#endif

#ifndef BSPCFG_ENABLE_CPP
    #define BSPCFG_ENABLE_CPP               0
#endif
//TONY
//TONY
/*
** The Atheros Wifi settings.
*/
#ifdef ATH_SPI_DMA    
#define BSP_ATHEROS_WIFI_SPI_DEVICE                 "dspi2:"    
#else
///e.y. change for K22D sp140
#define BSP_ATHEROS_WIFI_SPI_DEVICE                 "spi2:"
#endif
#define BSP_ATHEROS_WIFI_GPIO_INT_DEVICE                "gpio:input"
#define BSP_ATHEROS_WIFI_GPIO_INT_PIN                   (GPIO_PORT_D|GPIO_PIN_IRQ_FALLING|GPIO_PIN8)

#define BSP_ATHEROS_WIFI_GPIO_PWD_DEVICE                "gpio:write"
#define BSP_ATHEROS_WIFI_GPIO_PWD_PIN                   (GPIO_PORT_D|GPIO_PIN9) //TONY
#define BSP_ATHEROS_WIFI_GPIO_FET_PIN                   (GPIO_PORT_D|GPIO_PIN9)
#define BSP_ATHEROS_ADC_SENSOR_DEVICE "adc0:"
//TONY

/* End of Turingsense, adding Atheros */

/*-----------------------------------------------------------------------------
**                      DSPI
*/

#define BSP_DSPI_INT_LEVEL              (4)
#define BSP_SPI_MEMORY_CHANNEL          (0)

#define BSP_DSPI0_DMA_RX_CHANNEL        (0)
#define BSP_DSPI0_DMA_TX_CHANNEL        (1)
#define BSP_DSPI0_DMA_RX_SOURCE         (14)
#define BSP_DSPI0_DMA_TX_SOURCE         (15)

/*-----------------------------------------------------------------------------
**                      SDCARD
*/
#define BSP_SDCARD_ESDHC_CHANNEL            "esdhc:"
#define BSP_SDCARD_GPIO_DETECT              (GPIO_PORT_C | GPIO_PIN18)
#define BSP_SDCARD_DETECT_MUX_GPIO          (LWGPIO_MUX_C18_GPIO)
#define BSP_SDCARD_PROTECT_MUX_GPIO         (LWGPIO_MUX_C19_GPIO)


/*-----------------------------------------------------------------------------
**                      I2C
*/
#define BSP_I2C_CLOCK                       (BSP_BUS_CLOCK)
#define BSP_I2C0_ADDRESS                    (0x6E)
#define BSP_I2C1_ADDRESS                    (0x6F)
#define BSP_I2C2_ADDRESS                    (0x6F)
#define BSP_I2C0_BAUD_RATE                  (100000)
#define BSP_I2C1_BAUD_RATE                  (100000)
#define BSP_I2C2_BAUD_RATE                  (100000)
#define BSP_I2C0_MODE                       (I2C_MODE_MASTER)
#define BSP_I2C1_MODE                       (I2C_MODE_MASTER)
#define BSP_I2C2_MODE                       (I2C_MODE_MASTER)
#define BSP_I2C0_INT_LEVEL                  (5)
#define BSP_I2C1_INT_LEVEL                  (5)
#define BSP_I2C2_INT_LEVEL                  (5)
#define BSP_I2C0_RX_BUFFER_SIZE             (64)
#define BSP_I2C0_TX_BUFFER_SIZE             (64)
#define BSP_I2C1_RX_BUFFER_SIZE             (64)
#define BSP_I2C1_TX_BUFFER_SIZE             (64)
#define BSP_I2C2_RX_BUFFER_SIZE             (64)
#define BSP_I2C2_TX_BUFFER_SIZE             (64)

/*-----------------------------------------------------------------------------
**                      ADC
*/
#define ADC_MAX_MODULES                 (2)
#define BSP_ADC_POTENTIOMETER           (ADC0_SOURCE_AD12)
#define BSP_ADC_CH_POT                  (ADC0_SOURCE_AD12)
#define BSP_ADC0_VECTOR_PRIORITY        (3)
#define BSP_ADC1_VECTOR_PRIORITY        (3)
#define BSP_PDB_VECTOR_PRIORITY         (3)
#define BSP_ADC_VREF_DEFAULT            (3300)
#define BSP_ADC_POTENTIOMETER_MAX       (3300)
/*-----------------------------------------------------------------------------
**                      LWADC
*/
#define BSP_DEFAULT_LWADC_MODULE        lwadc0_init

/*-----------------------------------------------------------------------------
**                      USB
*/
#define USBCFG_MAX_DRIVERS               (2)
#define BSP_USB_INT_LEVEL                (4)
#define BSP_USB_TWR_SER2                 (0) /*set to 1 if TWR-SER2 (2 eth) board used (only host)*/

/* USB host controler for K21D processors */
#define USBCFG_KHCI                       1

#define USBCFG_DEFAULT_HOST_CONTROLLER    (&_bsp_usb_host_khci0_if)
#define USBCFG_DEFAULT_DEVICE_CONTROLLER  (&_bsp_usb_dev_khci0_if)

/*----------------------------------------------------------------------
**                  IO DEVICE DRIVERS CONFIGURATION
*/
/** MGCT: <category name="I/O Subsystem"> */

/*
** Enable I/O subsystem features in MQX. No I/O drivers and file I/O will be
** possible without this feature.
**
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_IO_SUBSYSTEM
    #define BSPCFG_ENABLE_IO_SUBSYSTEM      1
#endif
/** MGCT: </category> */


/** MGCT: <category name="Default Driver Installation in BSP startup"> */

/** MGCT: <category name="UART0 settings"> */

/*
** Polled TTY device (UART0)
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_TTYA
    #define BSPCFG_ENABLE_TTYA              1
#endif
/*
** Interrupt-driven TTY device (UART0)
** MGCT: <option type="bool"/>
*/

#ifndef BSPCFG_ENABLE_ITTYA
    #define BSPCFG_ENABLE_ITTYA             0
#endif
/*
** TTYA and ITTYA baud rate
** MGCT: <option type="number" min="0" max="115200"/>
*/

#ifndef BSPCFG_SCI0_BAUD_RATE
    #define BSPCFG_SCI0_BAUD_RATE             115200
#endif

/*
** TTYA and ITTYA buffer size
** MGCT: <option type="number" min="0" max="256"/>
*/

#ifndef BSPCFG_SCI0_QUEUE_SIZE
    #define BSPCFG_SCI0_QUEUE_SIZE             64
#endif
/** MGCT: </category> */

/** MGCT: <category name="UART1 settings"> */

/*
** Polled TTY device (UART1)
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_TTYB
    #define BSPCFG_ENABLE_TTYB              0
#endif

/*
** Interrupt-driven TTY device (UART1)
** MGCT: <option type="bool"/>
*/

#ifndef BSPCFG_ENABLE_ITTYB
    #define BSPCFG_ENABLE_ITTYB             0
#endif
/*
** TTYB and ITTYB baud rate
** MGCT: <option type="number" min="0" max="115200"/>
*/

#ifndef BSPCFG_SCI1_BAUD_RATE
    #define BSPCFG_SCI1_BAUD_RATE             115200
#endif
/*
** TTYB and ITTYB buffer size
** MGCT: <option type="number" min="0" max="256"/>
*/

#ifndef BSPCFG_SCI1_QUEUE_SIZE
    #define BSPCFG_SCI1_QUEUE_SIZE             64
#endif
/** MGCT: </category> */

/** MGCT: <category name="UART2 settings"> */

/*
** Polled TTY device (UART2)
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_TTYC
    #define BSPCFG_ENABLE_TTYC              0
#endif
/*
** Interrupt-driven TTY device (UART2)
** MGCT: <option type="bool"/>
*/

#ifndef BSPCFG_ENABLE_ITTYC
    #define BSPCFG_ENABLE_ITTYC             0
#endif
/*
** TTYC and ITTYC baud rate
** MGCT: <option type="number" min="0" max="115200"/>
*/

#ifndef BSPCFG_SCI2_BAUD_RATE
    #define BSPCFG_SCI2_BAUD_RATE             115200
#endif
/*
** TTYC and ITTYC buffer size
** MGCT: <option type="number" min="0" max="256"/>
*/

#ifndef BSPCFG_SCI2_QUEUE_SIZE
    #define BSPCFG_SCI2_QUEUE_SIZE             64
#endif
/** MGCT: </category> */

/** MGCT: <category name="UART3 settings"> */

/*
** Polled TTY device (UART3)
** MGCT: <option type="bool"/>
*/

#ifndef BSPCFG_ENABLE_TTYD
    #define BSPCFG_ENABLE_TTYD              0
#endif
/*
** Interrupt-driven TTY device (UART3)
** MGCT: <option type="bool"/>
*/

#ifndef BSPCFG_ENABLE_ITTYD
    #define BSPCFG_ENABLE_ITTYD             0
#endif
/*
** TTYD and ITTYD baud rate
** MGCT: <option type="number" min="0" max="115200"/>
*/

#ifndef BSPCFG_SCI3_BAUD_RATE
    #define BSPCFG_SCI3_BAUD_RATE             115200
#endif
/*
** TTYD and ITTYD buffer size
** MGCT: <option type="number" min="0" max="256"/>
*/

#ifndef BSPCFG_SCI3_QUEUE_SIZE
    #define BSPCFG_SCI3_QUEUE_SIZE             64
#endif

/* TTY device HW signals (UART3) */
#ifndef BSPCFG_ENABLE_TTYD_HW_SIGNALS
    #define BSPCFG_ENABLE_TTYD_HW_SIGNALS   0
#endif
/** MGCT: </category> */

/** MGCT: <category name="UART4 settings"> */

/*
** Polled TTY device (UART4)
** MGCT: <option type="bool"/>
*/

#ifndef BSPCFG_ENABLE_TTYE
    #define BSPCFG_ENABLE_TTYE              0
#endif
/*
** Interrupt-driven TTY device (UART4)
** MGCT: <option type="bool"/>
*/

#ifndef BSPCFG_ENABLE_ITTYE
    #define BSPCFG_ENABLE_ITTYE             0
#endif
/*
** TTYE and ITTYE baud rate
** MGCT: <option type="number" min="0" max="115200"/>
*/

#ifndef BSPCFG_SCI4_BAUD_RATE
    #define BSPCFG_SCI4_BAUD_RATE             115200
#endif

/*
** TTYE and ITTYE buffer size
** MGCT: <option type="number" min="0" max="256"/>
*/

#ifndef BSPCFG_SCI4_QUEUE_SIZE
    #define BSPCFG_SCI4_QUEUE_SIZE             64
#endif

/* TTY device HW signals (UART4) */
#ifndef BSPCFG_ENABLE_TTYE_HW_SIGNALS
    #define BSPCFG_ENABLE_TTYE_HW_SIGNALS   0
#endif

/** MGCT: </category> */

/** MGCT: <category name="UART5 settings"> */

/*
** Polled TTY device (UART5)
** MGCT: <option type="bool"/>
*/

#ifndef BSPCFG_ENABLE_TTYF
    #define BSPCFG_ENABLE_TTYF              0
#endif
/*
** Interrupt-driven TTY device (UART4)
** MGCT: <option type="bool"/>
*/

#ifndef BSPCFG_ENABLE_ITTYF
    #define BSPCFG_ENABLE_ITTYF             0
#endif
/*
** TTYE and ITTYE baud rate
** MGCT: <option type="number" min="0" max="115200"/>
*/

#ifndef BSPCFG_SCI5_BAUD_RATE
    #define BSPCFG_SCI5_BAUD_RATE             115200
#endif

/*
** TTYF and ITTYF buffer size
** MGCT: <option type="number" min="0" max="256"/>
*/

#ifndef BSPCFG_SCI5_QUEUE_SIZE
    #define BSPCFG_SCI5_QUEUE_SIZE             64
#endif

/* TTY device HW signals (UART4) */
#ifndef BSPCFG_ENABLE_TTYF_HW_SIGNALS
    #define BSPCFG_ENABLE_TTYF_HW_SIGNALS   0
#endif

/** MGCT: </category> */

/*
** Polled I2C0 device
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_I2C0
    #define BSPCFG_ENABLE_I2C0                  1
#endif
/*
** Interrupt-driven I2C0 device
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_II2C0
    #define BSPCFG_ENABLE_II2C0                 0
#endif
/*
** Polled I2C1 device
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_I2C1
    #define BSPCFG_ENABLE_I2C1                  0
#endif
/*
** Interrupt-driven I2C1 device
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_II2C1
    #define BSPCFG_ENABLE_II2C1                 0
#endif

/*
** GPIO device (obsolete)
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_GPIODEV
    #define BSPCFG_ENABLE_GPIODEV               0
#endif
/*
** RTC device
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_RTCDEV
    #define BSPCFG_ENABLE_RTCDEV                0
#endif

/*
** Enable SPI0 device
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_SPI0
    #define BSPCFG_ENABLE_SPI0                  0
#endif

//turingsense, don't use DM for SPI0
///*
//** Use DMA transfers on SPI0
//** MGCT: <option type="bool"/>
//*/
//#ifndef BSPCFG_DSPI0_USE_DMA
//    #define BSPCFG_DSPI0_USE_DMA                1
//#endif

/*
** Enable SPI1 device
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_SPI1
    #define BSPCFG_ENABLE_SPI1                  0
#endif

/*
** Enable SPI2 device
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_SPI2
    #define BSPCFG_ENABLE_SPI2                  0
#endif

/*
** ADC0
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_ADC0
    #define BSPCFG_ENABLE_ADC0                   0
#endif

/*
** ADC1
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_ADC1
    #define BSPCFG_ENABLE_ADC1                   0
#endif

/*
** FTFL
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_FLASHX
    #define BSPCFG_ENABLE_FLASHX                0
#endif
/*
** ESDHC
** MGCT: <option type="bool"/>
*/

#ifndef BSPCFG_ENABLE_ESDHC
    #define BSPCFG_ENABLE_ESDHC                 0
#endif

/*
** IODEBUG device
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_IODEBUG
    #define BSPCFG_ENABLE_IODEBUG               1
#endif
/** MGCT: </category> */
/*----------------------------------------------------------------------
**                  DEFAULT MQX INITIALIZATION DEFINITIONS
** MGCT: <category name="Default MQX initialization parameters">
*/

/* Defined in link.xxx */
extern unsigned char __KERNEL_DATA_START[];
extern unsigned char __KERNEL_DATA_END[];

#define BSP_DEFAULT_START_OF_KERNEL_MEMORY                    ((void *)__KERNEL_DATA_START)
#define BSP_DEFAULT_END_OF_KERNEL_MEMORY                      ((void *)__KERNEL_DATA_END)
#define BSP_DEFAULT_PROCESSOR_NUMBER                          ((uint32_t)__DEFAULT_PROCESSOR_NUMBER)

/* MGCT: <option type="string" quoted="false" allowempty="false"/> */
#ifndef BSP_DEFAULT_INTERRUPT_STACK_SIZE
    #define BSP_DEFAULT_INTERRUPT_STACK_SIZE                  ((uint32_t)__DEFAULT_INTERRUPT_STACK_SIZE)
#endif

/* MGCT: <option type="list">
** <item name="1 (all levels disabled)" value="(1L)"/>
** <item name="2" value="(2L)"/>
** <item name="3" value="(3L)"/>
** <item name="4" value="(4L)"/>
** <item name="5" value="(5L)"/>
** <item name="6" value="(6L)"/>
** <item name="7" value="(7L)"/>
** </option>
*/
#ifndef BSP_DEFAULT_MQX_HARDWARE_INTERRUPT_LEVEL_MAX
    #define BSP_DEFAULT_MQX_HARDWARE_INTERRUPT_LEVEL_MAX      (2L)
#endif
/*
** MGCT: <option type="number"/>
*/
#ifndef BSP_DEFAULT_MAX_MSGPOOLS
    #define BSP_DEFAULT_MAX_MSGPOOLS                          (2L)
#endif
/*
** MGCT: <option type="number"/>
*/
#ifndef BSP_DEFAULT_MAX_MSGQS
    #define BSP_DEFAULT_MAX_MSGQS                             (16L)
#endif

/*
 * Other Serial console options:(do not forget to enable BSPCFG_ENABLE_TTY define if changed)
 *      "ttyf:"      TWR-SER / OSJTAG-COM     polled mode
 *      "ittyf:"     TWR-SER / OSJTAG-COM  interrupt mode
 *      "iodebug:"   IDE debug console
 ** MGCT: <option type="string" maxsize="256" quoted="false" allowempty="false"/>
 */
#ifndef BSP_DEFAULT_IO_CHANNEL
    #if BSPCFG_ENABLE_TTYF
        #define BSP_DEFAULT_IO_CHANNEL                      "ttyf:"    /* OSJTAG-COM   polled mode   */
        #define BSP_DEFAULT_IO_CHANNEL_DEFINED
    #else
        #define BSP_DEFAULT_IO_CHANNEL                      NULL
    #endif
#endif
/*
** MGCT: <option type="string" maxsize="1024" quoted="false" allowempty="false"/>
*/
#ifndef BSP_DEFAULT_IO_OPEN_MODE
    #define BSP_DEFAULT_IO_OPEN_MODE                      (void *) (IO_SERIAL_XON_XOFF | IO_SERIAL_TRANSLATION | IO_SERIAL_ECHO)
#endif

/*
** FLASHX flash memory pool minimum size
** MGCT: <option type="string" maxsize="1024" quoted="false" allowempty="false"/>
*/
#if BSPCFG_ENABLE_FLASHX
    #ifndef BSPCFG_FLASHX_SIZE
        #define BSPCFG_FLASHX_SIZE      (FLASHX_END_ADDR - FLASHX_START_ADDR) /* defines maximum possible space */
    #endif
#else
    #undef  BSPCFG_FLASHX_SIZE
    #define BSPCFG_FLASHX_SIZE      0x0
#endif

/*
** FLASHX swap indicator address
** MGCT: <option type="string" maxsize="1024" quoted="false" allowempty="false"/>
*/
#if BSPCFG_ENABLE_FLASHX
    #ifndef BSPCFG_SWAP_INDICATOR_ADDR
        #define BSPCFG_SWAP_INDICATOR_ADDR      (((BSP_INTERNAL_FLASH_SIZE)/2) - (uint32_t)(__FLASHX_SECT_SIZE)) /* defines last sector of first half of flash space */
    #endif
#endif

/** MGCT: </category> */
#endif /* _twrk21f120m_h_  */
/* EOF */
