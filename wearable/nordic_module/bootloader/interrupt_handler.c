/* [4:51:53 PM] stefanusmantik: BranchIntHandler will check if the current mode
 * is bootloader mode or the app mode. Then call the appropriate int handler
 * (if bootloader mode) or forward to app's int handler.
 * [4:52:47 PM] stefanusmantik: DirectBranchIntHandler just directly forward the
 * interrupt to the app (since they are not used by bootloader)
 */

//static void NMI_Handler(void) ALIAS(BranchIntHandler);
//static void HardFault_Handler(void) ALIAS(BranchIntHandler);

#include "nordic_bootloader.h"
//cwati
#include "hub_timer.h"

#define ALIAS(f) __attribute__ ((alias (#f)))
  //  register void *user_program_offset asm("r2") = (void *) 0x14000;
static void NMI_Handler(void) ALIAS(BranchIntHandler);
static void HardFault_Handler(void) ALIAS(BranchIntHandler);
static void SVC_Handler(void) ALIAS(BranchIntHandler);
static void PendSV_Handler(void) ALIAS(BranchIntHandler);
static void SysTick_Handler(void) ALIAS(BranchIntHandler);
static void POWER_CLOCK_IRQHandler(void) ALIAS(BranchIntHandler);
static void RADIO_IRQHandler(void) ALIAS(BranchIntHandler);

static void SPI0_TWI0_IRQHandler(void) ALIAS(BranchIntHandler);
static void SPI1_TWI1_IRQHandler(void) ALIAS(BranchIntHandler);

static void ADC_IRQHandler(void) ALIAS(BranchIntHandler);
static void RTC0_IRQHandler(void) ALIAS(BranchIntHandler);
static void TIMER0_IRQHandler(void) ALIAS(BranchIntHandler);
static void TIMER1_IRQHandler(void) ALIAS(BranchIntHandler);
static void TIMER2_IRQHandler(void) ALIAS(BranchIntHandler);
static void TEMP_IRQHandler(void) ALIAS(BranchIntHandler);
static void RNG_IRQHandler(void) ALIAS(BranchIntHandler);
static void ECB_IRQHandler(void) ALIAS(BranchIntHandler);
static void CCM_AAR_IRQHandler(void) ALIAS(BranchIntHandler);
static void WDT_IRQHandler(void) ALIAS(BranchIntHandler);

static void QDEC_IRQHandler(void) ALIAS(BranchIntHandler);
static void LPCOMP_COMP_IRQHandler(void) ALIAS(BranchIntHandler);

static void SWI1_IRQHandler(void) ALIAS(BranchIntHandler);
static void SWI2_IRQHandler(void) ALIAS(BranchIntHandler);
static void SWI3_IRQHandler(void) ALIAS(BranchIntHandler);
static void SWI4_IRQHandler(void) ALIAS(BranchIntHandler);
static void SWI5_IRQHandler(void) ALIAS(BranchIntHandler);

static void UART0_IRQHandler(void) ALIAS(BranchIntHandler);
static void RTC1_IRQHandler(void) ALIAS(BranchIntHandler);
static void GPIOTE_IRQHandler(void) ALIAS(BranchIntHandler);
static void SWI0_IRQHandler(void) ALIAS(BranchIntHandler);

extern int main(void);

__asm void BranchIntHandler(void) {
            MRS    R0, PSR ;
            /* Mask the interrupt number only */
            MOVS   R1, #0x3F  ; 
            ANDS   R0, R1     ; /*R0 keeps the interrupt number*/
            extern bootloader_active;
            LDR R1 , =bootloader_active;
            LDR R2, [R1];
            CMP R2,#1;
            BEQ bootload;
            LDR     R2, =APPLICATION_BASE_ADDRESS;
            /* Irq address position = IRQ No * 4 */
            LSLS   R0, R0, #2 ;
            /* Fetch the user vector offset */
            LDR    R0, [R0, R2];
            /* Jump to user interrupt vector */
            BX     R0  ;
            
bootload 
            CMP R0, #0x24;
            BEQ swi0
            CMP R0, #0x21;
            BEQ rtc1
            CMP R0, #0x14;
            BEQ spi1
            //RETURN code needed
swi0            
            EXTERN SWI0_IRQHandler_Bootloader[WEAK]
						LDR    R0,= SWI0_IRQHandler_Bootloader
            B execute ;    
            
rtc1            
            EXTERN  RTC1_IRQHandler_Bootloader[WEAK] 
            LDR    R0,=RTC1_IRQHandler_Bootloader
            B execute ;
                      
spi1            
            EXTERN  SPI1_TWI1_IRQHandler_Bootloader[WEAK]     
            LDR    R0,=SPI1_TWI1_IRQHandler_Bootloader
                                 
execute            
            BX     R0  ;
            
}



//#define ALIAS(f) __attribute__ ((alias (#f)))
//  //  register void *user_program_offset asm("r2") = (void *) 0x14000;
//static void NMI_Handler(void) ALIAS(BranchIntHandler);
//static void HardFault_Handler(void) ALIAS(BranchIntHandler);
//static void SVC_Handler(void) ALIAS(BranchIntHandler);
//static void PendSV_Handler(void) ALIAS(BranchIntHandler);
//static void SysTick_Handler(void) ALIAS(BranchIntHandler);
//static void POWER_CLOCK_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void RADIO_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void UART0_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void SPI0_TWI0_IRQHandler(void) ALIAS(DirectBranchIntHandler);
////static void SPI1_TWI1_IRQHandler(void) ALIAS(DirectBranchIntHandler);
////static void GPIOTE_IRQHandler(void) ALIAS(BranchIntHandler);
//static void ADC_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void TIMER0_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void TIMER1_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void TIMER2_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void RTC0_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void TEMP_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void RNG_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void ECB_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void CCM_AAR_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void WDT_IRQHandler(void) ALIAS(DirectBranchIntHandler);
////static void RTC1_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void QDEC_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void LPCOMP_COMP_IRQHandler(void) ALIAS(DirectBranchIntHandler);
////static void SWI0_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void SWI1_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void SWI2_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void SWI3_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void SWI4_IRQHandler(void) ALIAS(DirectBranchIntHandler);
//static void SWI5_IRQHandler(void) ALIAS(DirectBranchIntHandler);



//__asm void DirectBranchIntHandler(void) {
//            MRS    R0, PSR ;
//            /* Mask the interrupt number only */
//            MOVS   R1, #0x3F  ; 
//            ANDS   R0, R1     ; /*R0 keeps the interrupt number*/

//            LDR     R2, =APPLICATION_BASE_ADDRESS;
//            /* Irq address position = IRQ No * 4 */
//            LSLS   R0, R0, #2 ;
//            /* Fetch the user vector offset */
//            LDR    R0, [R0, R2];
//            /* Jump to user interrupt vector */
//            BX     R0  ;
//            ALIGN
//}

//__asm void BranchIntHandler(void) {
//            MRS    R0, PSR ;
//            /* Mask the interrupt number only */
//            MOVS   R1, #0x3F  ; 
//            ANDS   R0, R1     ; /*R0 keeps the interrupt number*/
//            extern bootloader_active;
//            LDR R1 , =bootloader_active;
//            LDR R2, [R1];
//            CMP R2,#1;
//            BEQ bootload;
//            LDR     R2, =APPLICATION_BASE_ADDRESS;
//            /* Irq address position = IRQ No * 4 */
//            LSLS   R0, R0, #2 ;
//            /* Fetch the user vector offset */
//            LDR    R0, [R0, R2];
//            /* Jump to user interrupt vector */
//            BX     R0  ;
//            
//bootload 
//            LDR     R2, =BOOTLOADER_ADDRESS;
//            /* Irq address position = IRQ No * 4 */
//            LSLS   R0, R0, #2 ;
//            /* Fetch the user vector offset */
//            LDR    R0, [R0, R2];
//            /* Jump to user interrupt vector */
//            BX     R0  ;

//            ALIGN
//            
//}
