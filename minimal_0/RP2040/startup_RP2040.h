#include <stdint.h>

#include "RP2040/RP2040.h"

#ifndef XOSC_VALUE
#warning XOSC VALUE DEFAULT = 12Mhz
#define XOSC_VALUE   (12000000UL) // Oscillator frequency
#endif


void notmain(void) __attribute__((weak, __noreturn__));
extern int main();
void _exit(int i) __attribute__((weak));

#define SRAM_START  0x20000000U                     // SRAM start address
#define SRAM_SIZE   (264U * 1024U)                  // SRAM size (264K)
#define SRAM_END    ((SRAM_START) + (SRAM_SIZE))    // SRAM end address

#define __StackTop   (void*)SRAM_END            // stack init value

void isr_nmi(void) __attribute__((weak));
void isr_hardfault(void) __attribute__((weak));
void isr_svcall(void) __attribute__((weak));
void isr_pendsv(void) __attribute__((weak));
void isr_systick(void) __attribute__((weak));
void _default_isr_handler(void) __attribute__((weak));
void isr_invalid(void) __attribute__((weak));

// Reset Handler called on controller reset
// - initialises .data
// - clears .bss
// - calls runtime_init
// - calls main
// - calls exit (which should eventually hang the processor via _exit)
void _reset_handler(void) __attribute__((weak, __noreturn__));

void TIMER_IRQ_0_Handler(void)   __attribute__((weak, alias("_default_isr_handler")));
void TIMER_IRQ_1_Handler(void)   __attribute__((weak, alias("_default_isr_handler")));
void TIMER_IRQ_2_Handler(void)   __attribute__((weak, alias("_default_isr_handler")));
void TIMER_IRQ_3_Handler(void)   __attribute__((weak, alias("_default_isr_handler")));
void PWM_IRQ_WRAP_Handler(void)  __attribute__((weak, alias("_default_isr_handler")));
void USBCTRL_IRQ_Handler(void)   __attribute__((weak, alias("_default_isr_handler")));
void XIP_IRQ_Handler(void)       __attribute__((weak, alias("_default_isr_handler")));
void PIO0_IRQ_0_Handler(void)    __attribute__((weak, alias("_default_isr_handler")));
void PIO0_IRQ_1_Handler(void)    __attribute__((weak, alias("_default_isr_handler")));
void PIO1_IRQ_0_Handler(void)    __attribute__((weak, alias("_default_isr_handler")));
void PIO1_IRQ_1_Handler(void)    __attribute__((weak, alias("_default_isr_handler")));
void DMA_IRQ_0_Handler(void)     __attribute__((weak, alias("_default_isr_handler")));
void DMA_IRQ_1_Handler(void)     __attribute__((weak, alias("_default_isr_handler")));
void IO_IRQ_BANK0_Handler(void)  __attribute__((weak, alias("_default_isr_handler")));
void IO_IRQ_QSPI_Handler(void)   __attribute__((weak, alias("_default_isr_handler")));
void SIO_IRQ_PROC0_Handler(void) __attribute__((weak, alias("_default_isr_handler")));
void SIO_IRQ_PROC1_Handler(void) __attribute__((weak, alias("_default_isr_handler")));
void CLOCKS_IRQ_Handler(void)    __attribute__((weak, alias("_default_isr_handler")));
void SPI0_IRQ_Handler(void)      __attribute__((weak, alias("_default_isr_handler")));
void SPI1_IRQ_Handler(void)      __attribute__((weak, alias("_default_isr_handler")));
void UART0_IRQ_Handler(void)     __attribute__((weak, alias("_default_isr_handler")));
void UART1_IRQ_Handler(void)     __attribute__((weak, alias("_default_isr_handler")));
void ADC_IRQ_FIFO_Handler(void)  __attribute__((weak, alias("_default_isr_handler")));
void I2C0_IRQ_Handler(void)      __attribute__((weak, alias("_default_isr_handler")));
void I2C1_IRQ_Handler(void)      __attribute__((weak, alias("_default_isr_handler")));
void RTC_IRQ_Handler(void)       __attribute__((weak, alias("_default_isr_handler")));


#define isr_irq0    TIMER_IRQ_0_Handler   //   0  TIMER_IRQ_0
#define isr_irq1    TIMER_IRQ_1_Handler   //   1  TIMER_IRQ_1
#define isr_irq2    TIMER_IRQ_2_Handler   //   2  TIMER_IRQ_2
#define isr_irq3    TIMER_IRQ_3_Handler   //   3  TIMER_IRQ_3
#define isr_irq4    PWM_IRQ_WRAP_Handler  //   4  PWM_IRQ_WRAP
#define isr_irq5    USBCTRL_IRQ_Handler   //   5  USBCTRL_IRQ
#define isr_irq6    XIP_IRQ_Handler       //   6  XIP_IRQ
#define isr_irq7    PIO0_IRQ_0_Handler    //   7  PIO0_IRQ_0
#define isr_irq8    PIO0_IRQ_1_Handler    //   8  PIO0_IRQ_1
#define isr_irq9    PIO1_IRQ_0_Handler    //   9  PIO1_IRQ_0
#define isr_irq10   PIO1_IRQ_1_Handler    //   10 PIO1_IRQ_1
#define isr_irq11   DMA_IRQ_0_Handler     //   11 DMA_IRQ_0
#define isr_irq12   DMA_IRQ_1_Handler     //   12 DMA_IRQ_1
#define isr_irq13   IO_IRQ_BANK0_Handler  //   13 IO_IRQ_BANK0
#define isr_irq14   IO_IRQ_QSPI_Handler   //   14 IO_IRQ_QSPI
#define isr_irq15   SIO_IRQ_PROC0_Handler //   15 SIO_IRQ_PROC0
#define isr_irq16   SIO_IRQ_PROC1_Handler //   16 SIO_IRQ_PROC1
#define isr_irq17   CLOCKS_IRQ_Handler    //   17 CLOCKS_IRQ
#define isr_irq18   SPI0_IRQ_Handler      //   18 SPI0_IRQ
#define isr_irq19   SPI1_IRQ_Handler      //   19 SPI1_IRQ
#define isr_irq20   UART0_IRQ_Handler     //   20 UART0_IRQ
#define isr_irq21   UART1_IRQ_Handler     //   21 UART1_IRQ
#define isr_irq22   ADC_IRQ_FIFO_Handler  //   22 ADC_IRQ_FIFO
#define isr_irq23   I2C0_IRQ_Handler      //   23 I2C0_IRQ
#define isr_irq24   I2C1_IRQ_Handler      //   24 I2C1_IRQ
#define isr_irq25   RTC_IRQ_Handler       //   25 RTC_IRQ
#define isr_irq26   0
#define isr_irq27   0
#define isr_irq28   0
#define isr_irq29   0
#define isr_irq30   0
#define isr_irq31   0


const void* __Vectors[48] __attribute__((used, section(".vectors"))) = {
    __StackTop,     //      Initial Stack Pointer
    _reset_handler, //      Reset Handler
    isr_nmi,        //  -14 NMI Handler
    isr_hardfault,  //  -13 Hard Fault Handler
    isr_invalid,    //      Reserved, should never fire
    isr_invalid,    //      Reserved, should never fire
    isr_invalid,    //      Reserved, should never fire
    isr_invalid,    //      Reserved, should never fire
    isr_invalid,    //      Reserved, should never fire
    isr_invalid,    //      Reserved, should never fire
    isr_invalid,    //      Reserved, should never fire
    isr_svcall,     //  -5  SVCall Handler
    isr_invalid,    //      Reserved, should never fire
    isr_invalid,    //      Reserved, should never fire
    isr_pendsv,     //  -2  PendSV Handler
    isr_systick,    //  -1  SysTick Handler
    isr_irq0,       //  0   TIMER_IRQ_0
    isr_irq1,       //  1   TIMER_IRQ_1
    isr_irq2,       //  2   TIMER_IRQ_2
    isr_irq3,       //  3   TIMER_IRQ_3
    isr_irq4,       //  4   PWM_IRQ_WRAP
    isr_irq5,       //  5   USBCTRL_IRQ
    isr_irq6,       //  6   XIP_IRQ
    isr_irq7,       //  7   PIO0_IRQ_0
    isr_irq8,       //  8   PIO0_IRQ_1
    isr_irq9,       //  9   PIO1_IRQ_0
    isr_irq10,      //  10  PIO1_IRQ_1
    isr_irq11,      //  11  DMA_IRQ_0
    isr_irq12,      //  12  DMA_IRQ_1
    isr_irq13,      //  13  IO_IRQ_BANK0
    isr_irq14,      //  14  IO_IRQ_QSPI
    isr_irq15,      //  15  SIO_IRQ_PROC0
    isr_irq16,      //  16  SIO_IRQ_PROC1
    isr_irq17,      //  17  CLOCKS_IRQ
    isr_irq18,      //  18  SPI0_IRQ
    isr_irq19,      //  19  SPI1_IRQ
    isr_irq20,      //  20  UART0_IRQ
    isr_irq21,      //  21  UART1_IRQ
    isr_irq22,      //  22  ADC_IRQ_FIFO
    isr_irq23,      //  23  I2C0_IRQ
    isr_irq24,      //  24  I2C1_IRQ
    isr_irq25,      //  25  RTC_IRQ
    isr_irq26,
    isr_irq27,
    isr_irq28,
    isr_irq29,
    isr_irq30,
    isr_irq31
};


__attribute__((weak))
void isr_invalid(void) {
    while (1);
} // should never fire

__attribute__((weak))
void isr_nmi(void) {
    while (1);
}

__attribute__((weak))
void isr_hardfault(void) {
    while (1);
}

__attribute__((weak))
void isr_svcall(void) {
    while (1);
}

__attribute__((weak))
void isr_pendsv(void) {
    while (1);
}

__attribute__((weak))
void isr_systick(void) {
    while (1);
}

__attribute__((weak))
void _default_isr_handler(void) {
    while (1);
}


uint32_t SystemCoreClock = XOSC_VALUE; // System Core Clock Frequency

#define CLK_REF_SRC_ROSC 0x00
#define CLK_REF_SRC_AUX  0x01
#define CLK_REF_SRC_XOSC 0x02


__attribute__((weak))
void SystemCoreClockUpdate(void) {

    uint32_t clock_source = CLOCKS->CLK_REF_CTRL;
    uint32_t ref_div;
    uint32_t fb_div;
    uint32_t postdiv1;
    uint32_t postdiv2;

    switch (clock_source) {

        case CLK_REF_SRC_ROSC:
            // We can't directly calculate ROSC frequency, we have to use other stable source
            // In this case we use known frequency of XOSC
            CLOCKS->FC0_REF_KHZ  = XOSC_VALUE / 1000;
            CLOCKS->FC0_MIN_KHZ  = 0;
            CLOCKS->FC0_MAX_KHZ  = 0;
            CLOCKS->FC0_INTERVAL = 4;    // 1us * 2 * value
            // Since this gets more complicated we don't support this source now
            break;

        case CLK_REF_SRC_XOSC:
            ref_div = PLL_SYS->CS & PLL_SYS_CS_REFDIV_Msk;
            if (PLL_SYS->CS & PLL_SYS_CS_BYPASS_Msk) {
                SystemCoreClock = XOSC_VALUE / ref_div;
            } else {
                // Calculation from datasheet
                // (FREF / REFDIV) × FBDIV / (POSTDIV1 × POSTDIV2)
                fb_div          = PLL_SYS->FBDIV_INT;
                postdiv1        = (PLL_SYS->PRIM & PLL_SYS_PRIM_POSTDIV1_Msk) >> PLL_SYS_PRIM_POSTDIV1_Pos;
                postdiv2        = (PLL_SYS->PRIM & PLL_SYS_PRIM_POSTDIV2_Msk) >> PLL_SYS_PRIM_POSTDIV2_Pos;
                SystemCoreClock = (XOSC_VALUE / ref_div) * fb_div / (postdiv1 * postdiv2);
            }
            break;

        case CLK_REF_SRC_AUX:
            break;
    }
}


__attribute__((weak))
void SystemInit(void) {

    #if defined(__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)
    SCB->VTOR = (uint32_t)&(__Vectors);
    #endif

    // We only want to set XOSC to run on our board
    // Setup XOSC range
    XOSC->CTRL = 0xaa0; // 1_15Mhz
    // Setup startup delay
    uint32_t startup_delay = ((XOSC_VALUE / 1000) + 128) / 256;
    XOSC->STARTUP = startup_delay;
    // Enable XOSC
    XOSC->CTRL = (XOSC->CTRL & ~XOSC_CTRL_ENABLE_Msk) | (0xFAB << XOSC_CTRL_ENABLE_Pos);

    while (!(XOSC->STATUS & XOSC_STATUS_ENABLED_Msk));

    // // First bypass PLL
    // RESETS->RESET |= RESETS_RESET_pll_sys_Msk;
    // RESETS->RESET &= ~RESETS_RESET_pll_sys_Msk;
    // while (!(RESETS->RESET_DONE & RESETS_RESET_DONE_pll_sys_Msk));
    // PLL_SYS->PWR = 0x0;
    // PLL_SYS->CS |= PLL_SYS_CS_BYPASS_Msk;
    // while (!(PLL_SYS->CS & PLL_SYS_CS_BYPASS_Msk));
    // Change from using PLL to CLK_REF
    CLOCKS->CLK_REF_CTRL = 0x2 << CLOCKS_CLK_REF_CTRL_SRC_Pos;

    CLOCKS->CLK_SYS_CTRL = ((CLOCKS->CLK_SYS_CTRL) & (~(CLOCKS_CLK_SYS_CTRL_SRC_Msk))) | 0 ;
    CLOCKS->CLK_SYS_CTRL = ((CLOCKS->CLK_SYS_CTRL) & (~(CLOCKS_CLK_SYS_CTRL_AUXSRC_Msk))) | (0x03 << CLOCKS_CLK_SYS_CTRL_AUXSRC_Pos) ;
    CLOCKS->CLK_SYS_CTRL = ((CLOCKS->CLK_SYS_CTRL) & (~(CLOCKS_CLK_SYS_CTRL_SRC_Msk))) | 0x01 << CLOCKS_CLK_SYS_CTRL_SRC_Pos ;

    // Disable all clocks
    CLOCKS->ENABLED0 = 0x0;
    CLOCKS->ENABLED1 = 0x0;

    RESETS->RESET |=  (RESETS_RESET_pads_bank0_Msk);
    RESETS->RESET &= ~(RESETS_RESET_pads_bank0_Msk);
    while (!(RESETS->RESET_DONE & RESETS_RESET_DONE_pads_bank0_Msk));

    RESETS->RESET |=  (RESETS_RESET_io_bank0_Msk);
    RESETS->RESET &= ~(RESETS_RESET_io_bank0_Msk);

    while (!(RESETS->RESET_DONE & RESETS_RESET_DONE_io_bank0_Msk));
}


// which should eventually hang the processor via _exit
__attribute__((weak))
void _exit(int i __attribute__((unused))) {
    while (1);
}

__attribute__((weak, __noreturn__))
void _reset_handler(void) {
    SystemInit();
    SystemCoreClockUpdate();
    __PROGRAM_START(); /* Enter PreMain (C library entry point) */
}

__attribute__((weak, __noreturn__))
void notmain(void) {
    SystemInit();
    SystemCoreClockUpdate();

    typedef struct {
        uint32_t const* src;
        uint32_t* dest;
        uint32_t  wlen;
    } __copy_table_t;

    typedef struct {
        uint32_t* dest;
        uint32_t  wlen;
    } __zero_table_t;

    extern const __copy_table_t __copy_table_start__;
    extern const __copy_table_t __copy_table_end__;
    extern const __zero_table_t __zero_table_start__;
    extern const __zero_table_t __zero_table_end__;

    for (__copy_table_t const* pTable = &__copy_table_start__; pTable < &__copy_table_end__; ++pTable) {
        for(uint32_t i=0u; i<pTable->wlen; ++i) {
            pTable->dest[i] = pTable->src[i];
        }
    }

    for (__zero_table_t const* pTable = &__zero_table_start__; pTable < &__zero_table_end__; ++pTable) {
        for(uint32_t i=0u; i<pTable->wlen; ++i) {
            pTable->dest[i] = 0u;
        }
    }

    main();
    for(;;);
}




#if 0

//  HW defs
#define SRAM_START  0x20000000U                     // SRAM start address
#define SRAM_SIZE   (264U * 1024U)                  // SRAM size (264K)
#define SRAM_END    ((SRAM_START) + (SRAM_SIZE))    // SRAM end address
#define STACK_TOP   SRAM_END                        // stack init value

extern uint32_t _la_data;       // start address of .data segment in FLASH
extern uint32_t _sdata;         // start address of .data segment in SRAM
extern uint32_t _edata;         //   end address of .data segment in SRAM
extern uint32_t _sbss;          // start address of .bss segment in SRAM
extern uint32_t _ebss;          //   end address of .bss segment in SRAM

#define DATA_SIZE (((uint32_t)&_edata) - ((uint32_t)&_sdata))   // .data segment size
#define BSS_SIZE  (((uint32_t)&_ebss) - ((uint32_t)&_sbss))     // .bss segment size

// _reset_handler
// copy .data segment from FLASH to SRAM

uint32_t *p_src = (uint32_t*)&_la_data;
uint32_t *p_dst = (uint32_t*)&_sdata;

for (uint32_t i = 0; i < DATA_SIZE; i++) { *p_dst++ = *p_src++; }

// initialize .bss segment to 0
p_dst = (uint32_t*)&_sbss;
for (uint32_t i = 0; i < BSS_SIZE; i++) { *p_dst++ = 0x00000000; }

main();



// Link

ENTRY(_reset_handler)

MEMORY {

    FLASH(rx) : ORIGIN = 0x10000000, LENGTH = 2048k
    SRAM(rwx) : ORIGIN = 0x20000000, LENGTH = 264k
}

SECTIONS {
    .flash_begin : {
        __flash_binary_start = .;
    } > FLASH

    .boot2 : {
        __boot2_start__ = .;
        KEEP (*(.boot2))
        __boot2_end__ = .;
    } > FLASH

    ASSERT(__boot2_end__ - __boot2_start__ == 256,
           "ERROR: Pico second stage bootloader must be 256 bytes in size")


    .text : {

        *(.vector_table)
        *(.text)
        *(.rodata)
        . = ALIGN(4);

        _la_data = .;
    } > FLASH

    .data : {

        _sdata = .;
        *(.data*)
        _edata = .;
        . = ALIGN(4);

    } > SRAM AT> FLASH

    .bss  : {

        _sbss = .;
        *(.bss*)
        _ebss = .;
        . = ALIGN(4);

    } > SRAM
}



#endif
