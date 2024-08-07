/*
 * Copyright (c) 2015, Alex Taradov <alex@taradov.com>
 *               2024, Angus Logan <awjlogan@gmail.com>
 * All rights reserved.
 *
 * 3-Clause BSD license (https://opensource.org/license/BSD-3-clause)
 */

//-----------------------------------------------------------------------------
#include "samd21.h"

//-----------------------------------------------------------------------------
#define DUMMY __attribute__((weak, alias("irq_handler_dummy")))

//-----------------------------------------------------------------------------
void       irq_handler_reset(void);
DUMMY void irq_handler_nmi(void);
DUMMY void irq_handler_hard_fault(void);
DUMMY void irq_handler_sv_call(void);
DUMMY void irq_handler_pend_sv(void);
DUMMY void irq_handler_sys_tick(void);

DUMMY void irq_handler_pm(void);
DUMMY void irq_handler_sysctrl(void);
DUMMY void irq_handler_wdt(void);
DUMMY void irq_handler_rtc(void);
DUMMY void irq_handler_eic(void);
DUMMY void irq_handler_nvmctrl(void);
DUMMY void irq_handler_dmac(void);
DUMMY void irq_handler_usb(void);
DUMMY void irq_handler_evsys(void);
DUMMY void irq_handler_sercom0(void);
DUMMY void irq_handler_sercom1(void);
DUMMY void irq_handler_sercom2(void);
DUMMY void irq_handler_sercom3(void);
DUMMY void irq_handler_sercom4(void);
DUMMY void irq_handler_sercom5(void);
DUMMY void irq_handler_tcc0(void);
DUMMY void irq_handler_tcc1(void);
DUMMY void irq_handler_tcc2(void);
DUMMY void irq_handler_tc3(void);
DUMMY void irq_handler_tc4(void);
DUMMY void irq_handler_tc5(void);
DUMMY void irq_handler_tc6(void);
DUMMY void irq_handler_tc7(void);
DUMMY void irq_handler_adc(void);
DUMMY void irq_handler_ac(void);
DUMMY void irq_handler_dac(void);
DUMMY void irq_handler_ptc(void);
DUMMY void irq_handler_i2s(void);

extern int main(void);

extern void         _stack_top(void);
extern unsigned int _etext;
extern unsigned int _data;
extern unsigned int _edata;
extern unsigned int _bss;
extern unsigned int _ebss;

//-----------------------------------------------------------------------------
__attribute__((used, section(".vectors"))) void (*const vectors[])(void) = {
    &_stack_top, // 0 - Initial Stack Pointer Value

    // Cortex-M0+ handlers
    irq_handler_reset,      // 1 - Reset
    irq_handler_nmi,        // 2 - NMI
    irq_handler_hard_fault, // 3 - Hard Fault
    0,                      // 4 - Reserved
    0,                      // 5 - Reserved
    0,                      // 6 - Reserved
    0,                      // 7 - Reserved
    0,                      // 8 - Reserved
    0,                      // 9 - Reserved
    0,                      // 10 - Reserved
    irq_handler_sv_call,    // 11 - SVCall
    0,                      // 12 - Reserved
    0,                      // 13 - Reserved
    irq_handler_pend_sv,    // 14 - PendSV
    irq_handler_sys_tick,   // 15 - SysTick

    // Peripheral handlers
    irq_handler_pm,      // 0 - Power Manager
    irq_handler_sysctrl, // 1 - System Controller
    irq_handler_wdt,     // 2 - Watchdog Timer
    irq_handler_rtc,     // 3 - Real Time Counter
    irq_handler_eic,     // 4 - External Interrupt Controller
    irq_handler_nvmctrl, // 5 - Non-Volatile Memory Controller
    irq_handler_dmac,    // 6 - Direct Memory Access Controller
    irq_handler_usb,     // 7 - Universal Serial Bus Controller
    irq_handler_evsys,   // 8 - Event System
    irq_handler_sercom0, // 9 - Serial Communication Interface 0
    irq_handler_sercom1, // 10 - Serial Communication Interface 1
    irq_handler_sercom2, // 11 - Serial Communication Interface 2
    irq_handler_sercom3, // 12 - Serial Communication Interface 3
    irq_handler_sercom4, // 13 - Serial Communication Interface 4
    irq_handler_sercom5, // 14 - Serial Communication Interface 5
    irq_handler_tcc0,    // 15 - Timer/Counter for Control 0
    irq_handler_tcc1,    // 16 - Timer/Counter for Control 1
    irq_handler_tcc2,    // 17 - Timer/Counter for Control 2
    irq_handler_tc3,     // 18 - Timer/Counter 3
    irq_handler_tc4,     // 19 - Timer/Counter 4
    irq_handler_tc5,     // 20 - Timer/Counter 5
    irq_handler_tc6,     // 21 - Timer/Counter 6
    irq_handler_tc7,     // 22 - Timer/Counter 7
    irq_handler_adc,     // 23 - Analog-to-Digital Converter
    irq_handler_ac,      // 24 - Analog Comparator
    irq_handler_dac,     // 25 - Digital-to-Analog Converter
    irq_handler_ptc,     // 26 - Peripheral Touch Controller
    irq_handler_i2s,     // 27 - Inter-IC Sound Interface
};

//-----------------------------------------------------------------------------
void irq_handler_reset(void) {
  unsigned int *src, *dst;

  src = &_etext;
  dst = &_data;
  while (dst < &_edata)
    *dst++ = *src++;

  dst = &_bss;
  while (dst < &_ebss)
    *dst++ = 0;

  SCB->VTOR = (uint32_t)vectors;

  /* Change default QOS values to have the best performance and correct USB
   * behaviour.
   */
  SBMATRIX->SFR[SBMATRIX_SLAVE_HMCRAMC0].reg = 2;
#if defined(ID_USB)
  USB->DEVICE.QOSCTRL.bit.CQOS = 2;
  USB->DEVICE.QOSCTRL.bit.DQOS = 2;
#endif
  DMAC->QOSCTRL.bit.DQOS   = 2;
  DMAC->QOSCTRL.bit.FQOS   = 2;
  DMAC->QOSCTRL.bit.WRBQOS = 2;

  NVMCTRL->CTRLB.bit.MANW = 1;

  main();

  while (1)
    ;
}

//-----------------------------------------------------------------------------
void irq_handler_dummy(void) {
  while (1)
    ;
}

//-----------------------------------------------------------------------------
void _exit(int status) {
  (void)status;
  while (1)
    ;
}
