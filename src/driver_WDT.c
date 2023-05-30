#include "emon32_samd.h"

void
wdtSetup(WDT_PER_t per)
{
    /* OSCULP32 is enabled and connected to Generator 2, undivided
     * Connect Gen 2 -> WDT
     */
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(WDT_GCLK_ID)
                        | GCLK_CLKCTRL_GEN(2u)
                        | GCLK_CLKCTRL_CLKEN;

    WDT->CONFIG.reg |= per;

    /* Enable and synchronise (17.6.5) */
    WDT->CTRL.reg |= WDT_CTRL_ENABLE;
    while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);
}

void
wdtFeed()
{
    /* Write key (17.6.2.4) and synchronise (17.6.5) */
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
    while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);
}
