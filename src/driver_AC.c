#include "emon32_samd.h"

void
acSetup(unsigned int pin)
{
    /* Enable APB clock, connect to GCLK 3 (F_PERIPH) */
    PM->APBCMASK.reg |= PM_APBCMASK_AC;
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(AC_GCLK_ID_DIG)
                        | GCLK_CLKCTRL_GEN(3u)
                        | GCLK_CLKCTRL_CLKEN;


}