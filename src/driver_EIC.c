#include "emon32_samd.h"
#include "driver_EIC.h"

void
eicSetup()
{
    /* EIC APB clock is unmasked on reset (16.8.8) */

    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(EIC_GCLK_ID)
                        | GCLK_CLKCTRL_GEN(3u)
                        | GCLK_CLKCTRL_CLKEN;
    /* Zero crossing sense is on */
}
