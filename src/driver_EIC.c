#include "emon32_samd.h"

void
eicSetup()
{
    /* APB bus clock is enabled at reset, connect to GCLK 3 */
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(EIC_GCLK_ID)
                        | GCLK_CLKCTRL_GEN(3u)
                        | GCLK_CLKCTRL_CLKEN;

    EIC->CONFIG[0].reg |=   PULSE_EIC_FILTER
                          | PULSE_EIC_RISING;
}

void
eicEnablePin()
{
    EIC->CTRL.reg |= EIC_CTRL_ENABLE;
}


void
eicDisablePin()
{
    EIC->CTRL.reg &= ~(EIC_CTRL_ENABLE);
}


void
irq_handler_eic()
{
    emon32SetEvent(EVT_EIC_PULSE);
    EIC->INTFLAG.reg = PULSE_EIC_INTFLAG;
}