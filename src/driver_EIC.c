#include "emon32_samd.h"

#include "board_def.h"
#include "driver_EIC.h"
#include "driver_PORT.h"

void
eicSetup()
{
    /* EIC APB clock is unmasked on reset (16.8.8).
     * Require EIC GCLK for edge detection. (21.6.2.1)
     */
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(EIC_GCLK_ID)
                        | GCLK_CLKCTRL_GEN(3u)
                        | GCLK_CLKCTRL_CLKEN;

    /* Zero crossing sense is on {GRP,PIN}_ZEROX. Mux is "A" (Table 7-1) */
    portPinMux(GRP_ZEROX, PIN_ZEROX, PORT_PMUX_PMUXE_A_Val);

    /* Set up rising edge interrupt */
    EIC->CONFIG[1].reg =   EIC_CONFIG_FILTEN7
                         | EIC_CONFIG_SENSE7_RISE;
    EIC->INTENSET.reg = EIC_INTEN_ZEROX;
}

void
eicZeroXClr()
{
    EIC->INTFLAG.reg |= EIC_INTFLG_ZEROX;
}

int
eicZeroXStat()
{
    return (EIC->INTFLAG.reg & EIC_INTFLG_ZEROX) ? 1 : 0;
}
