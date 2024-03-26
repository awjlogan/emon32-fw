#include "emon32_samd.h"
#include "driver_SAMD.h"

void
clkSetup(void)
{
    /* Set up brown out detector to catch:
     *   - Brown in at startup before setting waiting states @ 48 MHz
           Suspend setting up the 48 MHz clock until the power is established
     *   - Brown out trigger reset
     * https://blog.thea.codes/sam-d21-brown-out-detector/
     */

    /* Disable BOD during configuration to avoid spurious reset */
    SYSCTRL->BOD33.reg &= ~SYSCTRL_BOD33_ENABLE;
    while(!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_B33SRDY));

    /* Configure BOD to watch the system voltage @3V3
     *   - Vmin = Vmin = 2V77 - 2V84 (Table 37-21)
     *   - Take no action
     *   - Enable hysteresis
     */
    SYSCTRL->BOD33.reg =   SYSCTRL_BOD33_LEVEL(39)
                         | SYSCTRL_BOD33_ACTION_NONE
                         | SYSCTRL_BOD33_HYST;

    SYSCTRL->BOD33.reg |= SYSCTRL_BOD33_ENABLE;
    while(!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_B33SRDY));

    /* PCLKSR.BOD33DET is 1 when the voltage is too low */
    while (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_BOD33DET);

    /* Now at ~3V3, set BOD33 to reset the micro on brown out */
    SYSCTRL->BOD33.reg |= SYSCTRL_BOD33_ACTION_RESET;

    /* 48 MHz DFLL for core - adapted from Arduino core library
     *  1. Set flash wait state to 1 for 48 MHz core
     *  2. Enable OSC32K clock (assuming no external crystal)
     *  3. Set GCLK Gen 1 source as OSC32K
     *  4. Set GCLK Gen 1 as source for GCLK Multiplexor 0 (DFLL48M reference)
     *  5. Enable DFLL48M CLK
     *  6. Switch GCLK Gen 0 to DFLL48M - core will run at 48 MHz
     */

    /* 1. Flash wait */
    NVMCTRL->CTRLB.bit.RWS = NVMCTRL_CTRLB_RWS_HALF_Val;

    /* 2. OSC32K setup */
    SYSCTRL->OSC32K.reg =   SYSCTRL_OSC32K_CALIB(samdCalibration(CAL_OSC32K))
                          | SYSCTRL_OSC32K_STARTUP(0x6u)
                          | SYSCTRL_OSC32K_EN32K
                          | SYSCTRL_OSC32K_ENABLE;
    while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC32KRDY));

    /* Reset clock system; 32K sources are not affected (Section 14.7)
     * CTRL.SWRST and STATUS.SYNCBUSY will be cleared simultaneously (Section 15.8.1)
     */
    GCLK->CTRL.reg = GCLK_CTRL_SWRST;
    while ((GCLK->CTRL.reg & GCLK_CTRL_SWRST) && (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY));

    /* 3. OSC32K -> generator 1 */
    GCLK->GENCTRL.reg =   GCLK_GENCTRL_ID(1u)
                        | GCLK_GENCTRL_SRC_OSC32K
                        | GCLK_GENCTRL_GENEN;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    /* Gen 1 -> Mux 0 */
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(0u)
                        | GCLK_CLKCTRL_GEN_GCLK1
                        | GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    /* 4. Enable DFLL48M in closed loop mode (Section 17.6.7.1) */
    /* ERRATA 9905: DFLL48 must be requested before configuration */
    SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
    while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));
    SYSCTRL->DFLLMUL.reg =   SYSCTRL_DFLLMUL_CSTEP(31)
                           | SYSCTRL_DFLLMUL_FSTEP(511)
                           | SYSCTRL_DFLLMUL_MUL((48000000ul + 32768ul / 2) / 32768ul);
    while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

    const uint32_t NVM_SW_CALIB_DFLL48M_COARSE_VAL = 58;
    uint32_t coarse =   (*((uint32_t *)(NVMCTRL_OTP4) + (NVM_SW_CALIB_DFLL48M_COARSE_VAL / 32)) >> (NVM_SW_CALIB_DFLL48M_COARSE_VAL % 32))
                      & ((1<<6) - 1);
    if (0x3f == coarse) coarse = 0x1f;
    const uint32_t fine = 0x1ff;
    SYSCTRL->DFLLVAL.bit.COARSE = coarse;
    SYSCTRL->DFLLVAL.bit.FINE = fine;

    SYSCTRL->DFLLMUL.reg =   SYSCTRL_DFLLMUL_CSTEP(0x1f / 4)
                           | SYSCTRL_DFLLMUL_FSTEP(10)
                           | SYSCTRL_DFLLMUL_MUL(48000);
    SYSCTRL->DFLLCTRL.reg = 0;
    while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

    SYSCTRL->DFLLCTRL.reg =   SYSCTRL_DFLLCTRL_MODE
                            | SYSCTRL_DFLLCTRL_CCDIS
                            | SYSCTRL_DFLLCTRL_BPLCKC;
    while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));
    SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_ENABLE;
    while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

    /* 5. DFLL48M -> generator 0 */
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(0u);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    GCLK->GENCTRL.reg =   GCLK_GENCTRL_ID(0u)
                        | GCLK_GENCTRL_SRC_DFLL48M
                        | GCLK_GENCTRL_IDC
                        | GCLK_GENCTRL_GENEN;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    /* OSC8M -> 8 MHz, connect to generator 3 */
    SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC_0_Val;

    GCLK->GENCTRL.reg =   GCLK_GENCTRL_ID(3u)
                        | GCLK_GENCTRL_SRC_OSC8M
                        | GCLK_GENCTRL_GENEN;
    while (GCLK->STATUS.bit.SYNCBUSY);
}
