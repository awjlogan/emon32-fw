#include "emon32_samd.h"

void (*tc2_cb)();

void
timerSetup()
{
    /* SysTick is used as the general purpose 1 kHz timer. The tick value is
     * reloaded on underflow SysTick is part of CMSIS so should be portable
     * across Cortex-M cores
     */
    const uint32_t tickkHz = (F_CORE / 1000u) - 1u;
    SysTick_Config(tickkHz);

    /* TC1 is used to trigger ADC sampling at constant rate */
    /* Enable APB clock, set TC1 to generator 3 (OSC8M @ F_PERIPH)  */
    PM->APBCMASK.reg |= PM_APBCMASK_TC1;
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(TC1_GCLK_ID)
                        | GCLK_CLKCTRL_GEN(3u)
                        | GCLK_CLKCTRL_CLKEN;

    /* Configure as 8bit counter (F_PERIPH - /8 -> F_TC1 */
    TC1->COUNT8.CTRLA.reg =   TC_CTRLA_MODE_COUNT8
                            | TC_CTRLA_PRESCALER_DIV8
                            | TC_CTRLA_PRESCSYNC_RESYNC;

    /* TC1 overflow event output to trigger ADC */
    TC1->COUNT8.EVCTRL.reg |= TC_EVCTRL_OVFEO;

    /* TC1 is running at 1 MHz, each tick is 1 us
     * PER, COUNT, and Enable require synchronisation (28.6.6)
     */
    const unsigned int cntPer = F_TC1 / SAMPLE_RATE / (VCT_TOTAL);
    TC1->COUNT8.PER.reg = (uint8_t)cntPer;
    while (TC1->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY);
    TC1->COUNT8.COUNT.reg = 0u;
    while (TC1->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY);
    TC1->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TC1->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY);

    /* TC2 is used as the delay and elapsed time counter
     * Enable APB clock, set TC1 to generator 3 @ F_PERIPH
     * Enable the interrupt for Compare Match, do not route to NVIC
     */
    PM->APBCMASK.reg |= PM_APBCMASK_TC2;
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(TC2_GCLK_ID)
                        | GCLK_CLKCTRL_GEN(3u)
                        | GCLK_CLKCTRL_CLKEN;
    TC2->COUNT32.CTRLA.reg =   TC_CTRLA_MODE_COUNT32
                             | TC_CTRLA_PRESCALER_DIV8
                             | TC_CTRLA_PRESCSYNC_RESYNC;
}

void
commonSetup(uint32_t delay)
{
    /* Unmask match interrrupt, zero counter, set compare value */
    TC2->COUNT32.INTENSET.reg |= TC_INTENSET_MC0;
    TC2->COUNT32.COUNT.reg = 0u;
    while (TC2->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
    TC2->COUNT32.CC[0].reg = delay;
    while (TC2->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
    TC2->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TC2->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
}

void
timerDisable()
{
    TC2->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    NVIC_DisableIRQ(TC2_IRQn);
}

int
timerDelayNB_us(uint32_t delay, void (*cb)())
{
    tc2_cb = cb;
    if (TC2->COUNT32.CTRLA.reg & TC_CTRLA_ENABLE)
    {
        return -1;
    }

    NVIC_EnableIRQ(TC2_IRQn);
    commonSetup(delay);

    return 0;
}

int
timerDelay_us(uint32_t delay)
{
    /* Return -1 if timer is already in use */
    if (TC2->COUNT32.CTRLA.reg & TC_CTRLA_ENABLE)
    {
        return -1;
    }

    commonSetup(delay);

    /* Wait for timer to complete, then disable */
    while (0 == (TC2->COUNT32.INTFLAG.reg & TC_INTFLAG_MC0));
    TC2->COUNT32.INTENCLR.reg = TC_INTENCLR_MC0;
    TC2->COUNT32.INTFLAG.reg |= TC_INTFLAG_MC0;
    TC2->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;

    return 0;
}

int
timerDelay_ms(uint16_t delay)
{
    return timerDelay_us(delay * 1000u);
}

int
timerElapsedStart()
{
    /* Return -1 if timer is already in use */
    if (TC2->COUNT32.CTRLA.reg & TC_CTRLA_ENABLE)
    {
        return -1;
    }

    /* Mask match interrupt, zero counter, and start */
    TC2->COUNT32.INTENCLR.reg |= TC_INTENCLR_MC0;
    TC2->COUNT32.COUNT.reg = 0u;
    while (TC2->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
    TC2->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TC2->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
    return 0;
}

uint32_t
timerElapsedStop()
{
    /* Disable timer, and return value of COUNT */
    __disable_irq();
    const uint32_t elapsed = TC2->COUNT32.COUNT.reg;
    __enable_irq();
    TC2->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (TC2->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
    return elapsed;
}


/*! @brief On SysTick overflow, set the event in the main loop
 */
void
irq_handler_sys_tick()
{
    emon32SetEvent(EVT_SYSTICK_1KHz);

    /* Clear the watchdog if in the configuration state, as the normal 1 kHz
     * tick event will not be serviced.
     */
    if (EMON_STATE_CONFIG == emon32StateGet())
    {
        wdtFeed();
    }
}

/*! @brief On delay timer (TC2) expiration, call the callback function
 */
void
irq_handler_tc2()
{
    if (0 != tc2_cb)
    {
        tc2_cb();
    }
}
