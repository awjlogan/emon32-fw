#include "emon32_samd.h"

#include "emon32.h"
#include "driver_TIME.h"
#include "driver_WDT.h"


void (*tc2_cb)();

void
timerSetup()
{
    /* SysTick is used as the general purpose 100 Hz timer. The tick value is
     * reloaded on underflow SysTick is part of CMSIS so should be portable
     * across Cortex-M cores
     */
    const uint32_t tickkHz = (F_CORE / 100u) - 1u;
    SysTick_Config(tickkHz);

    /* TIMER1 is used to trigger ADC sampling at constant rate */
    /* Enable APB clock, set TC1 to generator 3 (OSC8M @ F_PERIPH)  */
    PM->APBCMASK.reg |= TIMER1_APBCMASK;
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(TIMER1_GCLK_ID)
                        | GCLK_CLKCTRL_GEN(3u)
                        | GCLK_CLKCTRL_CLKEN;

    /* Configure as 8bit counter (F_PERIPH - /8 -> F_TIMER1 */
    TIMER1->COUNT8.CTRLA.reg =   TC_CTRLA_MODE_COUNT8
                               | TC_CTRLA_PRESCALER_DIV8
                               | TC_CTRLA_PRESCSYNC_RESYNC;

    /* TIMER1 overflow event output to trigger ADC */
    TIMER1->COUNT8.EVCTRL.reg |= TC_EVCTRL_OVFEO;

    /* TIMER1 is running at 1 MHz, each tick is 1 us
     * PER, COUNT, and Enable require synchronisation (28.6.6)
     */
    const unsigned int cntPer = F_TIMER1 / SAMPLE_RATE / (VCT_TOTAL);
    TIMER1->COUNT8.PER.reg = (uint8_t)cntPer;
    while (TIMER1->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY);
    TIMER1->COUNT8.COUNT.reg = 0u;
    while (TIMER1->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY);
    TIMER1->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TIMER1->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY);

    /* TIMER2 is used as the delay and elapsed time counter
     * Enable APB clock, set TIMER2 to generator 3 @ F_PERIPH
     * Enable the interrupt for Compare Match, do not route to NVIC
     */
    PM->APBCMASK.reg |= TIMER2_APBCMASK;
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(TIMER2_GCLK_ID)
                        | GCLK_CLKCTRL_GEN(3u)
                        | GCLK_CLKCTRL_CLKEN;
    TIMER2->COUNT32.CTRLA.reg =   TC_CTRLA_MODE_COUNT32
                                | TC_CTRLA_PRESCALER_DIV8
                                | TC_CTRLA_PRESCSYNC_RESYNC;
}

void
commonSetup(uint32_t delay)
{
    /* Unmask match interrrupt, zero counter, set compare value */
    TIMER2->COUNT32.INTENSET.reg |= TC_INTENSET_MC0;
    TIMER2->COUNT32.COUNT.reg = 0u;
    while (TIMER2->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
    TIMER2->COUNT32.CC[0].reg = delay;
    while (TIMER2->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
    TIMER2->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TIMER2->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
}

void
timerDisable()
{
    TIMER2->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    NVIC_DisableIRQ(TIMER2_IRQn);
}

int
timerDelayNB_us(uint32_t delay, void (*cb)())
{
    tc2_cb = cb;
    if (TIMER2->COUNT32.CTRLA.reg & TC_CTRLA_ENABLE)
    {
        return -1;
    }

    NVIC_EnableIRQ(TIMER2_IRQn);
    commonSetup(delay);

    return 0;
}

int
timerDelay_us(uint32_t delay)
{
    /* Return -1 if timer is already in use */
    if (TIMER2->COUNT32.CTRLA.reg & TC_CTRLA_ENABLE)
    {
        return -1;
    }

    commonSetup(delay);

    /* Wait for timer to complete, then disable */
    while (0 == (TIMER2->COUNT32.INTFLAG.reg & TC_INTFLAG_MC0));
    TIMER2->COUNT32.INTENCLR.reg = TC_INTENCLR_MC0;
    TIMER2->COUNT32.INTFLAG.reg |= TC_INTFLAG_MC0;
    TIMER2->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;

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
    if (TIMER2->COUNT32.CTRLA.reg & TC_CTRLA_ENABLE)
    {
        return -1;
    }

    /* Mask match interrupt, zero counter, and start */
    TIMER2->COUNT32.INTENCLR.reg |= TC_INTENCLR_MC0;
    TIMER2->COUNT32.COUNT.reg = 0u;
    while (TIMER2->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
    TIMER2->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TIMER2->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
    return 0;
}

uint32_t
timerElapsedStop()
{
    /* Disable timer, and return value of COUNT */
    __disable_irq();
    const uint32_t elapsed = TIMER2->COUNT32.COUNT.reg;
    __enable_irq();
    TIMER2->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (TIMER2->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
    return elapsed;
}


/*! @brief On SysTick overflow, set the event in the main loop
 */
void
irq_handler_sys_tick()
{
    emon32EventSet(EVT_SYSTICK_100Hz);

    /* Clear the watchdog if in the configuration state, as the normal 1 kHz
     * tick event will not be serviced.
     */
    if (EMON_STATE_CONFIG == emon32StateGet())
    {
        wdtFeed();
    }
}

/*! @brief On delay timer (TIMER2) expiration, call the callback function
 */
void
IRQ_TIMER2()
{
    if (0 != tc2_cb)
    {
        tc2_cb();
    }
}
