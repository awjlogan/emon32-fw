#include "emon32_samd.h"

#include "emon32.h"
#include "driver_TIME.h"
#include "driver_WDT.h"


/*! @brief Common setup for 1 us resolution timer
 *  @param [in] delay : delay in us
 */
static void commonSetup(uint32_t delay);


static volatile uint32_t timeMillisCounter = 0;
static volatile uint32_t timeSecondsCounter = 0;


/* Function pointer for non-blocking timer callback */
void (*tc2_cb)();
static unsigned int TIMER2InUse = 0;


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


int
timerDelay_ms(uint16_t delay)
{
    return timerDelay_us(delay * 1000u);
}


int
timerDelay_us(uint32_t delay)
{
    /* Return -1 if timer is already in use */
    if (TIMER2InUse)
    {
        return -1;
    }

    TIMER2InUse = 1;
    commonSetup(delay);

    /* Wait for timer to complete, then disable */
    while (0 == (TIMER2->COUNT32.INTFLAG.reg & TC_INTFLAG_MC0));
    TIMER2->COUNT32.INTENCLR.reg = TC_INTENCLR_MC0;
    TIMER2->COUNT32.INTFLAG.reg |= TC_INTFLAG_MC0;
    TIMER2->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;

    TIMER2InUse = 0;
    return 0;
}


void
timerDelayNB_NotInUse()
{
    TIMER2->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    TIMER2InUse = 0;
}


int
timerDelayNB_us(uint32_t delay, void (*cb)())
{

    if (TIMER2InUse)
    {
        return -1;
    }

    tc2_cb = cb;
    NVIC_EnableIRQ(TIMER2_IRQn);
    commonSetup(delay);

    return 0;
}


void
timerDisable()
{
    TIMER2->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    NVIC_DisableIRQ(TIMER2_IRQn);
}


int
timerElapsedStart()
{
    /* Return -1 if timer is already in use */
    if (TIMER2InUse)
    {
        return -1;
    }

    TIMER2InUse = 1;
    /* Mask match interrupt, zero counter, and start */
    TIMER2->COUNT32.INTENCLR.reg |= TC_INTENCLR_MC0;
    TIMER2->COUNT32.COUNT.reg = 0u;
    while (TIMER2->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
    TIMER2->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TIMER2->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);

    TIMER2InUse = 0;
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


uint32_t
timerMicros()
{
    /* Resynchronise COUNT32.COUNT, and then return the result */
    TIMER_TICK->COUNT32.READREQ.reg =   TC_READREQ_RREQ
                                      | TC_READREQ_ADDR(0x10);
    while (TIMER1->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
    return TIMER_TICK->COUNT32.COUNT.reg;
}


uint32_t
timerMicrosDelta(const uint32_t prevMicros)
{
    uint32_t delta = 0;
    uint32_t timeMicrosNow = timerMicros();

    /* Check for wrap (every ~1 h) */
    if (prevMicros > timeMicrosNow)
    {
        delta = (UINT32_MAX - prevMicros) + timeMicrosNow;
    }
    else
    {
        delta = timeMicrosNow - prevMicros;
    }

    return delta;
}


uint32_t
timerMillis()
{
    return timeMillisCounter;
}


uint32_t
timerMillisDelta(const uint32_t prevMillis)
{
    uint32_t delta = 0;

    /* Check for wrap around (every 49 days, so rare!) */
    if (prevMillis > timeMillisCounter)
    {
        delta = (UINT32_MAX - prevMillis) + timeMillisCounter;
    }
    else
    {
        delta = timeMillisCounter - prevMillis;
    }
    return delta;
}


void
timerSetup()
{
    /* TIMER1 is used to trigger ADC sampling at constant rate */
    /* Enable APB clock, set TC1 to generator 3 (OSC8M @ F_PERIPH)  */
    PM->APBCMASK.reg |= TIMER1_APBCMASK;
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(TIMER1_GCLK_ID)
                        | GCLK_CLKCTRL_GEN(3u)
                        | GCLK_CLKCTRL_CLKEN;

    /* Configure as 16bit counter (F_PERIPH / 8) -> F_TIMER1.
     * In MFRQ mode, the CC0 register is used as the period.
     */
    TIMER1->COUNT16.CTRLA.reg =   TC_CTRLA_MODE_COUNT16
                                | TC_CTRLA_PRESCALER_DIV1
                                | TC_CTRLA_WAVEGEN_MFRQ
                                | TC_CTRLA_RUNSTDBY
                                | TC_CTRLA_PRESCSYNC_RESYNC;

    /* TIMER1 MC0 match event to trigger ADC */
    TIMER1->COUNT16.EVCTRL.reg |= TC_EVCTRL_MCEO0;

    /* TIMER1 is running at 1 MHz, each tick is 1 us
     * PER, COUNT, and Enable require synchronisation (28.6.6)
     */
    const unsigned int cntPer = F_TIMER1 / SAMPLE_RATE / (VCT_TOTAL);
    TIMER1->COUNT16.CC[0].reg = (uint16_t)cntPer;
    while (TIMER1->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
    TIMER1->COUNT16.COUNT.reg = 0u;
    while (TIMER1->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
    TIMER1->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TIMER1->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

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
                                | TC_CTRLA_RUNSTDBY
                                | TC_CTRLA_PRESCSYNC_RESYNC;

    /* TIMER_TICK is used for accurate micro and millisecond time keeping and
     * provides a periodic wakeup to handle non-interrupting events and USB
     * management. Setup at 1 MHz (1 us), and route to NVIC for Compare Match.
     * The compare match is updated by 1000 on each interrupt for 1 ms time.
     */
    PM->APBCMASK.reg |= TIMER_TICK_APBCMASK;
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(TIMER_TICK_GCLK_ID)
                        | GCLK_CLKCTRL_GEN(3u)
                        | GCLK_CLKCTRL_CLKEN;

    TIMER_TICK->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
    while(TIMER_TICK->COUNT32.CTRLA.reg & TC_CTRLA_SWRST);

    TIMER_TICK->COUNT32.CTRLA.reg =   TC_CTRLA_MODE_COUNT32
                                    | TC_CTRLA_PRESCALER_DIV8
                                    | TC_CTRLA_RUNSTDBY
                                    | TC_CTRLA_PRESCSYNC_PRESC;

    /* Setup match interrupt for 1 ms  */
    TIMER_TICK->COUNT32.INTENSET.reg |= TC_INTENSET_MC0;
    TIMER_TICK->COUNT32.CC[0].reg = 1000u;
    while (TIMER1->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);

    NVIC_EnableIRQ(TIMER_TICK_IRQn);
    TIMER_TICK->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TIMER1->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
}


uint32_t
timerUptime()
{
    return timeSecondsCounter;
}


void
timerUptimeIncr()
{
    timeSecondsCounter++;
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

/*! @brief 1 ms timer overflow. Update for the next ms / s match, set the event
 *         and handle any immediate priority actions.
 */
void
IRQ_TIMER_TICK()
{
    if (TIMER_TICK->COUNT32.INTFLAG.reg & TC_INTFLAG_MC0)
    {
        TIMER_TICK->COUNT32.INTFLAG.reg |= TC_INTFLAG_MC0;
        TIMER_TICK->COUNT32.CC[0].reg += 1000u;
        while (TIMER_TICK->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);
        timeMillisCounter++;

        emon32EventSet(EVT_TICK_1kHz);
    }
}