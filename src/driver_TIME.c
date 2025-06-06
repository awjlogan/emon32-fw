#include "emon32_samd.h"

#include "driver_TIME.h"
#include "driver_USB.h"
#include "driver_WDT.h"
#include "emon32.h"

/*! @brief Common setup for 1 us resolution timer
 *  @param [in] delay : delay in us
 */
static void commonSetup(uint32_t delay);
static void timerSync(Tc *tc);

static volatile uint32_t timeMillisCounter  = 0;
static volatile uint32_t timeSecondsCounter = 0;

static bool TIMER_DELAYInUse = false;

static void commonSetup(uint32_t delay) {
  /* Unmask match interrrupt, zero counter, set compare value */
  TIMER_DELAY->COUNT32.INTENSET.reg = TC_INTENSET_MC0;
  TIMER_DELAY->COUNT32.COUNT.reg    = 0u;
  timerSync(TIMER_DELAY);
  TIMER_DELAY->COUNT32.CC[0].reg = delay;
  timerSync(TIMER_DELAY);
  TIMER_DELAY->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
  timerSync(TIMER_DELAY);
}

static void timerSync(Tc *tc) {
  /* All STATUS registers are at the same offset, use COUNT32 for access */
  while (tc->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY)
    ;
}

uint16_t timerADCPeriod(void) {
  return (F_TIMER_ADC / SAMPLE_RATE / VCT_TOTAL);
}

bool timerDelay_ms(uint16_t delay) { return timerDelay_us(delay * 1000u); }

bool timerDelay_us(uint32_t delay) {
  /* Return -1 if timer is already in use */
  if (TIMER_DELAYInUse) {
    return false;
  }

  TIMER_DELAYInUse = true;
  commonSetup(delay);

  /* Wait for timer to complete, then disable */
  while (0 == (TIMER_DELAY->COUNT32.INTFLAG.reg & TC_INTFLAG_MC0))
    ;
  TIMER_DELAY->COUNT32.INTENCLR.reg = TC_INTENCLR_MC0;
  TIMER_DELAY->COUNT32.INTFLAG.reg  = TC_INTFLAG_MC0;
  TIMER_DELAY->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;

  TIMER_DELAYInUse = 0;
  return true;
}

void timerDisable(void) {
  TIMER_DELAY->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  NVIC_DisableIRQ(TIMER_DELAY_IRQn);
}

bool timerElapsedStart(void) {
  /* Return -1 if timer is already in use */
  if (TIMER_DELAYInUse) {
    return false;
  }

  TIMER_DELAYInUse = true;

  /* Mask match interrupt, zero counter, and start */
  TIMER_DELAY->COUNT32.INTENCLR.reg = TC_INTENCLR_MC0;
  TIMER_DELAY->COUNT32.COUNT.reg    = 0u;
  timerSync(TIMER_DELAY);
  TIMER_DELAY->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
  timerSync(TIMER_DELAY);

  TIMER_DELAYInUse = false;
  return true;
}

uint32_t timerElapsedStop(void) {
  /* Disable timer, and return value of COUNT */
  __disable_irq();
  const uint32_t elapsed = TIMER_DELAY->COUNT32.COUNT.reg;
  __enable_irq();
  TIMER_DELAY->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  timerSync(TIMER_DELAY);
  return elapsed;
}

uint32_t timerMicros(void) {
  /* Resynchronise COUNT32.COUNT, and then return the result */
  TIMER_TICK->COUNT32.READREQ.reg = TC_READREQ_RREQ | TC_READREQ_ADDR(0x10);
  timerSync(TIMER_TICK);
  return TIMER_TICK->COUNT32.COUNT.reg;
}

uint32_t timerMicrosDelta(const uint32_t prevMicros) {
  uint32_t delta         = 0;
  uint32_t timeMicrosNow = timerMicros();

  /* Check for wrap (every ~1 h) */
  if (prevMicros > timeMicrosNow) {
    delta = (UINT32_MAX - prevMicros) + timeMicrosNow + 1u;
    ;
  } else {
    delta = timeMicrosNow - prevMicros;
  }

  return delta;
}

uint32_t timerMillis(void) { return timeMillisCounter; }

uint32_t timerMillisDelta(const uint32_t prevMillis) {
  uint32_t delta = 0;

  /* Check for wrap around (every 49 days, so rare!) */
  if (prevMillis > timeMillisCounter) {
    delta = (UINT32_MAX - prevMillis) + timeMillisCounter;
  } else {
    delta = timeMillisCounter - prevMillis;
  }
  return delta;
}

void timerSetup(void) {
  /* TIMER_ADC is used to trigger ADC sampling at constant rate. Enable APB
   * clock, run from generator 3 (OSC8M @ F_PERIPH).
   */
  PM->APBCMASK.reg |= TIMER_ADC_APBCMASK;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TIMER_ADC_GCLK_ID) |
                      GCLK_CLKCTRL_GEN(3u) | GCLK_CLKCTRL_CLKEN;

  /* Configure as 16bit counter (F_PERIPH / 8) -> F_TIMER_ADC.
   * In MFRQ mode, the CC0 register is used as the period.
   */
  TIMER_ADC->COUNT16.CTRLA.reg =
      TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_WAVEGEN_MFRQ |
      TC_CTRLA_RUNSTDBY | TC_CTRLA_PRESCSYNC_RESYNC;

  /* TIMER_ADC MC0 match event to trigger ADC */
  TIMER_ADC->COUNT16.EVCTRL.reg |= TC_EVCTRL_MCEO0;

  /* TIMER_ADC is running at 1 MHz, each tick is 1 us
   * PER, COUNT, and Enable require synchronisation (30.6.6)
   * COUNT is -1 to account for the wrap around
   */
  TIMER_ADC->COUNT16.CC[0].reg = timerADCPeriod() - 1u;
  timerSync(TIMER_ADC);
  TIMER_ADC->COUNT16.COUNT.reg = 0u;
  timerSync(TIMER_ADC);
  TIMER_ADC->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  timerSync(TIMER_ADC);

  /* TIMER_DELAY is used as the delay and elapsed time counter
   * Enable APB clock, set TIMER_DELAY to generator 3 @ F_PERIPH
   * Enable the interrupt for Compare Match, do not route to NVIC
   */
  PM->APBCMASK.reg |= TIMER_DELAY_APBCMASK;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TIMER_DELAY_GCLK_ID) |
                      GCLK_CLKCTRL_GEN(3u) | GCLK_CLKCTRL_CLKEN;
  TIMER_DELAY->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 |
                                   TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_RUNSTDBY |
                                   TC_CTRLA_PRESCSYNC_RESYNC;

  /* TIMER_TICK is used for accurate micro and millisecond time keeping and
   * provides a periodic wakeup to handle non-interrupting events and USB
   * management. Setup at 1 MHz (1 us), and route to NVIC for Compare Match.
   * The compare match is updated by 1000 on each interrupt for 1 ms time.
   */
  PM->APBCMASK.reg |= TIMER_TICK_APBCMASK;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TIMER_TICK_GCLK_ID) |
                      GCLK_CLKCTRL_GEN(3u) | GCLK_CLKCTRL_CLKEN;

  TIMER_TICK->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
  while (TIMER_TICK->COUNT32.CTRLA.reg & TC_CTRLA_SWRST)
    ;

  TIMER_TICK->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 |
                                  TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_RUNSTDBY |
                                  TC_CTRLA_PRESCSYNC_PRESC;

  /* Setup match interrupt for 1 ms  */
  TIMER_TICK->COUNT32.INTENSET.reg = TC_INTENSET_MC0;
  TIMER_TICK->COUNT32.CC[0].reg    = 1000u;
  timerSync(TIMER_TICK);

  NVIC_EnableIRQ(TIMER_TICK_IRQn);
  TIMER_TICK->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
  timerSync(TIMER_TICK);
}

uint32_t timerUptime(void) { return timeSecondsCounter; }

void timerUptimeIncr(void) { timeSecondsCounter++; }

/*! @brief 1 ms timer overflow. Update for the next ms / s match, set the event
 *         and handle any immediate priority actions:
 *          - Regular USB update
 */
void IRQ_TIMER_TICK(void) {
  if (TIMER_TICK->COUNT32.INTFLAG.reg & TC_INTFLAG_MC0) {
    TIMER_TICK->COUNT32.INTFLAG.reg = TC_INTFLAG_MC0;
    TIMER_TICK->COUNT32.CC[0].reg += 1000u;
    timerSync(TIMER_TICK);
    timeMillisCounter++;

    tud_task();
    usbCDCTask();

    emon32EventSet(EVT_TICK_1kHz);
  }
}
