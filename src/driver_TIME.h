#pragma once

#include <stdint.h>


/*! @brief  Blocking delay. Use with caution. Returns -1 if the timer is
 *          already in use.
 *  @param [in] delay : period in ms
 */
int timerDelay_ms(uint16_t delay);

/*! @brief  Blocking delay. Use with caution. Returns -1 if the timer is
 *          already in use.
 *  @param [in] delay : period in us
 */
int timerDelay_us(uint32_t delay);

/*! @brief Returns the non-block delay mutex
 */
void timerDelayNB_NotInUse(void);

/*! @brief Setup non-blocking delay.
 *  @param [in] delay : period in us
 *  @param [in] cb : pointer to function for callback
 *  @return : -1 if the timer is already in use, 0 for success
 */
int timerDelayNB_us(uint32_t delay, void (*cb)(void));

/*! @brief  Start the elapsed time counter at 1 us resolution. Returns -1 if
 *          the timer is already in use.
 */
int timerElapsedStart(void);

/*! @brief  End the elapsed time counter. Returns the elapsed time in us.
 */
uint32_t timerElapsedStop(void);

/*! @brief Returns the current microsecond count value
 */
uint32_t timerMicros(void);

/*! @brief Returns the time delta between microseconds, accounting for wrap
 *  @param [in] prevMicros : previous count value
 *  @return : time delta in microseconds
 */
uint32_t timerMicrosDelta(const uint32_t prevMicros);

/*! @brief Returns the current millisecond count value
 */
uint32_t timerMillis(void);

/*! @brief Returns the time delta between milliseconds, accounting for wrap
 *  @param [in] prevMicros : previous count value
 *  @return : time delta in milliseconds
 */
uint32_t timerMillisDelta(const uint32_t prevMillis);

/*! @brief  Sets up the system timer units */
void timerSetup(void);

/*! @brief Returns the uptime in seconds of the whole system
 *  @return : uptime in seconds
 */
uint32_t timerUptime(void);

/*! @brief Increment the uptime counter */
void timerUptimeIncr(void);
