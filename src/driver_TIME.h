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

/*! @brief  Non-blocking delay.
 *  @param [in] delay : period in us
 *  @param [in] cb : pointer to function for callback
 *  @return : -1 if the timer is already in use, 0 for success
 */
int timerDelayNB_us(uint32_t delay, void (*cb)());

/*! @brief Disable the non-blocking timer.
 */
void timerDisable();

/*! @brief  Start the elapsed time counter at 1 us resolution. Returns -1 if
 *          the timer is already in use.
 */
int timerElapsedStart();

/*! @brief  End the elapsed time counter. Returns the elapsed time in us.
 */
uint32_t timerElapsedStop();

/*! @brief Returns the current microsecond count value
 */
uint32_t timerMicros();

/*! @brief Returns the time delta between microseconds, accounting for wrap
 *  @param [in] prevMicros : previous count value
 *  @return : time delta in microseconds
 */
uint32_t timerMicrosDelta(const uint32_t prevMicros);

/*! @brief Returns the current millisecond count value
 */
uint32_t timerMillis();

/*! @brief Returns the time delta between milliseconds, accounting for wrap
 *  @param [in] prevMicros : previous count value
 *  @return : time delta in milliseconds
 */
uint32_t timerMillisDelta(const uint32_t prevMillis);

/*! @brief  Sets up the system timer units */
void timerSetup();

/*! @brief Returns the uptime in seconds of the whole system
 *  @return : uptime in seconds
 */
uint32_t timerUptime();

/*! @brief Increment the uptime counter */
void timerUptimeIncr();
