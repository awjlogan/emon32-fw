#pragma once

#include <stdbool.h>
#include <stdint.h>

/*! @brief Get the ADC trigger period
 *  @return trigger period in microsceconds
 */
uint16_t timerADCPeriod(void);

/*! @brief  Blocking delay. Use with caution.
 *  @param [in] delay : period in ms
 *  @return true if successful, false otherwise
 */
bool timerDelay_ms(uint16_t delay);

/*! @brief  Blocking delay. Use with caution.
 *  @param [in] delay : period in us
 *  @return true if successful, false otherwise
 */
bool timerDelay_us(uint32_t delay);

/*! @brief  Start the elapsed time counter at 1 us resolution.
 *  @return true if successful, false otherwise
 */
bool timerElapsedStart(void);

/*! @brief  End the elapsed time counter. Returns the elapsed time in us.
 */
uint32_t timerElapsedStop(void);

/*! @brief Returns the current microsecond count value
 */
uint32_t timerMicros(void);

/*! @brief Returns the time delta between microseconds, accounting for wrap
 *  @param [in] prevMicros : previous count value
 *  @return time delta in microseconds
 */
uint32_t timerMicrosDelta(const uint32_t prevMicros);

/*! @brief Returns the current millisecond count value
 */
uint32_t timerMillis(void);

/*! @brief Returns the time delta between milliseconds, accounting for wrap
 *  @param [in] prevMicros : previous count value
 *  @return time delta in milliseconds
 */
uint32_t timerMillisDelta(const uint32_t prevMillis);

/*! @brief  Sets up the system timer units */
void timerSetup(void);

/*! @brief Returns the uptime in seconds of the whole system
 *  @return uptime in seconds
 */
uint32_t timerUptime(void);

/*! @brief Increment the uptime counter */
void timerUptimeIncr(void);
