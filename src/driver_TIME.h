#ifndef DRIVER_TIME_H
#define DRIVER_TIME_H

#include <stdint.h>

/*! @brief  Sets up the systm timers unit */
void timerSetup();

/*! @brief  Non-blocking delay.
 *  @param [in] delay : period in us
 *  @param [in] cb : pointer to function for callback
 *  @return : -1 if the timer is already in use.
 */
int timerDelayNB_us(uint32_t delay, void (*cb)());

/*! @brief Disable the non-blocking timer.
 */
void timerDisable();

/*! @brief  Enable timer interrupt to NVIC (or similar)
 */
void timerInterruptEnable();

/*! @brief  Disable timer interrupt to NVIC (or similar)
 */
void timerInterruptDisable();

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

/*! @brief  Start the elapsed time counter at 1 us resolution. Returns -1 if
 *          the timer is already in use.
 */
int timerElapsedStart();

/*! @brief  End the elapsed time counter. Returns the elapsed time in us.
 */
uint32_t timerElapsedStop();

#endif
