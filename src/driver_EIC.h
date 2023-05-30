#ifndef DRIVER_EIC_H
#define DRIVER_EIC_H

/*! @brief Configure External Interrupt Controller */
void eicSetup();

/*! @brief Enable EIC pin. All pins are rising edge
 *         sensitive, with no filter.
 *         TODO Pin mapping is not contiguous, so use a LUT. For the current
 *              implementation, a single fixed pin is used but this could be
 *              generalised in future.
 */
void eicEnablePin();

/*! @brief Disable EIC pin */
void eicDisablePin();

#endif