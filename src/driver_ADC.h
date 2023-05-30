#ifndef DRIVER_ADC_H
#define DRIVER_ADC_H

#include <stdint.h>

/*! @brief Configure the ADC for the board
 *  TODO Make this a configurable function
 */
void adcSetup();

/*! @brief Starts the DMAC transfer from the ADC
 *  @param [in] buf : address of the "collecting" structure
 */
void adcStartDMAC(uint32_t buf);

#endif
