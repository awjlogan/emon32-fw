#pragma once

#include <stdint.h>

/*! @brief Configure the ADC for the board */
void adcSetup();

/*! @brief Do a single conversion on a channel
 *  @param [in] ch : ADC channel to convert
 *  @return : ADC conversion result
 */
int16_t adcSingleConversion(const unsigned int ch);

/*! @brief Starts the DMAC transfer from the ADC
 *  @param [in] buf : address of the "collecting" structure
 */
void adcStartDMAC(uint32_t buf);
