#pragma once

#include <stdint.h>

/*! @brief Starts the DMAC transfer from the ADC */
void adcDMACStart(void);

/*! @brief Stop the DMAC transfer from the ADC */
void adcDMACStop(void);

/*! @brief Configure the ADC for the board */
void adcSetup(void);

/*! @brief Do a single conversion on a channel
 *  @param [in] ch : ADC channel to convert
 *  @return : ADC conversion result
 */
int16_t adcSingleConversion(const unsigned int ch);
