#pragma once

#include <stdbool.h>
#include <stdint.h>

/* @brief Get the gain correction value
 * @return : the gain correction value
 */
int16_t adcCorrectionGain(void);

/* @brief Get the offset correction value
 * @return : the offset correction value
 */
int16_t adcCorrectionOffset(void);

/*! @brief Indicates if the correction values are valid
 *  @return true if valid, false otherwise
 */
bool adcCorrectionValid(void);

/*! @brief Starts the DMAC transfer from the ADC */
void adcDMACStart(void);

/*! @brief Stop the DMAC transfer from the ADC */
void adcDMACStop(void);

/*! @brief Configure the ADC for the board */
void adcSetup(void);
