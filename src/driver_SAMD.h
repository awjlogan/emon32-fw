#pragma once

#include <stdint.h>

typedef enum Calibration_ {
  CAL_ADC_LINEARITY,
  CAL_ADC_BIAS,
  CAL_OSC32K,
  CAL_USB_TRANSN,
  CAL_USB_TRANSP,
  CAL_USB_TRIM,
  CAL_DFLL48M_COARSE
} Calibration_t;

/*! @brief Return the calibration value from the NVM Calibration Row, described
 *         in Table 9-4
 *  @param [in] cal : enumeration of the calibration value required
 *  @return selected calibration value
 */
uint32_t samdCalibration(const Calibration_t cal);
