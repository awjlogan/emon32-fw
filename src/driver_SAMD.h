#ifndef DRIVER_SAMD_H
#define DRIVER_SAMD_H

typedef enum {
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
 *  @return : calibration value
 */
uint32_t samdCalibration(const Calibration_t cal);

#endif
