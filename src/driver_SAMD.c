#include "emon32_samd.h"

uint32_t
samdCalibration(const Calibration_t cal)
{
    uint32_t mask = 0;
    uint32_t position = 0;
    uint64_t cal_row = *(const volatile uint64_t *)(0x00806020);

    switch (cal)
    {
        case CAL_ADC_LINEARITY:
            mask        = 0xFFu;
            position    = 27u;
            break;
        case CAL_ADC_BIAS:
            mask        = 0x7u;
            position    = 35u;
            break;
        case CAL_OSC32K:
            mask        = 0x7Fu;
            position    = 38u;
            break;
        case CAL_USB_TRANSN:
            mask        = 0x1Fu;
            position    = 45u;
            break;
        case CAL_USB_TRANSP:
            mask        = 0x1Fu;
            position    = 50u;
            break;
        case CAL_USB_TRIM:
            mask        = 0x7u;
            position    = 55u;
            break;
        case CAL_DFLL48M_COARSE:
            mask        = 0x3Fu;
            position    = 58u;
            break;
    }

    return (uint32_t)(cal_row >> position) & mask;
}
