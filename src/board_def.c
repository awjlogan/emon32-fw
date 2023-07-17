#include "emon32_samd.h"

/* Each pin are defined in {GROUP, PIN} pairs. Pin numberings are logical,
 * not physical. Each collection of pins is terminated with a {0xFF, 0} pair.
 */
#if (BOARD_ID == BOARD_ID_DEV)

const uint8_t pinsGPIO_Out[][2] = {
    {GRP_PIN,   PIN_LED_STATUS},
    {GRP_PIN,   PIN_EXTINT},
    {GRP_PIN,   PIN_GEN_STATUS},
    {0xFF,      0}
};

const uint8_t pinsGPIO_In[][2] = {
    {GRP_PIN,   PIN_SW},
    {0xFF,      0}
};

const uint8_t pinsUnused[][2] = {
    {GRP_PIN,   22},
    {GRP_PIN,   25},
    {GRP_PIN,   34},
    {0xFF,      0}
};

const uint8_t pinsADC[][2] = {
    {GRP_PIN,   2u},
    {GRP_PIN,   3u},
    {GRP_PIN,   4u},
    {GRP_PIN,   5u},
    {GRP_PIN,   6u},
    {GRP_PIN,   7u},
    {GRP_PIN,   14u},
    {0xFF,      0}
};

#elif (BOARD_ID == BOARD_ID_LC)

const uint8_t pinsGPIO_Out[][2] = {
    {GRP_PIN,   PIN_LED_STATUS},
    {0xFF,      0}
};

const uint8_t pinsGPIO_In[][2] = {
    {0xFF,      0}
};

const uint8_t pinsUnused[][2] = {
    {GRP_PIN,   PIN_LED_PROG},
    {GRP_PIN,   27},
    {0xFF,      0}
};

const uint8_t pinsADC[][2] = {
    {GRP_PIN,   2u},
    {GRP_PIN,   3u},
    {GRP_PIN,   4u},
    {GRP_PIN,   5u},
    {GRP_PIN,   6u},
    {0xFF,      0}
};

#endif