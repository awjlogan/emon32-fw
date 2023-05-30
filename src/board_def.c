#include "emon32_samd.h"

/* Each pin are defined in {GROUP, PIN} pairs. Pin numberings are logical,
 * not physical. Each collection of pins is terminated with a {0xFF, 0} pair.
 */
#if (BOARD_ID == BOARD_ID_DEV)

const uint8_t pinsGPIO_Out[][2] = {
    {0,     PIN_LED},
    {0,     PIN_EXTINT},
    {0,     PIN_GEN_STATUS},
    {0xFF,  0}
};

const uint8_t pinsGPIO_In[][2] = {
    {0,     PIN_SW},
    {0xFF,  0}
};

const uint8_t pinsUnused[][2] = {
    {0,     22},
    {0,     25},
    {0,     34},
    {0xFF,  0}
};

const uint8_t pinsADC[][2] = {
    {0,     2u},
    {0,     3u},
    {0,     4u},
    {0,     5u},
    {0,     6u},
    {0,     7u},
    {0,     14u},
    {0xFF,  0}
};

#elif (BOARD_ID == BOARD_ID_LC)

const uint8_t pinsGPIO_Out[][2] = {
    {0,     PIN_LED},
    {0,     PIN_EXTINT},
    {0,     PIN_GEN_STATUS},
    {0xFF,  0}
};

const uint8_t pinsGPIO_In[][2] = {
    {0,     PIN_SW},
    {0xFF,  0}
};

const uint8_t pinsUnused[][2] = {
    {0,     17}, /* Programming indicator, unused in main */
    {0,     27},
    {0xFF,  0}
};

const uint8_t pinsADC[][2] = {
    {0,     2u},
    {0,     3u},
    {0,     4u},
    {0,     5u},
    {0,     6u},
    {0xFF,  0}
};

#endif