#include "emon32_samd.h"

/* Each pin are defined in {GROUP, PIN} pairs. Pin numberings are logical,
 * not physical. Each collection of pins is terminated with a {0xFF, 0} pair.
 */
#if (BOARD_ID == BOARD_ID_EMONPI)

const uint8_t pinsGPIO_Out[][2] = {
    {GRP_PINB,  PIN_LED_STATUS},
    {GRP_PINA,  PIN_LED_PROG},
    {0xFF,      0}
};

const uint8_t pinsGPIO_In[][2] = {
    {0xFF,      0}
};

const uint8_t pinsUnused[][2] = {
    {GRP_PINA,  22},
    {GRP_PINA,  25},
    {GRP_PINA,  34},
    {0xFF,      0}
};

const uint8_t pinsPulse[][2] = {
    {GRP_PINA,  17},
    {GRP_PINA,  16},
    {0xFF,      0}
};

const uint8_t pinsADC[][2] = {
    {GRP_PINA,  2u},
    {GRP_PINA,  3u},
    {GRP_PINA,  4u},
    {GRP_PINA,  5u},
    {GRP_PINA,  6u},
    {GRP_PINA,  7u},
    {GRP_PINA,  8u},
    {GRP_PINA,  9u},
    {GRP_PINA,  10u},
    {GRP_PINB,  0u},
    {GRP_PINB,  1u},
    {GRP_PINB,  2u},
    {GRP_PINB,  3u},
    {GRP_PINB,  4u},
    {GRP_PINB,  5u},
    {GRP_PINB,  6u},
    {GRP_PINB,  7u},
    {GRP_PINB,  8u},
    {GRP_PINB,  9u},
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