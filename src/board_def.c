#include "emon32_samd.h"

/* Each pin are defined in {GROUP, PIN} pairs. Pin numberings are logical,
 * not physical. Each collection of pins is terminated with a {0xFF, 0} pair.
 */

const uint8_t pinsGPIO_Out[][2] = {
    {GRP_PINB,      PIN_LED_STATUS},
    {GRP_PINA,      PIN_LED_PROG},
    {GRP_LED_USER,  PIN_LED_USER0},
    {GRP_LED_USER,  PIN_LED_USER1},
    {0xFF,      0}
};

const uint8_t pinsGPIO_In[][2] = {
    {GRP_TEST_SENSE, PIN_TEST_SENSE},
    {0xFF,      0}
};

const uint8_t pinsUnused[][2] = {
    {GRP_PINA,  0},
    {GRP_PINA,  1},
    {GRP_PINA,  19},
    {GRP_PINA,  20},
    {GRP_PINA,  21},
    {GRP_PINB,  22},
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