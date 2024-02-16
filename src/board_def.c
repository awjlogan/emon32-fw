#include <stdint.h>

#include "emon32_samd.h"

/* Each pin are defined in {GROUP, PIN} pairs. Pin numberings are logical,
 * not physical. Each collection of pins is terminated with a {0xFF, 0} pair.
 */

const uint8_t pinsGPIO_Out[][2] = {
    {GRP_PINB,      PIN_LED_STATUS},
    {GRP_PINA,      PIN_LED_PROG},
    {0xFF,      0}
};

const uint8_t pinsGPIO_In[][2] = {
    {GRP_PULSE,         PIN_PULSE1},
    {GRP_PULSE,         PIN_PULSE2},
    {GRP_ONEWIRE,       PIN_ONEWIRE},
    {GRP_PINA,          PIN_REV0},
    {GRP_PINA,          PIN_REV1},
    {GRP_PINA,          PIN_REV2},
    {0xFF,              0}
};

const uint8_t pinsUnused[][2] = {
    {GRP_PINA,  0},
    {GRP_PINA,  1},
    {GRP_PINA,  19},
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
