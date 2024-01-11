#pragma once

#include <stdint.h>

typedef enum {
    WDT_PER_8   = 0x0u,
    WDT_PER_16  = 0x1u,
    WDT_PER_32  = 0x2u,
    WDT_PER_64  = 0x3u,
    WDT_PER_128 = 0x4u,
    WDT_PER_256 = 0x5u,
    WDT_PER_512 = 0x6u,
    WDT_PER_1K  = 0x7u,
    WDT_PER_2K  = 0x8u,
    WDT_PER_4K  = 0x9u,
    WDT_PER_8K  = 0xAu,
    WDT_PER_16K = 0xBu
} WDT_PER_t;

/*! @brief Configure the watchdog timer in normal mode
 *  @param [in] per : clock period (weak enum from header)
 */
void wdtSetup(WDT_PER_t per);

/*! @brief Feed the watchdog to reset
 */
void wdtFeed();

