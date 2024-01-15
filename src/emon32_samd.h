#pragma once

#include "board_def.h"

/* SAMD uses Arm Cortex-M0+ or Cortex-M4 - can place fast functions into RAM
 * to avoid the penalty of laoding from flash with wait states
 */
#define RAMFUNC __attribute__((section(".ramfunc")))

#include "samd21.h"
