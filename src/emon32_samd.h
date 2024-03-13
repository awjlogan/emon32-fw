#pragma once


/* SAMD uses Arm Cortex-M0+ or Cortex-M4 - can place fast functions into RAM
 * to avoid the penalty of laoding from flash with wait states.
 * REVISIT resolve linker warning when using RAMFUNC
 */
/* #define RAMFUNC __attribute__((section(".ramfunc"))) */
#define RAMFUNC

#include "samd21.h"
