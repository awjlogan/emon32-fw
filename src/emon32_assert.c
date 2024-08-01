#include "emon32_assert.h"

void emon32_assert(const uint32_t *pc, const uint32_t *lr) {
  g_assert_info.pc = (uint32_t)pc;
  g_assert_info.lr = (uint32_t)lr;
  __asm("bkpt 0");
}
