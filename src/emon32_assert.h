#pragma once

/* Reference: https://interrupt.memfault.com/blog/asserts-in-embedded-systems */

#include <stdint.h>

#define GET_LR()    __builtin_return_address(0)
#define GET_PC(_a)  __asm volatile ("mov %0, pc" : "=r" (_a))

typedef struct AssertInfo {
    uint32_t pc;
    uint32_t lr;
} AssertInfo_t;
extern AssertInfo_t g_assert_info;

extern void emon32_assert(const uint32_t *pc, const uint32_t *lr);

#define EMON32_ASSERT_RECORD()  \
  do {                          \
    void *pc;                   \
    GET_PC(pc);                 \
    const void *lr = GET_LR();  \
    emon32_assert(pc, lr);      \
  } while (0)

#define EMON32_ASSERT(exp)      \
  do {                          \
    if (!(exp)) {               \
      EMON32_ASSERT_RECORD();   \
    }                           \
  } while (0)
