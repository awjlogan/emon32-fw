#include <stdbool.h>

#include "emon32_samd.h"

#include "board_def.h"
#include "driver_EIC.h"
#include "driver_PORT.h"
#include "periph_rfm69.h"

void eicConfigureRfmIrq(void) {
  // REVISIT : abstract from underlying hardware
  EIC->CONFIG[1].reg = EIC_CONFIG_FILTEN7 | EIC_CONFIG_SENSE7_HIGH;
  EIC->INTENSET.reg  = EIC_INTENSET_EXTINT14;
  EIC->CTRL.reg      = EIC_CTRL_ENABLE;
  while (EIC->STATUS.reg & EIC_STATUS_SYNCBUSY)
    ;
  NVIC_EnableIRQ(EIC_IRQn);
}

void eicSetup(void) {
  /* EIC APB clock is unmasked on reset (16.8.8)
   * Only required GCLK for edge detection */
}

void irq_handler_eic(void) {
  if (EIC->INTFLAG.reg & EIC_INTFLAG_EXTINT14) {
    rfmInterrupt();
    EIC->INTFLAG.reg |= EIC_INTFLAG_EXTINT14;
  }
}
