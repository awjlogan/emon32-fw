#include "fuses.h"
#include "board_def.h"
#include "emon32_samd.h"

static void writeFuses(NVM_USER_ROW_Type *fuses);

void checkBootProtection(void) {
  if (BOOTPROT_SAMD != NVM_USER_ROW->FUSES.bit.BOOTPROT) {
    NVM_USER_ROW_Type fuses  = *NVM_USER_ROW;
    fuses.FUSES.bit.BOOTPROT = BOOTPROT_SAMD;

    writeFuses(&fuses);
    NVIC_SystemReset();
  }
}

static void writeFuses(NVM_USER_ROW_Type *fuses) {
  __disable_irq();

  /* Prepare to write to the NVMCTRL */
  uint32_t ctrlb = NVMCTRL->CTRLB.reg;
  NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;
  NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_CACHEDIS | NVMCTRL_CTRLB_MANW;
  NVMCTRL->ADDR.reg = NVMCTRL_FUSES_BOOTPROT_ADDR / 2;

  /* Erase the row and flush the page cache */
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_EAR;
  while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY))
    ;
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
  while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY))
    ;

  /* Write data out in 32 bit packets */
  ((uint32_t *)NVMCTRL_USER)[0] = ((uint32_t *)fuses)[0];
  ((uint32_t *)NVMCTRL_USER)[1] = ((uint32_t *)fuses)[1];

  /* Write the page and restore the saved CTRLB value. */
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WAP;
  while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY))
    ;
  NVMCTRL->CTRLB.reg = ctrlb;

  __enable_irq();
}
