#include "board_def.h"
#include "emon32_samd.h"

#include "configuration.h"
#include "driver_PORT.h"
#include "driver_USB.h"

bool usbCDCIsConnected(void) { return tud_cdc_connected(); }

void usbCDCPutsBlocking(const char *s) {
  if (usbCDCTxFull()) {
    tud_cdc_write_flush();
  }
  tud_cdc_write_str(s);
}

bool usbCDCRxAvailable(void) { return tud_cdc_available(); }

uint8_t usbCDCRxGetChar(void) {
  if (tud_cdc_available()) {
    return tud_cdc_read_char();
  }

  return 0;
}

void usbCDCTask(void) {
  /* Flush write buffer and read any available characters */
  tud_cdc_write_flush();
  int nrx = tud_cdc_available();
  if (nrx) {
    for (int i = 0; i < nrx; i++) {
      configCmdChar(tud_cdc_read_char());
    }
  }
}

void usbCDCTxChar(uint8_t c) {
  if (usbCDCTxFull()) {
    tud_cdc_write_flush();
  }
  tud_cdc_write_char(c);
}

void usbCDCTxFlush(void) { tud_cdc_write_flush(); }

bool usbCDCTxFull(void) { return !tud_cdc_write_available(); }

void usbSetup(void) {
  /* Clocking:
   *  - AHB is enabled by default (16.8.7)
   *  - APB is enabled by default (16.8.9)
   *  - Use the 48 MHz PLL for required accuracy
   */
  PM->APBBMASK.reg |= PM_APBBMASK_USB;
  PM->AHBMASK.reg |= PM_AHBMASK_USB;

  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_ID_USB | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;

  /* Configure ports (Table 7-1) */
  portPinDir(GRP_USB_DM, PIN_USB_DM, PIN_DIR_OUT);
  portPinDir(GRP_USB_DP, PIN_USB_DP, PIN_DIR_OUT);
  portPinMux(GRP_USB_DM, PIN_USB_DM, PMUX_USB);
  portPinMux(GRP_USB_DP, PIN_USB_DP, PMUX_USB);

  NVIC_SetPriority(USB_IRQn, 1);
  NVIC_EnableIRQ(USB_IRQn);

  tud_init(0);
}

void irq_handler_usb(void) { tud_int_handler(0); }
