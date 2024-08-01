#include "ui.h"
#include "board_def.h"
#include "driver_PORT.h"

static Pin_t ledToPin(Led_t led);

static Pin_t ledToPin(Led_t led) {
  Pin_t p;
  switch (led) {
  case LED_PROG:
    p.grp = GRP_LED_PROG;
    p.pin = PIN_LED_PROG;
    break;
  case LED_STATUS:
    p.grp = GRP_LED_STATUS;
    p.pin = PIN_LED_STATUS;
    break;
  case LED_USER0:
    p.grp = GRP_LED_USER;
    p.pin = PIN_LED_USER0;
    break;
  case LED_USER1:
    p.grp = GRP_LED_USER;
    p.pin = PIN_LED_USER1;
    break;
  default:
    break;
  }
  return p;
}

void uiLedOn(Led_t led) {
  Pin_t p = ledToPin(led);
  portPinDrv(p.grp, p.pin, PIN_DRV_CLR);
}

void uiLedOff(Led_t led) {
  Pin_t p = ledToPin(led);
  portPinDrv(p.grp, p.pin, PIN_DRV_SET);
}

void uiLedToggle(Led_t led) {
  Pin_t p = ledToPin(led);
  portPinDrv(p.grp, p.pin, PIN_DRV_TGL);
}
