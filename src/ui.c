#include "ui.h"
#include "board_def.h"
#include "driver_PORT.h"

void uiLedColour(Led_t led) {
  switch (led) {
  case LED_OFF:
    portPinDrv(GRP_LED_PROG, PIN_LED_PROG, PIN_DRV_SET);
    portPinDrv(GRP_LED_STATUS, PIN_LED_STATUS, PIN_DRV_SET);
    break;
  case LED_GREEN:
    portPinDrv(GRP_LED_PROG, PIN_LED_PROG, PIN_DRV_SET);
    portPinDrv(GRP_LED_STATUS, PIN_LED_STATUS, PIN_DRV_CLR);
    break;
  case LED_RED:
    portPinDrv(GRP_LED_PROG, PIN_LED_PROG, PIN_DRV_CLR);
    portPinDrv(GRP_LED_STATUS, PIN_LED_STATUS, PIN_DRV_SET);
    break;
  case LED_YELLOW:
    portPinDrv(GRP_LED_PROG, PIN_LED_PROG, PIN_DRV_CLR);
    portPinDrv(GRP_LED_STATUS, PIN_LED_STATUS, PIN_DRV_CLR);
    break;
  }
}
