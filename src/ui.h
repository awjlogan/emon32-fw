#pragma once

typedef enum Led_ { LED_PROG, LED_STATUS, LED_USER0, LED_USER1 } Led_t;

void uiLedOn(Led_t led);

void uiLedOff(Led_t led);

void uiLedToggle(Led_t led);
