#pragma once

typedef enum Led_ { LED_OFF, LED_RED, LED_YELLOW, LED_GREEN } Led_t;

void uiLedColour(Led_t led);
