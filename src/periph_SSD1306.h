#pragma once

#include "emon32_samd.h"


typedef enum {
    SUCCESS_INIT,
    SUCCESS_DISPLAY,
    SUCCESS_UPDATE_POS,
    SUCCESS_DRAW_CHAR,
    SUCCESS_DRAW_STRING,
    FAIL_UPDATE_POS,
    FAIL_DRAW_STRING
} SSD1306_Status_t;

typedef struct _posXY {
    unsigned int x;
    unsigned int y;
} PosXY_t;


/*! @brief Clear the SSD1306 buffer */
void ssd1306ClearBuffer();

/*! @brief Turn the display off */
SSD1306_Status_t ssd1306DisplayOff();

/*! @brief Place the buffer contents onto the I2C bus */
SSD1306_Status_t ssd1306DisplayUpdate();

/*! @brief Draw a string at the current buffer position */
SSD1306_Status_t ssd1306DrawString(const char *s);

/*! @brief Initialise the SSD1306 OLED display */
SSD1306_Status_t ssd1306Init(Sercom *pSercomI2C);

/*! @brief Set the position in the buffer */
void ssd1306SetPosition(const PosXY_t pos);
