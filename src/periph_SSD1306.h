#pragma once

#include "emon32_samd.h"

typedef enum SSD1306_Status_ { SSD1306_SUCCESS, SSD1306_FAIL } SSD1306_Status_t;

typedef struct PosXY_ {
  unsigned int x;
  unsigned int y;
} PosXY_t;

/*! @brief Clear the SSD1306 buffer */
void ssd1306ClearBuffer(void);

/*! @brief Turn the display off */
SSD1306_Status_t ssd1306DisplayOff(void);

/*! @brief Place the buffer contents onto the I2C bus */
SSD1306_Status_t ssd1306DisplayUpdate(void);

/*! @brief Draw a string at the current buffer position */
SSD1306_Status_t ssd1306DrawString(const char *s);

/*! @brief Initialise the SSD1306 OLED display */
SSD1306_Status_t ssd1306Init(Sercom *pSercomI2C);

/*! @brief Set the position in the buffer */
void ssd1306SetPosition(const PosXY_t pos);
