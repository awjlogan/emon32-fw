#pragma once

#include <stdint.h>
#include <stdio.h>

#include "board_def.h"
#include "eeprom.h"

#define SERCOM1           0
#define SERCOM4           0
#define EEPROM_SIZE_BYTES 1024

#define printf_ printf

typedef enum { I2CM_ACK = 0u, I2CM_NACK = 1u } I2CM_Ack_t;

typedef enum I2CM_Status_ {
  I2CM_SUCCESS,
  I2CM_ERROR,
  I2CM_TIMEOUT,
  I2CM_NOACK
} I2CM_Status_t;

typedef enum {
  I2CM_ACK_CMD_NONE     = 0u,
  I2CM_ACK_CMD_START    = 1u,
  I2CM_ACK_CMD_CONTINUE = 2u,
  I2CM_ACK_CMD_STOP     = 3u
} I2CM_AckCmd_t;

I2CM_Status_t i2cActivate(int inst, uint8_t addr);
void          i2cDataWrite(int inst, uint8_t data);
uint8_t       i2cDataRead(int inst);
void          i2cAck(int inst, int action, int cmd);
int           timerDelay_us(int a);
int           timerDelay_ms(int a);
void          timerDisable(void);
