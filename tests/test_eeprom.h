#ifndef TEST_EEPROM_H
#define TEST_EEPROM_H

#include <stdint.h>
#include <stdio.h>

#include "eeprom.h"
#include "board_def.h"

#define SERCOM1 0

typedef enum {
    I2CM_ACK    = 0u,
    I2CM_NACK   = 1u
} I2CM_Ack_t;

typedef enum {
    I2CM_ACK_CMD_NONE       = 0u,
    I2CM_ACK_CMD_START      = 1u,
    I2CM_ACK_CMD_CONTINUE   = 2u,
    I2CM_ACK_CMD_STOP       = 3u
} I2CM_AckCmd_t;

void    i2cActivate(int inst, uint8_t addr);
void    i2cDataWrite(int inst, uint8_t data);
uint8_t i2cDataRead(int inst);
void    i2cAck(int inst, int action, int cmd);
int     timerDelayNB_us(int a, void (*p)());
int     timerDelay_us(int a);
void    timerDisable();

#endif
