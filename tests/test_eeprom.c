#include <assert.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "eeprom.h"
#include "emon32.h"
#include "test_eeprom.h"

typedef enum {
  I2C_IDLE,
  I2C_ACTIVATE_ADDR,
  I2C_ACTIVATE_WR,
  I2C_ACTIVATE_WR_ADDR,
  I2C_ACTIVATE_RD,
  I2C_BYTE_WR,
  I2C_BYTE_RD
} i2c_state_t;

i2c_state_t state = I2C_IDLE;

uint8_t      eeprom[1024];
unsigned int currentAddrHigh;
unsigned int currentAddrLow;

uint16_t calcCRC16_ccitt(const void *pSrc, unsigned int n) {
  (void)pSrc;
  (void)n;
  return 0;
}

uint16_t currentAddr(void) {
  uint32_t r = (((currentAddrHigh >> 1) & 0x3) << 8) | currentAddrLow;
  return (uint16_t)r;
}

void EMON32_ASSERT(bool val) { assert(val); }

I2CM_Status_t i2cActivate(int inst, uint8_t addr) {
  unsigned int isRead;

  (void)inst;
  isRead = addr & 0x1 ? 1 : 0;
  state  = isRead ? I2C_ACTIVATE_RD : I2C_ACTIVATE_ADDR;

  currentAddrHigh = addr;
  return I2CM_SUCCESS;
}

void i2cDataWrite(int inst, uint8_t data) {
  uint16_t lastAddr = currentAddr();

  if (I2C_ACTIVATE_ADDR == state) {
    currentAddrLow = data;
    state          = I2C_IDLE;
    return;
  }

  (void)inst;

  /* Can't over run a 16 byte boundary */
  if ((currentAddr() & 0xFFF0) ^ (lastAddr & 0xFFF0)) {
    printf("\nOver ran 16 byte boundary: %d -> %d\n\n", lastAddr,
           currentAddr());
    assert(0);
  }

  eeprom[(((currentAddrHigh >> 1) & 0x3) << 8) | currentAddrLow++] = data;
}

uint8_t i2cDataRead(int inst) {
  (void)inst;
  uint8_t  data;
  uint16_t idx = currentAddr();
  data         = eeprom[idx];
  // printf("%#x\tR\t%#x\n", currentAddr(), data);
  currentAddrLow++;
  return data;
}

void i2cAck(int inst, int action, int cmd) {}

int timerDelay_us(int a) { return 0; }

int timerDelay_ms(int a) { return 0; }

void timerDisable(void) {}

void dumpMem(unsigned int start) {
  const unsigned int lineLen = 16;
  char               c;

  printf("\n");
  for (unsigned int i = start / lineLen; i < EEPROM_SIZE_BYTES / lineLen; i++) {
    /* Hex dump */
    printf("0x%02x0\t", i);
    for (unsigned int j = 0; j < lineLen; j++) {

      printf("%02x ", eeprom[i * lineLen + j]);
    }

    /* Readable ASCII characters */
    printf("\t");
    for (unsigned int j = 0; j < lineLen; j++) {
      c = eeprom[i * lineLen + j];
      if (!isprint(c))
        c = '.';
      printf("%c", c);
    }
    printf("\n");
  }
}

void initMem(unsigned int start, unsigned int end, uint8_t val) {
  for (unsigned int i = start; i < end; i++) {
    eeprom[i] = val;
  }
}

/* Check that the lower (non-wear level protected) area is not
 * overwritten by the wear levelling routing */
void checkStatic(void) {
  static int call = 0;
  for (unsigned int i = 0; i < EEPROM_WL_OFFSET; i++) {
    if (eeprom[i] != 0xFF) {
      dumpMem(0);
      printf("\nWear levelling over wrote static area (call %d).\n", call);
      assert(0);
    }
  }
  call++;
}

uint32_t timerMicros(void) { return 0; }
uint32_t timerMicrosDelta(uint32_t prevMicros) { return EEPROM_WR_TIME + 1; }

int main(int argc, char *argv[]) {
  char str[] = "This is a very long string that will wrap over writes";
  Emon32Cumulative_t cumulative;

  /* Fresh EEPROM is all 1s */
  initMem(0, EEPROM_SIZE_BYTES, 0xFFu);

  printf("---- emon32 EEPROM test ----\n\n");

  printf("  > Write over multiple 16 byte pages ...");
  (void)eepromWrite(0, str, strlen(str) + 1u);
  while (EEPROM_WR_COMPLETE != eepromWrite(0, 0, 0))
    ;

  (void)eepromWrite(63, str, strlen(str) + 1u);
  while (EEPROM_WR_COMPLETE != eepromWrite(0, 0, 0))
    ;

  printf(" Done!\n");

  printf("  > Wear level routine tests ... ");

  initMem(0, (EEPROM_WL_OFFSET), 0xFFu);
  /* The wear levelled portion should be set to 0 initially, so that false
   * values are read at the very first time powered on */
  initMem(EEPROM_WL_OFFSET, (EEPROM_SIZE_BYTES - EEPROM_WL_OFFSET), 0x0u);
  eepromWLReset(sizeof(cumulative));

  for (int i = 0; i < NUM_CT; i++) {
    cumulative.wattHour[i] = i * i;
  }
  for (int i = 0; i < NUM_OPA; i++) {
    cumulative.pulseCnt[i] = i * i;
  }

  int idx = 0;
  /* Check for 0x00 -> 0xFF fill */
  for (int i = 0; i < (12 * 8); i++) {
    eepromWriteWL(&cumulative, &idx);

    /* Check no over run into non-wear levelled portion
     * and correct valid bytes written */
    checkStatic();
    if (eeprom[EEPROM_WL_OFFSET +
               ((i * 64) % (EEPROM_SIZE_BYTES - EEPROM_WL_OFFSET))] !=
        ((1 << (i / 12)) - 1)) {
      dumpMem(EEPROM_WL_OFFSET);
      printf("  > Incorrect valid indicator 0x%x at address 0x%x, expected "
             "0x%x in "
             "loop %d\n",
             eeprom[EEPROM_WL_OFFSET +
                    ((i * 64) % (EEPROM_SIZE_BYTES - EEPROM_WL_OFFSET))],
             (EEPROM_WL_OFFSET +
              ((i * 64) % (EEPROM_SIZE_BYTES - EEPROM_WL_OFFSET))),
             ((1 << (i / 12)) - 1), i);
      assert(0);
    }
    for (int i = 0; i < NUM_CT; i++) {
      cumulative.wattHour[i] += i;
    }
    for (int i = 0; i < NUM_OPA; i++) {
      cumulative.pulseCnt[i] += i;
    }
  }

  if (eeprom[EEPROM_WL_OFFSET] != 0xFF) {
    dumpMem(EEPROM_WL_OFFSET);
    printf("  > Incorrect valid indicator %d, expected 0xFF\n",
           eeprom[EEPROM_WL_OFFSET]);
    assert(0);
  }

  /* Check for 0xFF -> 0x00 fill */
  for (int i = 0; i < (12 * 8); i++) {
    eepromWriteWL(&cumulative, &idx);

    checkStatic();
    if (eeprom[EEPROM_WL_OFFSET +
               ((i * 64) % (EEPROM_SIZE_BYTES - EEPROM_WL_OFFSET))] !=
        (UINT8_MAX & ~((1 << (i / 12)) - 1))) {
      dumpMem(EEPROM_WL_OFFSET);
      printf("  > Incorrect valid indicator 0x%x at address 0x%x, expected "
             "0x%x in "
             "loop %d\n",
             eeprom[EEPROM_WL_OFFSET +
                    ((i * 64) % (EEPROM_SIZE_BYTES - EEPROM_WL_OFFSET))],
             (EEPROM_WL_OFFSET +
              ((i * 64) % (EEPROM_SIZE_BYTES - EEPROM_WL_OFFSET))),
             (UINT8_MAX & ~((1 << (i / 12)) - 1)), i);
      assert(0);
    }

    for (int i = 0; i < NUM_CT; i++) {
      cumulative.wattHour[i] += i;
    }
    for (int i = 0; i < NUM_OPA; i++) {
      cumulative.pulseCnt[i] += i;
    }
  }

  /* Check identifying the last write is successful
   * initialise EEPROM and seed values into the "valid" positions, then
   * find the latest. */
  const unsigned int lastValidPos = 5;
  initMem(EEPROM_WL_OFFSET, (EEPROM_SIZE_BYTES - EEPROM_WL_OFFSET), 0);
  eepromWLReset(sizeof(cumulative));
  for (unsigned int i = EEPROM_WL_OFFSET; i < EEPROM_SIZE_BYTES; i = i + 64) {
    eeprom[i] = 1;
  }
  for (int i = (EEPROM_WL_OFFSET + (lastValidPos * 64)); i < EEPROM_SIZE_BYTES;
       i     = i + 64) {
    eeprom[i] = 0;
  }
  eepromWriteWL(&cumulative, &idx);
  if (idx != lastValidPos) {
    printf("\r\n    > Failed. Expected write position to be %d, actual %d\r\n",
           lastValidPos, idx);
    assert(0);
  }

  printf("Done!\n");
}
