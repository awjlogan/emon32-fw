#include <stdint.h>

#include "board_def.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "emon32_samd.h"
#include "periph_rfm69.h"

#include "RFM69registers.h"

static uint16_t crc16_update(uint16_t crc, const uint8_t d);
static uint8_t  rfmReadReg(const unsigned int addr);
static void     rfmSetAESKey(void);
static void     rfmSleep(void);
static void     rfmWriteReg(const unsigned int addr, const uint8_t data);

const Pin_t     sel = {GRP_SERCOM_SPI, PIN_SPI_RFM_SS};
static int      initDone;
static RFMOpt_t rfmOpt;

static uint8_t rfmReadReg(const unsigned int addr) {
  uint8_t rdByte;
  spiSelect(sel);
  (void)spiSendByte(SERCOM_SPI_DATA, (uint8_t)addr);
  rdByte = spiSendByte(SERCOM_SPI_DATA, 0x00);
  spiDeSelect(sel);
  return rdByte;
}

static void rfmWriteReg(const unsigned int addr, const uint8_t data) {
  spiSelect(sel);
  /* Datasheet 5.2.1, Figure 24: "wnr is 1 for write" */
  (void)spiSendByte(SERCOM_SPI_DATA, ((uint8_t)addr | 0x80));
  (void)spiSendByte(SERCOM_SPI_DATA, data);
  spiDeSelect(sel);
}

/* Adapted from AVR GCC libc:
 * https://www.nongnu.org/avr-libc/user-manual/group__util__crc.html#ga95371c87f25b0a2497d9cba13190847f
 */
static uint16_t crc16_update(uint16_t crc, const uint8_t d) {
  crc ^= d;
  for (unsigned int i = 0; i < 8; i++) {
    crc = (crc & 0x1u) ? (crc >> 1) ^ 0xA001u : (crc >> 1);
  }

  return crc;
}

static void rfmSetAESKey(void) {
  const char aesKey[] = "89txbe4p8aik5kt3";

  /* aesKey includes the NULL terminator, so this will roll in the initial
   * address byte as well.
   */
  spiSelect(sel);
  spiSendByte(SERCOM_SPI_DATA, (REG_AESKEY1 | 0x80));
  spiSendBuffer(SERCOM_SPI_DATA, aesKey, (sizeof(aesKey) - 1));
  spiDeSelect(sel);
}

static void rfmSleep(void) {
  uint8_t tempRecv;

  /* REG_IRQFLAGS2: IRQ2_PACKETSENT */
  while (0 == (rfmReadReg(REG_IRQFLAGS2) & 0x8u)) {
    timerDelay_us(1000);
  }

  tempRecv = rfmReadReg(REG_OPMODE);
  tempRecv = (tempRecv & 0xE3u) | 0x1u;
  rfmWriteReg(REG_OPMODE, tempRecv);
}

RFMOpt_t *rfmGetHandle(void) { return &rfmOpt; }

void rfmInit(RFM_Freq_t freq) {
  /* Configuration parameters */
  const uint8_t config[][2] = {
      {REG_OPMODE, 0x04},     /* OPMODE: Sequencer, standby, listen off */
      {REG_DATAMODUL, 0x00},  /* DataModul: Packet, FSK, no shaping */
      {REG_BITRATEMSB, 0x02}, /* BitRate MSB: ~49.23 Kbps */
      {REG_BITRATELSB, 0x8A}, /* BitRate LSB */
      {REG_FDEVMSB, 0x05},    /* FdevMsb: ~90 kHz */
      {REG_FDEVLSB, 0xC3},    /* FdevLsb */
      {REG_FRFMSB,
       (RFM_FREQ_868MHz == freq)
           ? 0xD9
           : ((RFM_FREQ_915MHz == freq) ? 0xE4 : 0x6C)}, /* FrfMsb */
      {REG_FRFMID, 0x00},                                /* FrfMid */
      {REG_FRFLSB, 0x00},                                /* FrfLsb */
      {REG_PALEVEL, (0x80 | RFM_PALEVEL_DEF)},           /* PaLevel */
      {REG_AFCFEI, 0x2C},
      {REG_DIOMAPPING1, 0x80}, /* DioMapping1 */
      {REG_DIOMAPPING2, 0x03}, /* DioMapping2 */
      {REG_IRQFLAGS2, 0x00},   /* IrqFlags: FIFO overrun */
      {REG_SYNCCONFIG, 0x88},  /* SyncConfig : On, FIFO fill, 2 bytes, Tol */
      {REG_SYNCVALUE1, 0x2D},  /* SyncValue1 */
      {REG_PACKETCONFIG1,
       0x00}, /* PktConfig: fixed, !DC free, !CRC, !CRCClear */
      {0xFF, 0}};

  /* Initialise RFM69 */
  while (0xAA != rfmReadReg(REG_SYNCVALUE1)) {
    rfmWriteReg(REG_SYNCVALUE1, 0xAAu);
  }
  while (0x55u != rfmReadReg(REG_SYNCVALUE1)) {
    rfmWriteReg(REG_SYNCVALUE1, 0x55u);
  }

  /* Configuration */
  for (unsigned int idxCfg = 0; (0xFF != config[idxCfg][0]); idxCfg++) {
    rfmWriteReg(config[idxCfg][0], config[idxCfg][1]);
  }

  rfmSetAESKey();

  initDone = 1;
  rfmSleep();
}

RFMSend_t rfmSend(const void *pData) {
  uint16_t       crc  = ~0;
  const uint8_t *data = (uint8_t *)pData;
  uint8_t        tempRecv;
  unsigned int   txState = 0;
  uint8_t        writeByte;

  /* 1. Check for FIFO full each loop, then push into FIFO:
   *  - Node information (CTL, DST, ACK, ID
   *  - Number of payload bytes
   *  - Data bytes
   *  - CRC16
   * 1.1 If <12 bytes in the FIFO, fill with dummy bytes
   * 2. Send at specified RF power
   * 3. Enter sleep mode
   */
  crc = crc16_update(crc, rfmOpt.grp);
  while (txState < 5) {
    if (0 == (rfmReadReg(REG_IRQFLAGS2) & 0x80u)) {
      switch (txState) {
      case 0:
        writeByte = rfmOpt.node & 0x1F;
        txState++;
        break;
      case 1:
        writeByte = rfmOpt.n;
        txState++;
        break;
      case 2:
        writeByte = *data++;
        rfmOpt.n--;
        if (0 == rfmOpt.n) {
          txState++;
        }
        break;
      case 3:
        writeByte = (uint8_t)crc;
        txState++;
        break;
      case 4:
        writeByte = (uint8_t)(crc >> 8);
        txState++;
        break;
      }
      if (txState < 4) {
        crc = crc16_update(crc, writeByte);
      }
      rfmWriteReg(REG_FIFO, writeByte);
    }
  }

  /* Pad FIFO out to minimum 17 bytes */
  while (txState < 17u) {
    rfmWriteReg(REG_FIFO, 0xAAu);
    txState++;
  }

  writeByte = (rfmOpt.rf_pwr & 0x1F) | 0x80;
  rfmWriteReg(0x11u, writeByte);

  tempRecv = rfmReadReg(REG_OPMODE);
  tempRecv = (tempRecv & 0xE3) | 0xC;
  rfmWriteReg(REG_OPMODE, tempRecv);

  rfmSleep();
  return RFM_SUCCESS;
}

RFMSend_t rfmSendReady(uint32_t timeout) {
  unsigned int t_start_ms = timerMillis();
  uint8_t      tempRecv;

  if (0 == initDone) {
    return RFM_NO_INIT;
  }

  /* Wait for "clear" air to transmit in.
   * 1. Enter receive mode
   * 2. Listen until below threshold
   * 3. If over time, then return with failure. Otherwise proceed
   */
  while (1) {
    if (timerMillisDelta(t_start_ms) > timeout) {
      return RFM_TIMED_OUT;
    }

    tempRecv = rfmReadReg(REG_OPMODE);
    tempRecv = (tempRecv & 0xE3) | 0x10;

    /* Wait for READY */
    while (0 == (rfmReadReg(0x27) & 0x80))
      ;

    /* REG_RSSI_CONFIG: RSSI_START */
    rfmWriteReg(0x23u, 0x1u);
    /* RSSI_DONE */
    while (0 == (rfmReadReg(0x23u) & 0x02u))
      ;

    /* REG_RSSI_VALUE */
    if (rfmReadReg(0x24u) > (rfmOpt.threshold * -2)) {
      return RFM_SUCCESS;
    }
    /* Restart receiver */
    /* REG_PACKET_CONFIG2 */
    tempRecv = rfmReadReg(0x3Du);
    tempRecv = (tempRecv & 0xFB) | 0x4u;
    rfmWriteReg(0x3Du, tempRecv);
  }

  return RFM_FAILED;
}
