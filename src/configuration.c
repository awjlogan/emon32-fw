#include <inttypes.h>
#include <string.h>

#include "emon32_assert.h"

#include "driver_ADC.h"
#include "driver_PORT.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "driver_USB.h"

#include "configuration.h"
#include "eeprom.h"
#include "emon32.h"
#include "emon32_build_info.h"
#include "emon_CM.h"
#include "util.h"

#include "printf.h"
#include "qfplib-m0-full.h"

/*************************************
 * Types
 *************************************/

typedef enum {
  RCAUSE_SYST  = 0x40,
  RCAUSE_WDT   = 0x20,
  RCAUSE_EXT   = 0x10,
  RCAUSE_BOD33 = 0x04,
  RCAUSE_BOD12 = 0x02,
  RCAUSE_POR   = 0x01
} RCAUSE_t;

/*************************************
 * Prototypes
 *************************************/

static void     configDefault(void);
static void     configInitialiseNVM(void);
static int      configTimeToCycles(const float time, const int mainsFreq);
static bool     configureAnalog(void);
static bool     configureAssumed(void);
static bool     configureDatalog(void);
static void     configureOPA(void);
static bool     configureSerialLog(void);
static bool     configureWhDelta(void);
static void     enterBootloader(void);
static uint32_t getBoardRevision(void);
static char    *getLastReset(void);
static void     inBufferClear(int n);
static void     printSettings(void);
static void     printUptime(void);
static void     putFloat(float val, int flt_len);
static char     waitForChar(void);
static bool     zeroAccumulators(void);

/*************************************
 * Local variables
 *************************************/

#define IN_BUFFER_W 64
static Emon32Config_t config;
static char           inBuffer[IN_BUFFER_W];
static int            inBufferIdx   = 0;
static bool           cmdPending    = false;
static bool           resetReq      = false;
static bool           unsavedChange = false;

/*! @brief Set all configuration values to defaults */
static void configDefault(void) {
  config.key = CONFIG_NVM_KEY;

  /* Single phase, 50 Hz, 240 VAC, 10 s report period */
  config.baseCfg.nodeID       = NODE_ID; /* Node ID to transmit */
  config.baseCfg.mainsFreq    = 50u;     /* Mains frequency */
  config.baseCfg.reportTime   = 9.8f;
  config.baseCfg.whDeltaStore = DELTA_WH_STORE; /* 200 */
  config.baseCfg.dataGrp      = 210u;
  config.baseCfg.logToSerial  = true;
  config.baseCfg.useJson      = false;
  config.dataTxCfg.txType     = (uint8_t)DATATX_RFM69;
  config.dataTxCfg.rfmPwr     = RFM_PALEVEL_DEF;
  config.dataTxCfg.rfmFreq    = 2; /* 433 MHz */

  for (int idxV = 0u; idxV < NUM_V; idxV++) {
    config.voltageCfg[idxV].voltageCal = 100.0f;
    config.voltageCfg[idxV].vActive    = (0 == idxV);
  }

  /* 4.2 degree shift @ 50 Hz */
  for (int idxCT = 0u; idxCT < NUM_CT; idxCT++) {
    config.ctCfg[idxCT].ctCal    = 100.0f;
    config.ctCfg[idxCT].phase    = 4.2f;
    config.ctCfg[idxCT].vChan1   = 0;
    config.ctCfg[idxCT].vChan2   = 0;
    config.ctCfg[idxCT].ctActive = (idxCT < NUM_CT_ACTIVE_DEF);
  }

  /* OneWire/Pulse configuration:
   *   - Pulse input
   *   - Period: 100 ms
   *   - Rising edge trigger
   *   - Pull up disabled
   *   - All disabled
   */
  for (int i = 0u; i < NUM_OPA; i++) {
    config.pulseCfg[i].pulseActive = false;
    config.pulseCfg[i].period      = 100u;
    config.pulseCfg[i].func        = 'r';
    config.pulseCfg[i].puEn        = false;
  }

  config.crc16_ccitt = calcCRC16_ccitt(&config, (sizeof(config) - 2u));
}

/*! @brief Write the configuration values to index 0, and zero the
 *         accumulator space to.
 */
static void configInitialiseNVM(void) {

  dbgPuts("  - Initialising NVM... ");

  configDefault();
  eepromInitBlock(0, 0, 256);
  eepromInitConfig(&config, sizeof(config));

  eepromInitBlock(EEPROM_WL_OFFSET, 0, (EEPROM_SIZE - EEPROM_WL_OFFSET));
  dbgPuts("Done!\r\n");
}

static bool configureAnalog(void) {
  /* String format: k<x> <a> <y.y> <z.z> v1 v2
   * Find space delimiters, then convert to null and a->i/f
   */
  ConvFloat_t convF     = {false, 0.0f};
  ConvInt_t   convI     = {false, 0};
  int         ch        = 0;
  bool        active    = false;
  float       calAmpl   = 0.0f;
  float       calPhase  = 0.0f;
  int         vCh1      = 0;
  int         vCh2      = 0;
  int         posActive = 0;
  int         posCalib  = 0;
  int         posPhase  = 0;
  int         posV1     = 0;
  int         posV2     = 0;
  ECMCfg_t   *ecmCfg    = 0;

  for (int i = 0; i < IN_BUFFER_W; i++) {
    if (0 == inBuffer[i]) {
      break;
    }
    if (' ' == inBuffer[i]) {
      inBuffer[i] = 0;
      if (0 == posActive) {
        posActive = i + 1;
      } else if (0 == posCalib) {
        posCalib = i + 1;
      } else if (0 == posPhase) {
        posPhase = i + 1;
      } else if (0 == posV1) {
        posV1 = i + 1;
      } else if (0 == posV2) {
        posV2 = i + 1;
        break;
      }
    }
  }

  /* Voltage channels are [1..3], CTs are [4..] but 0 indexed internally. All
   * fields must be present for a given channel type.
   */
  convI = utilAtoi(inBuffer + 1u, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  ch = convI.val - 1;

  if ((ch < 0) || (ch >= VCT_TOTAL)) {
    return false;
  }

  if ((0 == posCalib) || (0 == posActive)) {
    return false;
  }
  if (ch >= NUM_V) {
    if ((0 == posPhase) || (0 == posV1) || (0 == posV2)) {
      return false;
    }
  }

  ecmCfg = ecmConfigGet();
  EMON32_ASSERT(ecmCfg);

  convI = utilAtoi(inBuffer + posActive, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  active = (bool)convI.val;

  convF = utilAtof(inBuffer + posCalib);
  if (!convF.valid) {
    return false;
  }
  calAmpl = convF.val;

  if (NUM_V > ch) {

    if ((calAmpl <= 25.0f) || (calAmpl >= 150.0f)) {
      return false;
    }

    config.voltageCfg[ch].vActive    = active;
    config.voltageCfg[ch].voltageCal = calAmpl;
    ecmCfg->vCfg[ch].vActive         = active;
    ecmCfg->vCfg[ch].voltageCalRaw   = calAmpl;

    printf_("> V%d calibration set to: ", (ch + 1));
    putFloat(config.voltageCfg[ch].voltageCal, 0);
    dbgPuts("\r\n");

    ecmConfigChannel(ch);
    return true;
  }

  convF = utilAtof(inBuffer + posPhase);
  if (!convF.valid) {
    return false;
  }
  calPhase = convF.val;

  convI = utilAtoi(inBuffer + posV1, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  vCh1 = convI.val;

  convI = utilAtoi(inBuffer + posV2, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  vCh2 = convI.val;

  if ((vCh1 < 1) || (vCh1 > NUM_V) || (vCh2 < 1) || (vCh2 > NUM_V)) {
    return false;
  }

  /* CT configuration - assume 10/200 A min/max CTs */
  if ((calAmpl < 10.0f) || (calAmpl) > 200.0f) {
    return false;
  }

  ch -= NUM_V;
  config.ctCfg[ch].ctActive = active;
  ecmCfg->ctCfg[ch].active  = active;

  config.ctCfg[ch].ctCal     = calAmpl;
  ecmCfg->ctCfg[ch].ctCalRaw = calAmpl;
  printf_("> CT%d calibration set to: ", (ch + 1));
  putFloat(config.ctCfg[ch].ctCal, 0);
  dbgPuts("\r\n");

  config.ctCfg[ch].phase  = calPhase;
  ecmCfg->ctCfg[ch].phCal = calPhase;
  printf_("> CT%d phase set to: ", (ch + 1));
  putFloat(config.ctCfg[ch].phase, 0);
  dbgPuts("\r\n");

  config.ctCfg[ch].vChan1  = vCh1 - 1;
  ecmCfg->ctCfg[ch].vChan1 = vCh1 - 1;
  printf_("> CT%d voltage channel 1 set to: %d\r\n", (ch + 1), vCh1);

  config.ctCfg[ch].vChan2  = vCh2 - 1;
  ecmCfg->ctCfg[ch].vChan2 = vCh2 - 1;
  printf_("> CT%d voltage channel 1 set to: %d\r\n", (ch + 1), vCh2);

  ecmConfigChannel(ch + NUM_V);
  return true;
}

static bool configureAssumed(void) {
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (convI.valid) {
    ECMCfg_t *pEcmCfg          = ecmConfigGet();
    pEcmCfg->assumedVrms       = convI.val;
    config.baseCfg.assumedVrms = convI.val;
    return true;
  }
  return false;
}

static bool configureDatalog(void) {
  ConvFloat_t convF = utilAtof(inBuffer + 1);
  /* Set the datalog period (s) in range 0.5 <= t <= 600 */
  if (convF.valid) {
    if ((convF.val < 0.5f) || (convF.val > 600.0f)) {
      dbgPuts("> Log report time out of range.\r\n");
    } else {
      config.baseCfg.reportTime = convF.val;
      ecmConfigReportCycles(
          configTimeToCycles(convF.val, config.baseCfg.mainsFreq));

      dbgPuts("> Data log report time set to: ");
      putFloat(config.baseCfg.reportTime, 0);
      dbgPuts("\r\n");
      return true;
    }
  }
  return false;
}

static void configureOPA(void) {
  /* String format in inBuffer:
   *      [1] -> ch;
   *      [3] -> active;
   *      [5] -> edge (rising, falling, both)
   *      [7] -> NULL: blank time
   */
  ConvInt_t convI;
  int       ch     = 0;
  int       active = 0;
  int       period = 0;
  char      edge   = 0;

  convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return;
  }
  ch = convI.val - 1;

  if ((ch < 0) || (ch >= NUM_OPA)) {
    return;
  }

  convI = utilAtoi(inBuffer + 3, ITOA_BASE10);
  if (!convI.valid) {
    return;
  }
  active = (bool)convI.val;

  convI = utilAtoi((inBuffer + 7), ITOA_BASE10);
  if (!convI.valid) {
    return;
  }
  period = convI.val;

  edge = inBuffer[5];
  if (!(('r' == edge) || ('f' == edge) || ('b' == edge))) {
    return;
  }

  /* If inactive, clear active flag, no decode for the rest */
  if (0 == active) {
    config.pulseCfg[ch].pulseActive = false;
    printf_("> Pulse channel %d disabled.\r\n", (ch + 1u));
    return;
  } else {
    config.pulseCfg[ch].pulseActive = true;
    printf_("> Pulse channel %d: ", (ch + 1u));
    switch (edge) {
    case 'r':
      dbgPuts("Rising, ");
      config.pulseCfg[ch].func = 'r';
      break;
    case 'f':
      dbgPuts("Falling, ");
      config.pulseCfg[ch].func = 'f';
      break;
    case 'b':
      dbgPuts("Both, ");
      config.pulseCfg[ch].func = 'b';
      break;
    }
    config.pulseCfg[ch].period = period;
    printf_("%d ms\r\n", config.pulseCfg[ch].period);
  }
}

static bool configureSerialLog(void) {
  /* Log to serial output, default TRUE
   * Format: c0 | c1
   */
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);

  if (convI.valid) {
    config.baseCfg.logToSerial = (bool)convI.val;
    printf_("> Log to serial: %c\r\n", config.baseCfg.logToSerial ? 'Y' : 'N');
    return true;
  }
  return false;
}

static bool configureWhDelta(void) {
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);

  if (convI.val < 10) {
    return false;
  }

  if (convI.valid) {
    config.baseCfg.whDeltaStore = convI.val;
    printf_("> Energy delta set to: %d\r\n", config.baseCfg.whDeltaStore);
    return true;
  }
  return false;
}

static void enterBootloader(void) {
  /* Linker reserves 4 bytes at the bottom of the stack and write the UF2
   * bootloader key followed by reset. Will enter bootloader upon reset. */
  char               c;
  volatile uint32_t *p_blsm =
      (volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4);
  // Key is uf2-samdx1/inc/uf2.h:DBL_TAP_MAGIC
  const uint32_t blsm_key = 0xF01669EF;
  dbgPuts("> Enter bootloader? All unsaved changes will be lost. 'y' to "
          "proceed.\r\n");
  c = waitForChar();
  if ('y' == c) {
    *p_blsm = blsm_key;
    NVIC_SystemReset();
  } else {
    dbgPuts("    - Cancelled.");
  }
}

/*! @brief Get the board revision, software visible changes only
 *  @return : board revision, 0-7
 */
static uint32_t getBoardRevision(void) {
  uint32_t boardRev = 0;
  boardRev |= portPinValue(GRP_REV0, PIN_REV0);
  boardRev |= portPinValue(GRP_REV1, PIN_REV1) << 1;
  boardRev |= portPinValue(GRP_REV2, PIN_REV2) << 2;
  return boardRev;
}

/*! @brief Get the last reset cause (16.8.14)
 *  @return : null-terminated string with the last cause.
 */
static char *getLastReset(void) {
  const RCAUSE_t lastReset = (RCAUSE_t)PM->RCAUSE.reg;
  switch (lastReset) {
  case RCAUSE_SYST:
    return "Reset request";
    break;
  case RCAUSE_WDT:
    return "Watchdog timeout";
    break;
  case RCAUSE_EXT:
    return "External reset";
    break;
  case RCAUSE_BOD33:
    return "3V3 brownout";
    break;
  case RCAUSE_BOD12:
    return "1V2 brownout";
    break;
  case RCAUSE_POR:
    return "Power on cold reset";
    break;
  }
  return "Unknown";
}

/*! @brief Fetch the SAMD's 128bit unique ID
 *  @param [in] idx : index of 32bit word
 *  @return : 32bit word from index
 */
uint32_t getUniqueID(int idx) {
  /* Section 10.3.3 Serial Number */
  const uint32_t id_addr_lut[4] = {0x0080A00C, 0x0080A040, 0x0080A044,
                                   0x0080A048};
  return *(volatile uint32_t *)id_addr_lut[idx];
}

static void inBufferClear(int n) {
  inBufferIdx = 0;
  (void)memset(inBuffer, 0, n);
}

static void printSettings(void) {
  dbgPuts("\r\n\r\n==== Settings ====\r\n\r\n");
  printf_("Mains frequency (Hz)       %d\r\n", config.baseCfg.mainsFreq);
  dbgPuts("Data log time (s):         ");
  putFloat(config.baseCfg.reportTime, 0);
  printf_("\r\nMinimum accumulation (Wh): %d\r\n", config.baseCfg.whDeltaStore);
  dbgPuts("Data transmission:         ");
  if (DATATX_RFM69 == (TxType_t)config.dataTxCfg.txType) {
    dbgPuts("RFM69, ");
    switch (config.dataTxCfg.rfmFreq) {
    case 0:
      dbgPuts("868");
      break;
    case 1:
      dbgPuts("915");
      break;
    case 2:
      dbgPuts("433");
      break;
    }
    printf_(" MHz, power %d\r\n", config.dataTxCfg.rfmPwr);
  } else {
    dbgPuts("Serial\r\n");
  }
  printf_("Data format:               %s\r\n",
          config.baseCfg.useJson ? "JSON" : "Key:Value");
  dbgPuts("\r\n");

  for (unsigned int i = 0; i < NUM_OPA; i++) {
    bool enabled = config.pulseCfg[i].pulseActive;
    printf_("Pulse Channel %d (%sactive)\r\n", (i + 1), enabled ? "" : "in");
    printf_("  - Hysteresis (ms): %d\r\n", config.pulseCfg[i].period);
    dbgPuts("  - Edge:            ");
    switch (config.pulseCfg[i].func) {
    case 'r':
      dbgPuts("Rising");
      break;
    case 'f':
      dbgPuts("Falling");
      break;
    case 'b':
      dbgPuts("Both");
      break;
    default:
      dbgPuts("Unknown");
    }
    dbgPuts("\r\n\r\n");
  }

  dbgPuts(
      "| Ref | Channel | Active | Calibration | Phase  | In 1 | In 2 |\r\n");
  dbgPuts(
      "+=====+=========+========+=============+========+======+======+\r\n");
  for (int i = 0; i < NUM_V; i++) {
    printf_("| %2d  |  V %2d   | %c      | ", (i + 1), (i + 1),
            (config.voltageCfg[i].vActive ? 'Y' : 'N'));
    putFloat(config.voltageCfg[i].voltageCal, 6);
    dbgPuts("      |        |      |      |\r\n");
  }
  for (int i = 0; i < NUM_CT; i++) {
    printf_("| %2d  | CT %2d   | %c      | ", (i + 1 + NUM_V), (i + 1),
            (config.ctCfg[i].ctActive ? 'Y' : 'N'));
    putFloat(config.ctCfg[i].ctCal, 6);
    dbgPuts("      | ");
    putFloat(config.ctCfg[i].phase, 6);
    printf_(" | %d    | %d    |\r\n", (config.ctCfg[i].vChan1 + 1),
            (config.ctCfg[i].vChan2 + 1));
  }
  dbgPuts("\r\n");

  if (unsavedChange) {
    dbgPuts("There are unsaved changes. Command \"s\" to save.\r\n\r\n");
  }
}

static void putFloat(float val, int flt_len) {
  char strBuffer[16];
  int  ftoalen = utilFtoa(strBuffer, val);

  if (flt_len) {
    int fillSpace = flt_len - ftoalen;

    while (fillSpace--) {
      dbgPuts(" ");
    }
  }

  dbgPuts(strBuffer);
}

static void printUptime(void) {

  uint32_t tSeconds = timerUptime();
  uint32_t tMinutes = tSeconds / 60;
  uint32_t tHours   = tMinutes / 60;
  uint32_t tDays    = tHours / 24;

  tSeconds = tSeconds % 60;
  tMinutes = tMinutes % 60;
  tHours   = tHours % 24;

  printf_("%" PRIu32 "d %" PRIu32 "h %" PRIu32 "m %" PRIu32 "s\r\n", tDays,
          tHours, tMinutes, tSeconds);
}

/*! @brief Blocking wait for a key from the serial link. If the USB CDC is
 *         connected the key will come from here.
 */
static char waitForChar(void) {
  /* Disable the NVIC for the interrupt if needed while waiting for the
   * character otherwise it is handled by the configuration buffer.
   */
  char c;
  if (usbCDCIsConnected()) {
    while (!usbCDCRxAvailable())
      ;
    c = usbCDCRxGetChar();
  } else {
    int irqEnabled = (NVIC->ISER[0] &
                      (1 << ((uint32_t)(SERCOM_UART_INTERACTIVE_IRQn) & 0x1F)))
                         ? 1
                         : 0;
    if (irqEnabled)
      NVIC_DisableIRQ(SERCOM_UART_INTERACTIVE_IRQn);

    while (0 ==
           (uartInterruptStatus(SERCOM_UART_DBG) & SERCOM_USART_INTFLAG_RXC))
      ;
    c = uartGetc(SERCOM_UART_DBG);

    if (irqEnabled)
      NVIC_EnableIRQ(SERCOM_UART_INTERACTIVE_IRQn);
  }

  return c;
}

/*! @brief Zero the accumulator portion of the NVM
 *  @return true if cleared, false if cancelled
 */
static bool zeroAccumulators(void) {
  char c;
  dbgPuts("> Zero accumulators. This can not be undone. 'y' to proceed.\r\n");

  c = waitForChar();
  if ('y' == c) {
    eepromInitBlock(EEPROM_WL_OFFSET, 0, (1024 - EEPROM_WL_OFFSET));
    dbgPuts("    - Accumulators cleared.\r\n");
    return true;
  } else {
    dbgPuts("    - Cancelled.\r\n");
    return false;
  }
}

void configCmdChar(const uint8_t c) {
  if (('\r' == c) || ('\n' == c)) {
    if (!cmdPending) {
      dbgPuts("\r\n");
      cmdPending = true;
      emon32EventSet(EVT_PROCESS_CMD);
    }
  } else if ('\b' == c) {
    dbgPuts("\b \b");
    if (0 != inBufferIdx) {
      inBufferIdx--;
      inBuffer[inBufferIdx] = 0;
    }
  } else if ((inBufferIdx < IN_BUFFER_W) && utilCharPrintable(c)) {
    inBuffer[inBufferIdx++] = c;
  } else {
    inBufferClear(IN_BUFFER_W);
    dbgPuts("\r\n");
  }
}

void configFirmwareBoardInfo(void) {
  dbgPuts("\033c==== emon32 ====\r\n\r\n");

  dbgPuts("> Board:\r\n");
  printf_("  - emonPi3     (arch. rev. %" PRIu32 ")\r\n", getBoardRevision());
  printf_("  - Serial:     0x%02x%02x%02x%02x\r\n",
          (unsigned int)getUniqueID(0), (unsigned int)getUniqueID(1),
          (unsigned int)getUniqueID(2), (unsigned int)getUniqueID(3));
  printf_("  - Last reset: %s\r\n", getLastReset());
  dbgPuts("  - Uptime    : ");
  printUptime();
  dbgPuts("\r\n");

  dbgPuts("> Firmware:\r\n");
  printf_("  - Version:    %d.%d.%d\r\n", VERSION_FW_MAJ, VERSION_FW_MIN,
          VERSION_FW_REV);
  dbgPuts("  - Build:      ");
  dbgPuts(emon32_build_info_string());
  dbgPuts("\r\n\r\n");
  dbgPuts("  - Distributed under GPL3 license, see COPYING.md\r\n");
  dbgPuts("  - emon32 Copyright (C) 2023-24 Angus Logan\r\n");
  dbgPuts("  - For Bear and Moose\r\n\r\n");
}

Emon32Config_t *configGetConfig(void) { return &config; }

void configLoadFromNVM(void) {

  const uint32_t cfgSize     = sizeof(config);
  uint16_t       crc16_ccitt = 0;
  char           c           = 0;

  /* Load from "static" part of EEPROM. If the key does not match
   * CONFIG_NVM_KEY, write the default configuration to the EEPROM and zero
   * wear levelled portion. Otherwise, read configuration from EEPROM.
   */
  eepromRead(0, &config, cfgSize);

  if (CONFIG_NVM_KEY != config.key) {
    configInitialiseNVM();
  } else {
    /* Check the CRC and raise a warning if not matched. -2 from the base
     * size to account for the stored 16 bit CRC.
     */
    crc16_ccitt = calcCRC16_ccitt(&config, cfgSize - 2u);
    if (crc16_ccitt != config.crc16_ccitt) {
      printf_("  - CRC mismatch. Found: 0x%04x -- Expected: 0x%04x\r\n",
              config.crc16_ccitt, crc16_ccitt);
      dbgPuts("    - NVM may be corrupt. Overwrite with default? (y/n)\r\n");
      while ('y' != c && 'n' != c) {
        c = waitForChar();
      }
      if ('y' == c) {
        configInitialiseNVM();
      }
    }
  }

  config.baseCfg.reportCycles =
      configTimeToCycles(config.baseCfg.reportTime, config.baseCfg.mainsFreq);
}

void configProcessCmd(void) {
  unsigned int arglen    = 0;
  bool         termFound = false;
  ConvInt_t    convI     = {false, 0};

  /* Help text - serves as documentation interally as well */
  const char helpText[] =
      "\r\n"
      "emon32 information and configuration commands\r\n\r\n"
      " - ?           : show this text again\r\n"
      " - a<n>        : set the assumed RMS voltage as integer\r\n"
      " - b<n>        : set RF band. n = 4: 433 MHz, 8: 868 MHz, 9: 915 MHz\r\n"
      " - c<n>        : log to serial output. n = 0: OFF, n = 1: ON\r\n"
      " - d<x.x>      : data log period (s)\r\n"
      " - f<n>        : line frequency (Hz)\r\n"
      " - e           : enter bootloader\r\n"
      " - g<n>        : set network group (default = 210)\r\n"
      " - j<n>        : JSON serial format. n = 0: OFF, n = 1: ON\r\n"
      " - k<x> <a> <y.y> <z.z> v1 v2\r\n"
      "   - Configure an analogue input\r\n"
      "   - x:        : channel (1-3 -> V; 4... -> CT)\r\n"
      "   - a:        : channel active. a = 0: DISABLED, a = 1: ENABLED\r\n"
      "   - y.y       : V/CT calibration constant\r\n"
      "   - z.z       : CT phase calibration value\r\n"
      "   - v1        : CT voltage channel 1\r\n"
      "   - v2        : CT voltage channel 2\r\n"
      " - l           : list settings\r\n"
      " - m<v> <w> <x> <y> <z>\r\n"
      "   - Configure a OneWire/pulse input.\r\n"
      "     - v : channel index\r\n"
      "     - w : function select. w = p: pulse, w = o: OneWire.\r\n"
      "     - x : edge sensitivity (r,f,b). Ignored if w = o\r\n"
      "     - y : minimum period (ms). Ignored if w = o\r\n"
      "     - z : pull-up. z = 1: PULL UP, z = 0: NO PULL UP\r\n"
      " - n<n>        : set node ID [1..60]\r\n"
      " - p<n>        : set the RF power level\r\n"
      " - r           : restore defaults\r\n"
      " - s           : save settings to NVM\r\n"
      " - t           : trigger report on next cycle\r\n"
      " - v           : firmware and board information\r\n"
      " - w<n>        : minimum difference in energy before saving (Wh)\r\n"
      " - z           : zero energy accumulators\r\n\r\n";

  /* Convert \r or \n to 0, and get the length until then. */
  while (!termFound && (arglen < IN_BUFFER_W)) {
    if (0 == inBuffer[arglen]) {
      termFound = true;
      break;
    }
    arglen++;
  }

  if (!termFound) {
    return;
  }

  /* Decode on first character in the buffer */
  switch (inBuffer[0]) {
  case '?':
    /* Print help text */
    dbgPuts(helpText);
    break;
  case 'a':
    if (configureAssumed()) {
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'c':
    if (configureSerialLog()) {
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'd':
    if (configureDatalog()) {
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'e':
    /* Enter the bootloader through firmware */
    enterBootloader();
    break;
  case 'f':
    /* Set line frequency.
     * Format: f50 | f60
     */
    if (3u == arglen) {
      convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
      if (!convI.valid) {
        break;
      }

      if (!((50 == convI.val) || (60 == convI.val))) {
        break;
      }

      config.baseCfg.mainsFreq = convI.val;

      printf_("> Mains frequency set to: %d\r\n", config.baseCfg.mainsFreq);

      unsavedChange = true;
      resetReq      = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'j':
    if (2u == arglen) {
      convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
      if (!convI.valid) {
        break;
      }

      config.baseCfg.useJson = (bool)convI.val;

      printf_("> Use JSON: %c\r\n", config.baseCfg.useJson ? 'Y' : 'N');

      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'k':
    if (configureAnalog()) {
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'l':
    printSettings();
    break;
  case 'm':
    configureOPA();
    unsavedChange = true;
    emon32EventSet(EVT_CONFIG_CHANGED);
    break;
  case 'o':
    /* Start auto calibration of CT<x> lead */
    dbgPuts("> Reserved for auto calibration. Not yet implemented.\r\n");
    break;
  case 'p':
    /* Configure RF power */
    resetReq      = true;
    unsavedChange = true;
    emon32EventSet(EVT_CONFIG_CHANGED);
    break;
  case 'r':
    configDefault();

    dbgPuts("> Restored default values.\r\n");

    unsavedChange = true;
    resetReq      = true;
    emon32EventSet(EVT_CONFIG_CHANGED);
    break;
  case 's':
    /* Save to EEPROM config space after recalculating CRC and indicate if a
     * reset is required.
     */
    config.crc16_ccitt = calcCRC16_ccitt(&config, (sizeof(config) - 2));

    dbgPuts("> Saving configuration to NVM... ");
    eepromInitConfig(&config, sizeof(config));
    dbgPuts("Done!\r\n");

    unsavedChange = false;
    if (!resetReq) {
      emon32EventSet(EVT_CONFIG_SAVED);
    } else {
      emon32EventSet(EVT_SAFE_RESET_REQ);
    }
    break;
  case 't':
    /* Trigger processing on set on next cycle complete */
    emon32EventSet(EVT_ECM_TRIG);
    break;
  case 'v':
    configFirmwareBoardInfo();
    break;
  case 'w':
    if (configureWhDelta()) {
      unsavedChange = true;
      emon32EventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'z':
    if (zeroAccumulators()) {
      emon32EventSet(EVT_CLEAR_ACCUM);
    }
    break;
  }

  cmdPending = false;
  inBufferClear(arglen + 1);
}

int configTimeToCycles(const float time, const int mainsFreq) {
  return qfp_float2uint(qfp_fmul(time, qfp_int2float(mainsFreq)));
}

/* =======================
 * UART Interrupt handler
 * ======================= */

void SERCOM_UART_INTERACTIVE_HANDLER {
  /* Echo the received character to the TX channel, and send to the command
   * stream.
   */
  if (uartGetcReady(SERCOM_UART_INTERACTIVE)) {
    uint8_t rx_char = uartGetc(SERCOM_UART_INTERACTIVE);
    configCmdChar(rx_char);

    if (utilCharPrintable(rx_char) && !cmdPending) {
      uartPutcBlocking(SERCOM_UART_INTERACTIVE, rx_char);
    }
  }
}
