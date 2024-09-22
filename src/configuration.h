#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board_def.h"

/* Configurable options. All the structs are packed to allow simple write to
 * EEPROM as a contiguous set.
 */
typedef struct __attribute__((__packed__)) BaseCfg_ {
  uint8_t  nodeID;       /* ID for report*/
  uint8_t  mainsFreq;    /* Mains frequency */
  uint16_t reportCycles; /* Cycles between reports */
  uint16_t whDeltaStore; /* Minimum energy delta to store */
  uint8_t  dataGrp;      /* Transmission group - default 210 */
  bool     logToSerial;  /* Log data to serial output */
  bool     useJson;      /* JSON format for serial output */
  uint8_t  assumedVrms;  /* Assumed RMS voltage if not present */
  uint8_t  res0[10];
  float    reportTime; /* Time between reports */
} BaseCfg_t;

typedef enum DataTx_ { DATATX_RFM69 = 0, DATATX_UART = 1 } TxType_t;

typedef struct __attribute__((__packed__)) DataTxCfg_ {
  uint8_t txType;  /* UART or RFM on SPI */
  uint8_t rfmFreq; /* 0: 868 MHz, 1: 915 MHz, 2: 433 MHz. */
  uint8_t rfmPwr;
  uint8_t res0;
} DataTxCfg_t;

typedef struct __attribute__((__packed__)) PulseCfgPacked_ {
  uint8_t period;
  uint8_t edge;
  bool    pulseActive;
  uint8_t res0;
} PulseCfgPacked_t;

typedef struct __attribute__((__packed__)) VoltageCfg_ {
  float   voltageCal; /* Conversion to real V value */
  bool    vActive;
  uint8_t res0[3];
} VoltageCfgPacked_t;

typedef struct __attribute__((__packed__)) CTCfg_ {
  float   ctCal; /* Conversion to real I value */
  float   phase; /* Phase angle, recalculated to fixed point */
  uint8_t vChan1;
  bool    ctActive;
  uint8_t vChan2;
  uint8_t res0;
} CTCfgPacked_t;

typedef struct __attribute__((__packed__)) Emon32Config_ {
  uint32_t           key;
  BaseCfg_t          baseCfg;
  DataTxCfg_t        dataTxCfg;
  VoltageCfgPacked_t voltageCfg[NUM_V];
  CTCfgPacked_t      ctCfg[NUM_CT];
  PulseCfgPacked_t   pulseCfg[NUM_PULSECOUNT];
  uint8_t            res0[17];
  uint16_t           crc16_ccitt;
} Emon32Config_t;

_Static_assert((sizeof(BaseCfg_t) == 24), "BaseCfg_t is not 24 bytes wide.");
_Static_assert((sizeof(DataTxCfg_t) == 4), "DataTxCfg_t is not 4 bytes wide.");
_Static_assert((sizeof(PulseCfgPacked_t) == 4),
               "PulseCfgPacked_t is not 4 bytes wide.");
_Static_assert((sizeof(VoltageCfgPacked_t) == 8),
               "VoltageCfgPacked_t is not 8 bytes wide.");
_Static_assert((sizeof(CTCfgPacked_t) == 12),
               "CTCfgPacked_t is not 12 bytes wide.");

/* Check the configuration struct will fit within the "static" area */
_Static_assert((sizeof(Emon32Config_t) < EEPROM_WL_OFFSET),
               "Emon32Config_t >= EEPROM_WL_OFFSET");

/*! @brief Add a character to the command stream
 *  @param [in] c : character to add
 */
void configCmdChar(const uint8_t c);

/*! @brief Print the board and firmware information to serial */
void configFirmwareBoardInfo(void);

/*! @brief Get the configuration pointer
 *  @return pointer to the configuration struct
 */
Emon32Config_t *configGetConfig(void);

/*! @brief This functions loads the default configuration and from NVM.
 */
void configLoadFromNVM(void);

/*! @brief Process a pending command from the UART */
void configProcessCmd(void);

/*! @brief Return one word from the SAMD's unique ID
 *  @param[in] idx : index of the word to fetch
 *  @return word idx from the unique ID
 */
uint32_t getUniqueID(int idx);
