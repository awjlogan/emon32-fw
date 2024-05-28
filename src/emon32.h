#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "emon_CM.h"

_Static_assert((sizeof(bool) == 1), "bool must be 1 byte");

/*********************************
 * Common configurable options
 *********************************/

#define DELTA_WH_STORE      200u    /* Threshold in WH to store to NVM */
#define DOWNSAMPLE_DSP      1u      /* 0: no downsampling; 1: half band LPF */
#define NODE_ID             17u     /* Node ID for reports */
#define PERF_ENABLED        1u      /* Performance tracing enabled (1) or disabled (0) */
#define TX_INDICATE_T       250u    /* Transmission indication time (ms) */

/*********************************
 * Firmware version
 *********************************/
#define VERSION_FW_MAJ      0u
#define VERSION_FW_MIN      0u
#define VERSION_FW_REV      1u

/*********************************
 * Remaining
 *********************************/
#define TX_BUFFER_W         448u

/* Configuration key - indicates that the configuration is the default or
 * has been retrieved from non-volatile storage */
#define CONFIG_NVM_KEY      0xca55e77eul

/* Pulse count setup */
#define NUM_PULSECOUNT      2

/* Configurable options. All the structs are packed to allow simple write to
 * EEPROM as a contiguous set.
 */

typedef struct __attribute__((__packed__)) BaseCfg_ {
    uint8_t     nodeID;         /* ID for report*/
    uint8_t     mainsFreq;      /* Mains frequency */
    float       reportTime;     /* Time between reports */
    uint16_t    reportCycles;   /* Cycles between reports */
    uint16_t    whDeltaStore;   /* Minimum energy delta to store */
    uint8_t     dataGrp;        /* Transmission group - default 210 */
    bool        logToSerial;    /* Log data to serial output */
    bool        useJson;        /* JSON format for serial output */
} BaseCfg_t;

typedef enum __attribute__ ((__packed__)) DataTx_ {
    DATATX_RFM69    = 0,
    DATATX_UART     = 1
} TxType_t;

typedef struct __attribute__((__packed__)) DataTxCfg_ {
    TxType_t    txType;     /* UART or RFM on SPI */
    uint8_t     rfmFreq;    /* 0: 868 MHz, 1: 915 MHz, 2: 433 MHz. */
    uint8_t     rfmPwr;
} DataTxCfg_t;

typedef struct __attribute__((__packed__)) PulseCfgPacked_ {
    uint8_t     period;
    uint8_t     edge;
} PulseCfgPacked_t;

typedef struct __attribute__((__packed__)) VoltageCfg_ {
   float        voltageCal;      /* Conversion to real V value */
} VoltageCfg_t;

typedef struct __attribute__((__packed__)) CTCfg_ {
    float   ctCal;          /* Conversion to real I value */
    float   phase;          /* Phase angle, recalculated to fixed point */
    uint8_t vChan;
} CTCfg_t;

typedef struct __attribute__((__packed__)) Emon32Config_ {
    uint32_t            key;
    BaseCfg_t           baseCfg;
    DataTxCfg_t         dataTxCfg;
    VoltageCfg_t        voltageCfg[NUM_V];
    CTCfg_t             ctCfg[NUM_CT];
    uint32_t            ctActive;       /* Bitmap of active inputs */
    PulseCfgPacked_t    pulseCfg[NUM_PULSECOUNT];
    uint8_t             pulseActive;
    uint16_t            crc16_ccitt;
} Emon32Config_t;

typedef struct Emon32Dataset_ {
    uint32_t        msgNum;
    ECMDataset_t    *pECM;
    uint32_t        pulseCnt[NUM_PULSECOUNT];
    float           temp[TEMP_MAX_ONEWIRE];
    unsigned int    numTempSensors;
} Emon32Dataset_t;

typedef struct __attribute__((__packed__)) Emon32Cumulative_ {
    uint32_t    wattHour[NUM_CT];
    uint32_t    pulseCnt[NUM_PULSECOUNT];
} Emon32Cumulative_t;

typedef struct __attribute__((__packed__)) Emon32CumulativeSave_ {
    uint8_t             valid;  /* Valid byte for wear levelling */
    Emon32Cumulative_t  report;
    uint16_t            crc;    /* CRC16-CCITT of data */
} Emon32CumulativeSave_t;

/* This struct must match the OEM definitions found at:
 * https://docs.openenergymonitor.org/electricity-monitoring/networking/sending-data-between-nodes-rfm.html
 */
typedef struct __attribute__((__packed__)) PackedData_ {
    uint32_t    msg;
    int16_t     V[NUM_V];
    int16_t     P[NUM_CT];
    int32_t     E[NUM_CT];
    int16_t     T[TEMP_MAX_ONEWIRE];
    uint32_t    pulse[NUM_PULSECOUNT];
} PackedData_t;


/* EVTSRC_t contains all the event/interrupts sources. This value is shifted
 * to provide a vector of set events as bits.
 */
typedef enum EVTSRC_ {
    EVT_DMA             = 0u,
    EVT_TICK_1kHz       = 1u,
    EVT_TCC             = 2u,
    EVT_UART            = 3u,
    EVT_ADC             = 4u,
    EVT_DMAC_UART_CMPL  = 5u,
    EVT_DMAC_SMP_CMPL   = 6u,
    EVT_ECM_CYCLE_CMPL  = 7u,
    EVT_ECM_SET_CMPL    = 8u,
    EVT_SAVE_RESET      = 9u,
    EVT_DMAC_I2C_CMPL   = 10u,
    EVT_TIMER_MC        = 11u,
    EVT_EIC_PULSE       = 12u,
    EVT_EEPROM_TMR      = 13u,
    EVT_TEMP_SAMPLE     = 14u,
    EVT_TEMP_READ       = 15u,
    EVT_CONFIG_CHANGED  = 16u,
    EVT_CONFIG_SAVED    = 17u,
    EVT_SAFE_RESET_REQ  = 18u,
    EVT_PROCESS_CMD     = 19u,
    EVT_PROCESS_DATASET = 20u,
    EVT_EEPROM_STORE    = 21u
} EVTSRC_t;


/*! @brief Output a string to the debug destination
 *  @param [in] s: pointer to null terminated string s
 */
void dbgPuts(const char *s);

/*! @brief Clear a pending event/interrupt flag after the task has been handled
 *  @param [in] Event source in enum
 */
void emon32EventClr(const EVTSRC_t evt);

/*! @brief Set the pending event/interrupt flag for tasks that are not handled
 *         within an ISR
 *  @param [in] evt : Event source in enum
 */
void emon32EventSet(const EVTSRC_t evt);
