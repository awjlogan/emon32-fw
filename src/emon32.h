#pragma once

#include <stdint.h>
#include "emon_CM.h"

/* Firmware version */
#define VERSION_FW_MAJ      0u
#define VERSION_FW_MIN      1u

#define NODE_ID             17u
#define TX_BUFFER_W         64u

/* Configuration key - indicates that the configuration is the default or
 * has been retrieved from non-volatile storage */
#define CONFIG_NVM_KEY      0xca55e77eul

/* Voltage and CT setup.
 */
#define DELTA_WH_STORE      200u    /* Threshold to store to non-volatile */

/* Pulse count setup */
#define NUM_PULSECOUNT      2

/* Set to 1 to downsample the sample rate by low pass filter. Otherwise, the
 * second sample from each set will be discarded.
 */
#define DOWNSAMPLE_DSP      1u

/* Configurable options. All the structs are packed to allow simple write to
 * EEPROM as a contiguous set.
 */
typedef enum __attribute__ ((__packed__)) {
    DATATX_RFM69    = 0,
    DATATX_UART     = 1
} DataTx_t;

typedef struct __attribute__((__packed__)) {
    uint8_t     nodeID;         /* ID for report*/
    uint8_t     mainsFreq;      /* Mains frequency */
    float       reportTime;     /* Cycle count before reporting */
    uint16_t    whDeltaStore;   /* Minimum energy delta to store */
    DataTx_t    dataTx;         /* Data transmission hardware type */
    uint8_t     dataGrp;        /* Transmission group - default 210 */
    uint8_t     logToSerial;    /* Log data to serial output */
} BaseCfg_t;

typedef struct __attribute__((__packed__)) {
    uint8_t     period;
    uint8_t     edge;
} PulseCfgPacked_t;

typedef struct __attribute__((__packed__)) {
   float        voltageCal;      /* Conversion to real V value */
} VoltageCfg_t;

typedef struct __attribute__((__packed__)) {
    float   ctCal;          /* Conversion to real I value */
    q15_t   phaseX;         /* Phase calibrations for interpolation */
    q15_t   phaseY;
    uint8_t vChan;
} CTCfg_t;

typedef struct __attribute__((__packed__)) {
    uint32_t            key;
    BaseCfg_t           baseCfg;
    float               voltageAssumed;
    VoltageCfg_t        voltageCfg[NUM_V];
    CTCfg_t             ctCfg[NUM_CT];
    uint32_t            ctActive;       /* Multihot active bits */
    PulseCfgPacked_t    pulseCfg[NUM_PULSECOUNT];
    uint8_t             pulseActive;
    uint16_t            crc16_ccitt;
} Emon32Config_t;

typedef struct {
    uint32_t        msgNum;
    ECMDataset_t    *pECM;
    uint64_t        pulseCnt[NUM_PULSECOUNT];
} Emon32Dataset_t;

typedef struct __attribute__((__packed__)) {
    uint32_t    wattHour[NUM_CT];
    uint64_t    pulseCnt[NUM_PULSECOUNT];
} Emon32Cumulative_t;

typedef struct __attribute__((__packed__)) {
    uint8_t             valid;  /* Valid byte for wear levelling */
    Emon32Cumulative_t  report;
    uint16_t            crc;    /* CRC16-CCITT of data */
} Emon32CumulativeSave_t;

typedef struct __attribute__((__packed__)) {
    uint32_t    msg;
    int16_t     V[NUM_V];
    int16_t     P[NUM_CT];
    int32_t     E[NUM_CT];
    int16_t     T[TEMP_MAX_ONEWIRE];
    uint64_t    pulse[NUM_PULSECOUNT];
} PackedData_t;


/* EVTSRC_t contains all the event/interrupts sources. This value is shifted
 * to provide a vector of set events as bits.
 */
typedef enum {
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
    EVT_CONFIG_RESET    = 18u   /* A changed option requires a reset */
} EVTSRC_t;


/*! @brief Clear a pending event/interrupt flag after the task has been handled
 *  @param [in] Event source in enum
 */
void emon32EventClr(const EVTSRC_t evt);

/*! @brief Set the pending event/interrupt flag for tasks that are not handled
 *         within an ISR
 *  @param [in] evt : Event source in enum
 */
void emon32EventSet(const EVTSRC_t evt);

/*! @brief Output a string to the debug destination
 *  @param [in] s: pointer to null terminated string s
 */
void dbgPuts(const char *s);

