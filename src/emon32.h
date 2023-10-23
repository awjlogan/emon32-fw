#ifndef EMON32_H
#define EMON32_H

#include <stdint.h>
#include "emon_CM.h"

/* Firmware version */
#define VERSION_FW_MAJ      0u
#define VERSION_FW_MIN      1u

#define NODE_ID             17u
#define TX_BUFFER_W         64u

/* UI timing */
#define SW_TIME_RESET       2048u   /* time in ms to press switch to reset */

/* Configuration key - indicates that the configuration is the default or
 * has been retrieved from non-volatile storage */
#define CONFIG_NVM_KEY      0xca55e77eul

/* Voltage and CT setup.
 */
#define DELTA_WH_STORE      200u    /* Threshold to store to non-volatile */

/* Pulse count setup */
#define NUM_PULSECOUNT      1

/* Temperature sensor setup */
#define NUM_TEMP            0

/* Precalculate the size of the EEPROM storage required to capture cumulative
 * energy and pulse count values. 2 bytes for CRC, 1 for valid
 */
#define EEPROM_WL_SIZE_BLK  (NUM_CT * 4) + (NUM_PULSECOUNT * 4) + 2 + 1
#define EEPROM_WL_NUM_BLK   EEPROM_WL_SIZE / EEPROM_WL_SIZE_BLK

/* Uncomment to downsample the sample rate by low pass filter
 * Otherwise, the second sample from each set will be discarded
 */
#define DOWNSAMPLE_DSP
#define DOWNSAMPLE_TAPS     19u

/* Configurable options. All the structs are packed to allow simple write to
 * EEPROM as a contiguous set.
 */
typedef enum __attribute__ ((__packed__)) {
    DATATX_RFM69    = 0,
    DATATX_UART     = 1
} DataTx_t;

typedef struct __attribute__((__packed__)) {
    uint8_t         nodeID;         /* ID for report*/
    uint8_t         mainsFreq;      /* Mains frequency */
    uint16_t        reportCycles;   /* Cycle count before reporting */
    uint16_t        whDeltaStore;   /* Minimum energy delta to store */
    DataTx_t        dataTx;         /* Data transmission hardware type */
} BaseCfg_t;

typedef struct __attribute__((__packed__)) {
    uint8_t         period;
    uint8_t         edge;
} PulseCfgPacked_t;

typedef struct __attribute__((__packed__)) {
   float           voltageCal;      /* Conversion to real V value */
} VoltageCfg_t;

typedef struct __attribute__((__packed__)) {
    float           ctCal;          /* Conversion to real I value */
    q15_t           phaseX;         /* Phase calibrations for interpolation */
    q15_t           phaseY;
    uint8_t         vChan;
} CTCfg_t;

typedef struct __attribute__((__packed__)) {
    uint32_t            key;
    BaseCfg_t           baseCfg;
#if (NUM_PULSECOUNT > 0)
    PulseCfgPacked_t    pulseCfg[NUM_PULSECOUNT];
    uint8_t             pulseActive;
#endif
    VoltageCfg_t        voltageCfg[NUM_V];
    CTCfg_t             ctCfg[NUM_CT];
    uint32_t            ctActive;       /* Multihot active bits */
    uint16_t            crc16_ccitt;
} Emon32Config_t;

typedef struct {
    uint32_t        msgNum;
    ECMDataset_t    *pECM;
    #if (NUM_PULSECOUNT > 0)
    uint64_t        pulseCnt[NUM_PULSECOUNT];
    #endif
} Emon32Dataset_t;

typedef struct __attribute__((__packed__)) {
    uint32_t        wattHour[NUM_CT];
    #if (NUM_PULSECOUNT > 0)
    uint64_t        pulseCnt[NUM_PULSECOUNT];
    #endif
} Emon32Cumulative_t;

typedef struct __attribute__((__packed__)) {
    uint8_t             valid;  /* Valid byte for wear levelling */
    Emon32Cumulative_t  report;
    uint16_t            crc;    /* CRC16-CCITT of data */
} Emon32CumulativeSave_t;

typedef struct __attribute__((__packed__)) {
    uint32_t        msg;
    int16_t         V[NUM_V];
    int16_t         P[NUM_CT];
    int32_t         E[NUM_CT];
    #if (NUM_TEMP > 0)
    int16_t         T[NUM_TEMP];
    #endif
    #if (NUM_PULSECOUNT > 0)
    uint64_t        pulse[NUM_PULSECOUNT];
    #endif
} PackedData_t;

/* Contains the states that are available to emon32 */
typedef enum {
    EMON_STATE_IDLE,    /* Ready to start */
    EMON_STATE_ACTIVE,  /* Collecting data */
    EMON_STATE_ERROR,   /* An error has occured */
    EMON_STATE_CONFIG   /* If configuration state */
} EmonState_t;

/* EVTSRC_t contains all the event/interrupts sources. This value is shifted
 * to provide a vector of set events as bits.
 */
typedef enum {
    EVT_DMA             = 0u,
    EVT_SYSTICK_100Hz   = 1u,
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
    EVT_TEMP_READ       = 15u
} EVTSRC_t;

/*! @brief Get the default configuration values
 *  @param [out] pCfg : pointer to configuration struct
 */
void emon32DefaultConfiguration(Emon32Config_t *pCfg);

/*! @brief Set the pending event/interrupt flag for tasks that are not handled
 *         within an ISR
 *  @param [in] evt : Event source in enum
 */
void emon32SetEvent(EVTSRC_t evt);

/*! @brief Clear a pending event/interrupt flag after the task has been handled
 *  @param [in] Event source in enum
 */
void emon32ClrEvent(EVTSRC_t evt);

/*! @brief Set the state of the emon32 system
 *  @param [in] state : state to set
 */
void emon32StateSet(EmonState_t state);

/*! @brief Returns the state of the emon32 system
 */
EmonState_t emon32StateGet();

/*! @brief Output a string to the debug destination
 *  @param [in] s: pointer to null terminated string s
 */
void dbgPuts(const char *s);

#endif
