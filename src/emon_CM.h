#ifndef EMON_CM_H
#define EMON_CM_H

#include <stdint.h>

#include "board_def.h"

/* Number of samples available for power calculation. must be power of 2 */
#define PROC_DEPTH          4u

#define ZC_HYST             3u      /* Zero crossing hysteresis */
#define EQUIL_CYCLES        5u      /* Number of cycles to discard at POR */

/******************************************************************************
 * Type definitions
 *****************************************************************************/

typedef enum {
    ECM_INIT_SUCCESS,           /* Init was successful */
    ECM_INIT_FAIL_ENABLED,      /* Init failed as currently enabled */
    ECM_ENABLE_SUCCESS,
    ECM_ENABLE_FAIL_ENABLED,
    ECM_CYCLE_ONGOING,          /* A mains cycle is being accumulated */
    ECM_CYCLE_COMPLETE,         /* A full mains cycle has completed */
    ECM_REPORT_ONGOING,         /* A full set is accumulating */
    ECM_REPORT_COMPLETE         /* A full set to report is complete */
} ECM_STATUS_t;

/* Alias integer types for fixed point calculation */
typedef int16_t     q15_t;
typedef int32_t     q31_t;

/* SingleSampleSet_t contains a single set of V + CT ADC samples */
typedef struct __attribute__((__packed__)) SingleSampleSet {
    q15_t smp[VCT_TOTAL];
} SingleRawSampleSet_t;

/* SampleSetPacked_t contains a set of single sample sets. This allows the DMAC
 * to blit samples across multiple sample sets, depending on processing needs
 */
typedef struct __attribute__((__packed__)) SampleSetPacked {
    SingleRawSampleSet_t samples[SAMPLES_IN_SET];
} RawSampleSetPacked_t;

typedef struct RawSampleSetUnpacked {
    q15_t smp[NUM_V + NUM_CT];
} RawSampleSetUnpacked_t;

/* SampleSet_t contains an unpacked set of single sample sets */
typedef struct SampleSet {
    q15_t smpV[NUM_V];
    q15_t smpCT[NUM_CT];
} SampleSet_t;

typedef struct {
    q15_t   phaseX;
    q15_t   phaseY;
    float   ctCal;
} CTCfgUnpacked_t;

typedef struct {
    uint16_t        reportCycles;
    CTCfgUnpacked_t ctCfg[NUM_CT];
    float           voltageCal[NUM_V];
} ECMCfg_t;

typedef enum {
    POL_POS,
    POL_NEG
} Polarity_t;

typedef struct {
    uint32_t    sumV_sqr;
    int32_t     sumV_deltas;
} VAccumulator_t;

typedef struct {
    uint32_t    sumPA;
    uint32_t    sumPB;
    uint32_t    sumI_sqr;
    int32_t     sumI_deltas;
} CTAccumulator_t;

typedef struct {
    VAccumulator_t     processV[NUM_V];
    CTAccumulator_t    processCT[NUM_CT];
    unsigned int       num_samples;
} Accumulator_t;

/* This struct matches emonLibCM's calculations */
typedef struct {
    int32_t powerNow;           /* Summed power in cycles */
    int32_t rmsCT;              /* Accumulated I_RMS */
} CycleCT_t;

typedef struct {
    uint32_t    cycleCount;
    int32_t     rmsV[NUM_V];    /* Accumulated V_RMS */
    CycleCT_t   valCT[NUM_CT];  /* Combined CT values */
} ECMCycle_t;

typedef struct {
    float       realPower;
    uint32_t    wattHour;
    float       residualEnergy; /* Energy held over to next set */
} DataCT_t;

typedef struct {
    float       rmsV[NUM_V];
    DataCT_t    CT[NUM_CT];
} ECMDataset_t;

/******************************************************************************
 * Function prototypes
 *****************************************************************************/

/*! @brief Get the pointer to the configuration struct
 *  @return : pointer to Emon CM configuration struct
 */
ECMCfg_t *ecmGetConfig();

/*! @brief Swaps the ADC data buffer pointers
 */
void ecmSwapDataBuffer();

/*! @brief Returns a pointer to the ADC data buffer
 */
volatile RawSampleSetPacked_t *ecmDataBuffer();

/*! @brief For testing only, this function halves the incoming data rate with
 *         optional low pass filtering.
 *  @param [in] pDst : pointer to the SampleSet struct
 */
void ecmFilterSample(SampleSet_t *pDst);

/*! @brief Injects a raw sample from the ADC into the accumulators.
 */
ECM_STATUS_t ecmInjectSample();

/*! @brief Processes a whole cycle
 */
ECM_STATUS_t ecmProcessCycle();

/*! @brief Processes a whole data set
 *  @param [out] pData : pointer to the processed data structure
 */
void ecmProcessSet(ECMDataset_t *pData);

#endif
