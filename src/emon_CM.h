#ifndef EMON_CM_H
#define EMON_CM_H

#include "board_def.h"
#include "emon32.h"

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

/* KEY:VALUE pair to match EmonESP and EmonTx3eInterfacer */
typedef struct {
    uint32_t    msgNum;
    float       rmsV[NUM_V];
    DataCT_t    CT[NUM_CT];
    #if (NUM_PULSECOUNT > 0)
    uint64_t    pulseCnt[NUM_PULSECOUNT];
    #endif
} ECMSet_t;

/******************************************************************************
 * Function prototypes
 *****************************************************************************/

/*! @brief Configure the power/energy accumulation system
 *  @param [in] pConfig : pointer to configuration
 */
void ecmInit(Emon32Config_t * const pConfig);

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
 *  @param [in] pSet : pointer to the processed data structure
 */
void ecmProcessSet(ECMSet_t *set);

#endif
