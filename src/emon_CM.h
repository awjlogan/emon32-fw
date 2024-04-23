#pragma once

#include <stdint.h>

#include "board_def.h"

#ifndef RAMFUNC
#define RAMFUNC
#endif

/******************************************************************************
 * Type definitions
 *****************************************************************************/

#define DOWNSAMPLE_TAPS     19u

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
typedef int64_t     q63_t;

/* SingleSampleSet_t contains a single set of V + CT ADC samples */
typedef struct __attribute__((__packed__)) SingleSampleSet_ {
    q15_t smp[VCT_TOTAL];
} SingleRawSampleSet_t;

/* RawSampleSetPacked_t contains a set of single sample sets. This allows the
 * DMAC to blit samples across multiple sample sets, depending on processing
 * needs.
 */
typedef struct __attribute__((__packed__)) SampleSetPacked_ {
    SingleRawSampleSet_t samples[SAMPLES_IN_SET];
} RawSampleSetPacked_t;

typedef struct RawSampleSetUnpacked {
    q15_t smp[VCT_TOTAL];
} RawSampleSetUnpacked_t;

/* SampleSet_t contains an unpacked set of single sample sets */
typedef struct SampleSet_ {
    q15_t smpV[NUM_V];
    q15_t smpCT[NUM_CT];
} SampleSet_t;

typedef struct CTCfgUnpacked_ {
    q15_t           phaseX;
    q15_t           phaseY;
    float           ctCal;
    unsigned int    active;
    unsigned int    vChan;
} CTCfgUnpacked_t;

typedef struct ECMCfg_ {
    unsigned int    downsample;                     /* DSP enabled */
    int             (*zx_hw_stat)(void);            /* HW zero crossing status function */
    void            (*zx_hw_clr)(void);             /* HW zero crossing clear function */
    uint32_t        (*timeMicros)(void);            /* Time in microseconds now */
    uint32_t        (*timeMicrosDelta)(uint32_t);   /* Time delta in microseconds */
    unsigned int    reportCycles;                   /* Number of cycles before reporting */
    CTCfgUnpacked_t ctCfg[NUM_CT];                  /* CT Configuration */
    float           voltageCal[NUM_V];              /* Voltage calibration */
} ECMCfg_t;

typedef enum Polarity_ {
    POL_POS,
    POL_NEG
} Polarity_t;

typedef struct VAccumulator_ {
    uint32_t    sumV_sqr;
    int32_t     sumV_deltas;
} VAccumulator_t;

typedef struct CTAccumulator_ {
    uint32_t    sumPA;
    uint32_t    sumPB;
    uint32_t    sumI_sqr;
    int32_t     sumI_deltas;
} CTAccumulator_t;

typedef struct Accumulator_ {
    VAccumulator_t  processV[NUM_V];
    CTAccumulator_t processCT[NUM_CT];
    unsigned int    numSamples;
} Accumulator_t;

/* This struct matches emonLibCM's calculations */
typedef struct CycleCT_ {
    int32_t powerNow;           /* Summed power in cycles */
    float   rmsCT;              /* Accumulated I_RMS */
} CycleCT_t;

typedef struct ECMCycle_ {
    uint32_t    cycleCount;
    float       rmsV[NUM_V];    /* Accumulated V_RMS */
    CycleCT_t   valCT[NUM_CT];  /* Combined CT values */
} ECMCycle_t;

typedef struct DataCT_ {
    float       realPower;
    uint32_t    wattHour;
    float       residualEnergy; /* Energy held over to next set */
} DataCT_t;

typedef struct ECMDataset_ {
    float       rmsV[NUM_V];
    DataCT_t    CT[NUM_CT];
} ECMDataset_t;

typedef struct PhaseXY_ {
    q15_t phaseX;
    q15_t phaseY;
} PhaseXY_t;

typedef struct ECMPerformance_ {
    int numSlices;
    int microsSlices;
    int numCycles;
    int microsCycles;
    int numDatasets;
    int microsDatasets;
} ECMPerformance_t;

/******************************************************************************
 * Function prototypes
 *****************************************************************************/

/*! @brief Turn an amplitude calibration value into a factor to change the
 *         abstract value into the real value, accounting for ADC width.
 *  @param [in] cal : the calibration value
 *  @return : the scaled calibration value
 */
float ecmCalibrationCalculate(float cal);

/*! @brief Returns a pointer to the ADC data buffer
 *  @return : pointer to the active ADC data buffer.
 */
volatile RawSampleSetPacked_t *ecmDataBuffer(void);

/*! @brief Swap the data sampling buffers. ADC will be filling the other
 *         while it is handled.
 */
void ecmDataBufferSwap(void);

/*! @brief Unpack and optionally low pass filter the raw sample
 *         The struct from the DMA has no partition into V/CT channels, so
 *         alter this function to move data from the implementation specific
 *         DMA addressess to the defined SampleSet_t fields
 *  @param [out] pDst : pointer to the SampleSet_t destination
 */
void ecmFilterSample(SampleSet_t *pDst) RAMFUNC;

/*! @brief Flush all data and reset the equilibration cycle count */
void ecmFlush(void);

/*! @brief Get the pointer to the configuration struct
 *  @return : pointer to Emon CM configuration struct
 */
ECMCfg_t *ecmGetConfig(void);

/*! @brief Injects a raw sample from the ADC into the accumulators.
 */
ECM_STATUS_t ecmInjectSample(void) RAMFUNC;

/*! @brief Gets the performance counter
 *  @return : pointer to the performance counter
 */
ECMPerformance_t *ecmPerformance(void);

/*! @brief Decompose a floating point CT phase into an X/Y pair for
 *         interpolation.
 *  @param [in] phase : CT lead phase in degrees.
 *  @return : structure with the X/Y fixed point coefficients.
 */
PhaseXY_t ecmPhaseCalculate(float phase);

/*! @brief Calibrate a CT sensor's lead against the input voltage
 *  @param [in] idx : CT index
 *  @return : phase lead in degrees
 */
float ecmPhaseCalibrate(unsigned int idx);

/*! @brief Processes a whole cycle
 */
ECM_STATUS_t ecmProcessCycle(void) RAMFUNC;

/*! @brief Processes a whole data set
 *  @param [out] pData : pointer to the processed data structure
 */
void ecmProcessSet(ECMDataset_t *pData) RAMFUNC;

/*! @brief Force trigger data set processing on next cycle complete */
void ecmProcessSetTrigger(void);
