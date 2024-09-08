#include <stdbool.h>
#include <string.h>

#ifndef HOSTED

#include "printf.h"
#include "qfplib-m0-full.h"

#else

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "emonCM_test.h"

#endif /* HOSTED */

#include "emon_CM.h"
#include "emon_CM_coeffs.h"

/* Number of samples available for power calculation. Must be power of 2. */
#define PROC_DEPTH   32u /* REVISIT only need  to store V values */
#define ZC_HYST      2u  /* Zero crossing hysteresis */
#define EQUIL_CYCLES 8u  /* Number of cycles to discard at startup */

static const float twoPi = 6.2831853072f;

static bool channelActive[VCT_TOTAL];

/*************************************
 * Local typedefs
 *************************************/

/* @typedef
 * @brief This struct contains V and CT phase information
 */
typedef struct PhaseCal_ {
  int   low;       /* RESERVED */
  float phaseLow;  /* Phase error at and below low limit */
  int   high;      /* RESERVED */
  float phaseHigh; /* RESERVED */
} PhaseCal_t;

typedef struct PhaseData_ {
  int   positionOfV;         /* Voltage index */
  float phaseErrorV;         /* Voltage phase error (degrees) */
  int   positionOfC;         /* CT index */
  float phaseErrorC;         /* CT phase error (degrees) */
  int   relativeCSample;     /* Position of current relative to this */
  int   relativeLastVSample; /* Position of previous V */
  int   relativeThisVSample; /* Position of next V */
  float x;                   /* Coefficient for interpolation */
  float y;                   /* Coefficient for interpolation */
} PhaseData_t;

typedef struct VoltageChan_ {
  int        inPin;
  float      voltageCal; /* Percentage deviation from ideal, 100.0 -> ideal */
  PhaseCal_t phaseCal;
  int        scanPos;
  float      Vrms;
  bool       inUse;
  bool       acPresent;
} VoltageChan_t;

typedef struct CurrentChan_ {
  int        inPin;
  float      ctCal; /* Nominal 100 A CT @ 333 mV output */
  PhaseCal_t phaseCal;
  int        scanPos;
  float      Irms;
  bool       inUse;
} CurrentChan_t;

typedef struct PowerInput_ {
  CurrentChan_t *cIn;            /* Input no. of current input from API */
  VoltageChan_t *vIn1;           /* Input no. of voltage input 1 */
  VoltageChan_t *vIn2;           /* Input no. of voltage input 2 */
  float          residualEnergy; /* left over from value reported */
  uint64_t       sumEnergy;      /* accumulated so far (Wh) */
  PhaseData_t    phaseDataV1;    /* phase data for V1 */
  PhaseData_t    phaseDataV2;    /* phase data for V2 */
  int32_t        sumPA1;         /* 'Partial powers' line-neutral loads */
  int32_t        sumPB1;
  int32_t        sumPA2; /* Second 'Partial powers' line-line loads */
  int32_t        sumPB2;
  int            realPower;
  int            apparentPower;
  int32_t        wh;
  float          pf;
  bool           inUse;
} PowerInput_t;

VoltageChan_t vInput[NUM_V];
CurrentChan_t cInput[NUM_CT];
PowerInput_t  pInput[NUM_CT];

typedef enum Polarity_ { POL_POS, POL_NEG } Polarity_t;

typedef struct VAccumulator_ {
  int64_t sumV_sqr;
  int     sumV_deltas;
} VAccumulator_t;

typedef struct CTAccumulator_ {
  int64_t sumPA;
  int64_t sumPB;
  int64_t sumPA1;
  int64_t sumPB1;
  int64_t sumI_sqr;
  int     sumI_deltas;
} CTAccumulator_t;

typedef struct Accumulator_ {
  VAccumulator_t  processV[NUM_V];
  CTAccumulator_t processCT[NUM_CT];
  int             numSamples;
  int             cycles;
  uint32_t        tStart_us;
  uint32_t        tDelta_us;
} Accumulator_t;

typedef struct CalcRMS_ {
  float   cal;
  int64_t sSqr;
  int     sDelta;
  int     numSamples;
} CalcRMS_t;

typedef struct PhaseXY_ {
  float phaseX;
  float phaseY;
} PhaseXY_t;

typedef struct RawSampleSetUnpacked {
  q15_t smp[VCT_TOTAL];
} RawSampleSetUnpacked_t;

/*************************************
 * Function prototypes
 *************************************/

static inline q15_t __STRUNCATE(int32_t val) RAMFUNC;
static void         accumSwapClear(void);
static float        calcRMS(CalcRMS_t *pSrc) RAMFUNC;
static float        calibrationAmplitude(float cal, float fixed);
static void         calibrationPhase(PhaseXY_t *pPh, float phase, int idxCT);
static void         configChannelV(int ch);
static void         configChannelCT(int ch);
static void         swapPtr(void **pIn1, void **pIn2);
static bool         zeroCrossingSW(q15_t smpV) RAMFUNC;

/******** FIXED POINT MATHS FUNCTIONS ********
 *
 * Adapted from Arm CMSIS-DSP: https://github.com/ARM-software/CMSIS-DSP
 */

/*! @brief Truncate Q31 to a Q15 fixed point, round to nearest LSB
 *  @param [in] val value to truncate
 *  @return Q15 truncated val
 */
static RAMFUNC inline q15_t __STRUNCATE(int32_t val) {
  int roundUp = 0;
  if (0 != (val & (1u << 14))) {
    roundUp = 1;
  }
  return (q15_t)((val >> 15) + roundUp);
}

/***** END FIXED POINT FUNCIONS *****/

static RAMFUNC float calcRMS(CalcRMS_t *pSrc) {
  int64_t numSamplesSqr = (int64_t)pSrc->numSamples * pSrc->numSamples;
  float   vcal          = pSrc->cal;

  int64_t deltasSqr = pSrc->sDelta * pSrc->sDelta;

  float offsetCorr =
      qfp_fdiv(qfp_int642float(deltasSqr), qfp_int642float(numSamplesSqr));

  float rms =
      qfp_fdiv(qfp_int642float(pSrc->sSqr), qfp_int2float(pSrc->numSamples));

  rms = qfp_fsub(rms, offsetCorr);
  rms = qfp_fsqrt(rms);
  rms = qfp_fmul(vcal, rms);

  return rms;
}

/*! @brief Swap pointers to buffers
 */
static void swapPtr(void **pIn1, void **pIn2) {
  void *tmp = *pIn1;
  *pIn1     = *pIn2;
  *pIn2     = tmp;
}

/******************************************************************************
 * Configuration
 *****************************************************************************/

static ECMCfg_t ecmCfg         = {0};
static bool     processTrigger = false;
static int      discardCycles  = EQUIL_CYCLES;
static bool     initDone       = true;
static int      samplePeriodus;
static float    sampleIntervalRad;

ECMCfg_t *ecmConfigGet(void) { return &ecmCfg; }

void ecmConfigChannel(int ch) {
  if (ch < NUM_V) {
    configChannelV(ch);
  } else {
    configChannelCT(ch);
  }
}

void configChannelCT(int ch) {
  const float iCal          = 6.0f; // Port from emonPi2/Tx4 for 333 mV CT
  channelActive[ch + NUM_V] = ecmCfg.ctCfg[ch].active;
  ecmCfg.ctCfg[ch].ctCal =
      calibrationAmplitude(ecmCfg.ctCfg[ch].ctCalRaw, iCal);

  PhaseXY_t phaseXY;
  calibrationPhase(&phaseXY, ecmCfg.ctCfg[ch].phCal, ch);
  ecmCfg.ctCfg[ch].phaseX = phaseXY.phaseX;
  ecmCfg.ctCfg[ch].phaseY = phaseXY.phaseY;
}

void configChannelV(int ch) {
  const float vsCal = 16.0174f; // Port from emonPi2/Tx4 * 2 for differential
  channelActive[ch] = ecmCfg.vCfg[ch].vActive;
  ecmCfg.vCfg[ch].voltageCal =
      calibrationAmplitude(ecmCfg.vCfg[ch].voltageCalRaw, vsCal);
}

void ecmConfigInit(void) {

  /* Calculate the angular sampling rate in degrees and radians. */
  float sampleIntervalDeg = qfp_fmul(360.0f, qfp_int2float(ecmCfg.mainsFreq));
  sampleIntervalDeg =
      qfp_fdiv(sampleIntervalDeg, qfp_int2float(ecmCfg.sampleRateHz));
  samplePeriodus = 1000000 / (ecmCfg.sampleRateHz * (VCT_TOTAL));

  /* This (should) be optimised at compile time */
  sampleIntervalRad = twoPi / 360.0f;
  sampleIntervalRad = qfp_fmul(sampleIntervalRad, sampleIntervalDeg);

  for (int i = 0; i < NUM_V; i++) {
    configChannelV(i);
  }

  for (int i = 0; i < NUM_CT; i++) {
    configChannelCT(i);
  }

  initDone = true;
}

void ecmConfigReportCycles(int reportCycles) {
  ecmCfg.reportCycles = reportCycles;
}

/******************************************************************************
 * Data acquisition
 *****************************************************************************/

static volatile RawSampleSetPacked_t adcSamples[SAMPLE_BUF_DEPTH];
static volatile RawSampleSetPacked_t *volatile adcActive = adcSamples;
static volatile RawSampleSetPacked_t *volatile adcProc   = adcSamples + 1;

void ecmDataBufferSwap(void) {
  swapPtr((void **)&adcActive, (void **)&adcProc);
}

volatile RawSampleSetPacked_t *ecmDataBuffer(void) { return adcActive; }

/******************************************************************************
 * Pre-processing
 *****************************************************************************/

static RawSampleSetUnpacked_t dspBuffer[DOWNSAMPLE_TAPS];
static SampleSet_t            sampleRingBuffer[PROC_DEPTH];

/******************************************************************************
 * Accumulators
 *****************************************************************************/

static Accumulator_t  accumBuffer[2];
static Accumulator_t *accumCollecting = accumBuffer;
static Accumulator_t *accumProcessing = accumBuffer + 1;

static ECMPerformance_t  perfCounter[2];
static ECMPerformance_t *perfActive = perfCounter;
static ECMPerformance_t *perfIdle   = perfCounter + 1;

/******************************************************************************
 * Functions
 *****************************************************************************/

static void accumSwapClear(void) {
  swapPtr((void **)&accumCollecting, (void **)&accumProcessing);
  memset((void *)accumCollecting, 0, sizeof(*accumCollecting));
}

/*! @brief Zero crossing detection, software
 *  @param [in] smpV : current voltage sample
 *  @return true for positive crossing, false otherwise
 */
RAMFUNC bool zeroCrossingSW(q15_t smpV) {
  Polarity_t        polarityNow;
  static Polarity_t polarityLast = POL_POS;
  static int        hystCnt      = ZC_HYST;

  polarityNow = (smpV < 0) ? POL_NEG : POL_POS;

  if (polarityNow != polarityLast) {
    hystCnt--;
    if (0 == hystCnt) {
      hystCnt      = ZC_HYST;
      polarityLast = polarityNow;
      if (POL_POS == polarityNow) {
        return true;
      }
    }
  }
  return false;
}

/*! @brief Turn an amplitude calibration value into a factor to change the
 *         abstract value into the real value, accounting for ADC width.
 *  @param [in] cal : the calibration value
 *  @param [in] fixed : the hardware specific calibration value
 *  @return : the scaled calibration value
 */
static float calibrationAmplitude(float cal, float fixed) {
  const int   adcWidth = ADC_RES_BITS;
  const float vRef     = ADC_VREF;

  float cmCal = qfp_fmul(cal, vRef);
  cmCal       = qfp_fmul(cmCal, fixed);
  cmCal       = qfp_fdiv(cmCal, qfp_uint2float(1 << adcWidth));

  return cmCal;
}

/*! @brief Decompose a floating point CT phase into an X/Y pair for
 *         interpolation.
 *  @param [in] phase : CT lead phase in degrees.
 *  @param [in] idxCT : physical index (0-based) of the CT
 *  @return : structure with the X/Y fixed point coefficients.
 */
static void calibrationPhase(PhaseXY_t *pPh, float phase, int idxCT) {

  float phaseShift = qfp_fdiv(phase, 360.0f);
  int   phCorr_i   = (idxCT + NUM_V) * ecmCfg.mainsFreq * samplePeriodus;
  float phCorr_f   = qfp_fdiv(qfp_int2float(phCorr_i), 1000000.0f);
  phaseShift       = qfp_fadd(phaseShift, phCorr_f);
  phaseShift       = qfp_fmul(phaseShift, twoPi);

  pPh->phaseY = qfp_fdiv(qfp_fsin(phaseShift), qfp_fsin(sampleIntervalRad));

  pPh->phaseX = qfp_fsub(qfp_fcos(phaseShift),
                         (qfp_fmul(pPh->phaseY, qfp_fcos(sampleIntervalRad))));
}

float ecmPhaseCalibrate(unsigned int idx) {
  /* REVISIT Automatic phase calibration */
  (void)idx;
  return 0.0f;
}

void ecmFlush(void) {
  discardCycles = EQUIL_CYCLES;

  memset(accumBuffer, 0, (2 * sizeof(*accumBuffer)));
  memset(sampleRingBuffer, 0, (PROC_DEPTH * sizeof(*sampleRingBuffer)));
  memset(dspBuffer, 0, (DOWNSAMPLE_TAPS * sizeof(*dspBuffer)));
}

RAMFUNC void ecmFilterSample(SampleSet_t *pDst) {
  if (!ecmCfg.downsample) {
    /* No filtering, discard the second sample in the set */
    for (unsigned int idxV = 0; idxV < NUM_V; idxV++) {
      pDst->smpV[idxV] = adcProc->samples[0].smp[idxV];
    }

    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++) {
      pDst->smpCT[idxCT] = adcProc->samples[0].smp[idxCT + NUM_V];
    }
  } else {
    /* The FIR half band filter is symmetric, so the coefficients are folded.
     * Alternating coefficients are 0, so are not included in any outputs.
     * For an ODD number of taps, the centre coefficent is handled
     * individually, then the other taps in a loop.
     *
     * b_0 | b_2 | .. | b_X | .. | b_2 | b_0
     *
     * For an EVEN number of taps, loop across all the coefficients:
     *
     * b_0 | b_2 | .. | b_2 | b_0
     */
    static unsigned int idxInj            = 0;
    int32_t             intRes[VCT_TOTAL] = {0};

    const unsigned int downsampleTaps = DOWNSAMPLE_TAPS;
    const unsigned int idxInjPrev =
        (0 == idxInj) ? (downsampleTaps - 1u) : (idxInj - 1u);

    /* Copy the packed raw ADC value into the unpacked buffer; index 1 is the
     * most recent sample.
     */
    for (unsigned int idxSmp = 0; idxSmp < VCT_TOTAL; idxSmp++) {
      dspBuffer[idxInjPrev].smp[idxSmp] = adcProc->samples[0].smp[idxSmp];
      dspBuffer[idxInj].smp[idxSmp]     = adcProc->samples[1].smp[idxSmp];
    }

    /* For an ODD number of taps, take the unique middle value to start. As
     * the filter is symmetric, this is the final element in the array.
     */
    const q15_t  coeffMid = firCoeffs[numCoeffUnique - 1u];
    unsigned int idxMid   = idxInj + (downsampleTaps / 2) + 1u;
    if (idxMid >= downsampleTaps)
      idxMid -= downsampleTaps;

    for (unsigned int idxChannel = 0; idxChannel < VCT_TOTAL; idxChannel++) {
      intRes[idxChannel] += coeffMid * dspBuffer[idxMid].smp[idxChannel];
    }

    /* Loop over the FIR coefficients, sub loop through channels. The filter
     * is folded so the symmetric FIR coefficients are used for both samples.
     */
    unsigned int idxSmpStart = idxInj;
    unsigned int idxSmpEnd =
        ((downsampleTaps - 1u) == idxInj) ? 0 : (idxInj + 1u);
    if (idxSmpEnd >= downsampleTaps)
      idxSmpEnd -= downsampleTaps;

    for (unsigned int idxCoeff = 0; idxCoeff < (numCoeffUnique - 1u);
         idxCoeff++) {
      const q15_t coeff = firCoeffs[idxCoeff];
      for (unsigned int idxChannel = 0; idxChannel < VCT_TOTAL; idxChannel++) {
        intRes[idxChannel] += coeff * (dspBuffer[idxSmpStart].smp[idxChannel] +
                                       dspBuffer[idxSmpEnd].smp[idxChannel]);
      }

      /* Converge toward the middle, check for over/underflow */
      idxSmpStart -= 2u;
      if (idxSmpStart > downsampleTaps)
        idxSmpStart += downsampleTaps;

      idxSmpEnd += 2u;
      if (idxSmpEnd >= downsampleTaps)
        idxSmpEnd -= downsampleTaps;
    }

    /* Truncate with rounding to nearest LSB and place into field. 0 if
     * the channel is not in use.
     */
    for (unsigned int idxChannel = 0; idxChannel < VCT_TOTAL; idxChannel++) {
      const q15_t resTrunc =
          channelActive[idxChannel] ? __STRUNCATE(intRes[idxChannel]) : 0;

      if (idxChannel < NUM_V) {
        pDst->smpV[idxChannel] = resTrunc;
      } else {
        pDst->smpCT[idxChannel - NUM_V] = resTrunc;
      }
    }

    /* Each injection is 2 samples */
    idxInj += 2u;
    if (idxInj > (downsampleTaps - 1)) {
      idxInj -= (downsampleTaps);
    }
  }
}

RAMFUNC ECM_STATUS_t ecmInjectSample(void) {
  bool       reportReady = false;
  bool       zerox_flag  = false;
  bool       pend_1s     = false;
  static int idxInject;
  uint32_t   t_start = 0;

  if (0 != ecmCfg.timeMicros) {
    t_start = (*ecmCfg.timeMicros)();
  }

  SampleSet_t smpProc;

  /* Copy the pre-processed sample data into the ring buffer */
  ecmFilterSample(&smpProc);
  memcpy((void *)(sampleRingBuffer + idxInject), (const void *)&smpProc,
         sizeof(*sampleRingBuffer));

  accumCollecting->numSamples++;

  const unsigned int idxLast = (idxInject - 1u) & (PROC_DEPTH - 1u);

  for (unsigned int idxV = 0; idxV < NUM_V; idxV++) {
    if (channelActive[idxV]) {
      int64_t V = sampleRingBuffer[idxInject].smpV[idxV];
      accumCollecting->processV[idxV].sumV_sqr += V * V;
      accumCollecting->processV[idxV].sumV_deltas += V;
    }
  }

  for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++) {
    if (channelActive[idxCT + NUM_V]) {
      int64_t thisV =
          sampleRingBuffer[idxInject].smpV[ecmCfg.ctCfg[idxCT].vChan];
      int64_t lastV = sampleRingBuffer[idxLast].smpV[ecmCfg.ctCfg[idxCT].vChan];
      int64_t thisCT = sampleRingBuffer[idxInject].smpCT[idxCT];

      accumCollecting->processCT[idxCT].sumPA += thisCT * lastV;
      accumCollecting->processCT[idxCT].sumPB += thisCT * thisV;
      accumCollecting->processCT[idxCT].sumI_sqr += thisCT * thisCT;
      accumCollecting->processCT[idxCT].sumI_deltas += thisCT;
    }
  }

  /* Flag if there has been a (-) -> (+) crossing, always on V1. Check for
   * zero-crossing, swap buffers and pend event.
   */
  zerox_flag = ecmCfg.zx_hw_stat
                   ? (*ecmCfg.zx_hw_stat)()
                   : zeroCrossingSW(sampleRingBuffer[idxInject].smpV[0]);

  /* Check for zero crossing, swap buffers and pend event */
  if (zerox_flag) {
    if (ecmCfg.zx_hw_clr)
      (*ecmCfg.zx_hw_clr)();

    if (0 == discardCycles) {
      accumCollecting->cycles++;
    } else {
      discardCycles--;
      if (0 == discardCycles)
        accumSwapClear();
      if (0 != ecmCfg.timeMicros) {
        accumCollecting->tStart_us = (*ecmCfg.timeMicros)();
      }
    }

    /* Flag one second before the report is due to allow slow sensors to sample.
     * For example, DS18B20 requires 750 ms to sample.
     */
    if (accumCollecting->cycles == (ecmCfg.reportCycles - ecmCfg.mainsFreq)) {
      pend_1s = true;
    }

    /* All samples for this set are complete, or there has been a request from
     * software to read out the sample.
     */
    if ((accumCollecting->cycles >= ecmCfg.reportCycles) || processTrigger) {
      accumSwapClear();

      if ((0 != ecmCfg.timeMicros) && (0 != ecmCfg.timeMicrosDelta)) {
        accumCollecting->tStart_us = (*ecmCfg.timeMicros)();
        accumProcessing->tDelta_us =
            (*ecmCfg.timeMicrosDelta)(accumProcessing->tStart_us);
      } else {
        accumCollecting->tStart_us = 0;
        accumProcessing->tDelta_us = 0;
      }

      processTrigger = false;
      reportReady    = true;
    }
  }

  if (0 != ecmCfg.timeMicrosDelta) {
    perfActive->numSlices++;
    perfActive->microsSlices += (*ecmCfg.timeMicrosDelta)(t_start);
  }

  /* Advance injection point, masking for overflow */
  idxInject = (idxInject + 1u) & (PROC_DEPTH - 1u);

  return reportReady ? ECM_REPORT_COMPLETE
                     : (pend_1s ? ECM_PEND_1S : ECM_CYCLE_ONGOING);
}

ECMPerformance_t *ecmPerformance(void) {
  swapPtr((void **)&perfActive, (void **)&perfIdle);
  memset(perfActive, 0, sizeof(*perfActive));

  return perfIdle;
}

RAMFUNC void ecmProcessSet(ECMDataset_t *pData) {
  uint32_t  t_start = 0;
  CalcRMS_t rms;

  if (0 != ecmCfg.timeMicros) {
    t_start = (*ecmCfg.timeMicros)();
  }

  /* Reused constants */
  const int     numSamples    = accumProcessing->numSamples;
  const int64_t numSamplesSqr = numSamples * numSamples;
  rms.numSamples              = numSamples;

  /* Use the actual count period (in us) to account for rounding */
  const int   cntPer    = F_TIMER_ADC / SAMPLE_RATE / VCT_TOTAL;
  const int   usForSet  = (cntPer * VCT_TOTAL * SAMPLES_IN_SET) * numSamples;
  const float timeTotal = qfp_fdiv(qfp_int2float(usForSet), 1000000.0f);
  pData->calcTime       = timeTotal;
  pData->wallTime =
      qfp_fdiv(qfp_uint2float(accumProcessing->tDelta_us), 1000000.0f);

  for (int idxV = 0; idxV < NUM_V; idxV++) {
    if (channelActive[idxV]) {
      rms.cal           = ecmCfg.vCfg[idxV].voltageCal;
      rms.sDelta        = accumProcessing->processV[idxV].sumV_deltas;
      rms.sSqr          = accumProcessing->processV[idxV].sumV_sqr;
      pData->rmsV[idxV] = calcRMS(&rms);
    } else {
      pData->rmsV[idxV] = 0.0f;
    }
  }

  for (int idxCT = 0; idxCT < NUM_CT; idxCT++) {
    if (channelActive[idxCT + NUM_V]) {
      int idxV = ecmCfg.ctCfg[idxCT].vChan;

      // RMS Current
      rms.cal               = ecmCfg.ctCfg[idxCT].ctCal;
      rms.sDelta            = accumProcessing->processCT[idxCT].sumI_deltas;
      rms.sSqr              = accumProcessing->processCT[idxCT].sumI_sqr;
      pData->CT[idxCT].rmsI = calcRMS(&rms);

      // Power and energy
      float sumEnergy = qfp_fadd(
          (qfp_fmul(qfp_int642float(accumProcessing->processCT[idxCT].sumPA),
                    ecmCfg.ctCfg[idxCT].phaseX)),
          (qfp_fmul(qfp_int642float(accumProcessing->processCT[idxCT].sumPB),
                    ecmCfg.ctCfg[idxCT].phaseY)));

      int vi_offset = rms.sDelta * accumProcessing->processV[idxV].sumV_deltas;

      float powerNow = qfp_fdiv(sumEnergy, qfp_int2float(numSamples));
      powerNow       = qfp_fsub(powerNow, qfp_fdiv(qfp_int2float(vi_offset),
                                                   qfp_int642float(numSamplesSqr)));
      powerNow =
          qfp_fmul(powerNow, qfp_fmul(rms.cal, ecmCfg.vCfg[idxV].voltageCal));

      // Power factor
      float rmsV          = pData->rmsV[idxV];
      float VA            = qfp_fmul(pData->CT[idxCT].rmsI, rmsV);
      float pf            = qfp_fdiv(powerNow, VA);
      bool  pf_b          = ((pf > 1.05f) || (pf < -1.05f) || (pf != pf));
      pData->CT[idxCT].pf = pf_b ? 0.0f : pf;

      // Energy and power, rounding to nearest integer
      pData->CT[idxCT].realPower     = qfp_float2int(qfp_fadd(powerNow, 0.5f));
      pData->CT[idxCT].apparentPower = qfp_float2int(qfp_fadd(VA, 0.5f));

      // REVISIT : Consider double precision here, some truncation observed
      float energyNow = qfp_fmul(powerNow, timeTotal);
      energyNow       = qfp_fadd(energyNow, pData->CT[idxCT].residualEnergy);
      int whNow       = qfp_float2int(qfp_fdiv(energyNow, 3600.0f));

      pData->CT[idxCT].wattHour += whNow;
      pData->CT[idxCT].residualEnergy =
          qfp_fsub(energyNow, qfp_int2float(whNow * 3600));
    } else {
      /* Zero all values otherwise */
      memset(&pData->CT[idxCT], 0, sizeof(*pData->CT));
    }
  }

  if (0 != ecmCfg.timeMicrosDelta) {
    perfActive->numCycles++;
    perfActive->microsCycles += (*ecmCfg.timeMicrosDelta)(t_start);
  }
}

void ecmProcessSetTrigger(void) { processTrigger = true; }
