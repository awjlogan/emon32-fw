#include <string.h>

#ifndef HOSTED

#include "qfplib-m0-full.h"

#else

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "emonCM_test.h"

#endif /* HOSTED */

#include "emon_CM.h"
#include "emon_CM_coeffs.h"

/* Number of samples available for power calculation. must be power of 2 */
#define PROC_DEPTH      32u     /* REVISIT only need  to store V values */
#define ZC_HYST         3u      /* Zero crossing hysteresis */
#define EQUIL_CYCLES    5u      /* Number of cycles to discard at POR */

const float twoPi = 6.2831853072f;

/*************************************
 * Function prototypes
 *************************************/

static inline q15_t     __STRUNCATE     (int32_t val)   RAMFUNC;
static void             ecmSwapPtr      (void **pIn1, void **pIn2);
static int              zeroCrossingSW  (q15_t smpV)    RAMFUNC;

/******** FIXED POINT MATHS FUNCTIONS ********
 *
 * Adapted from Arm CMSIS-DSP: https://github.com/ARM-software/CMSIS-DSP
 */

/*! @brief Truncate Q31 to a Q15 fixed point, round to nearest LSB
 *  @param [in] val value to truncate
 *  @return Q15 truncated val
 */
static RAMFUNC inline q15_t
__STRUNCATE(int32_t val)
{
    unsigned int roundUp = 0;
    if (0 != (val & (1u << 14)))
    {
        roundUp = 1u;
    }
    return (q15_t) ((val >> 15) + roundUp);
}


/***** END FIXED POINT FUNCIONS *****/

/*! @brief Swap pointers to buffers
 */
static void
ecmSwapPtr(void **pIn1, void **pIn2)
{
    void *tmp = *pIn1;
    *pIn1 = *pIn2;
    *pIn2 = tmp;
}

/******************************************************************************
 * Configuration
 *****************************************************************************/

static ECMCfg_t ecmCfg;
static int      processTrigger = 0;
static int      discardCycles = EQUIL_CYCLES;
static int      initDone = 0;
static int      samplePeriodus;
static float    sampleIntervalRad;


ECMCfg_t *
ecmConfigGet(void)
{
    return &ecmCfg;
}


void
ecmConfigInit(void)
{
    /* Calculate the angular sampling rate in degrees and radians. */
    float sampleIntervalDeg = qfp_fmul(360.0f, qfp_int2float(ecmCfg.mainsFreq));
    sampleIntervalDeg = qfp_fdiv(sampleIntervalDeg, qfp_int2float(ecmCfg.sampleRateHz));
    samplePeriodus = 1000000 / (ecmCfg.sampleRateHz * (VCT_TOTAL));

    /* This (should) be optimised at compile time */
    sampleIntervalRad = twoPi / 360.0f;
    sampleIntervalRad = qfp_fmul(sampleIntervalRad, sampleIntervalDeg);

    initDone = 1;
}

/******************************************************************************
 * Data acquisition
 *****************************************************************************/

static volatile RawSampleSetPacked_t adcSamples[SAMPLE_BUF_DEPTH];
static volatile RawSampleSetPacked_t *volatile adcActive    = adcSamples;
static volatile RawSampleSetPacked_t *volatile adcProc      = adcSamples + 1;


void
ecmDataBufferSwap(void)
{
    ecmSwapPtr((void **)&adcActive, (void **)&adcProc);
}


volatile RawSampleSetPacked_t *
ecmDataBuffer(void)
{
    return adcActive;
}

/******************************************************************************
 * Pre-processing
 *****************************************************************************/

static RawSampleSetUnpacked_t   dspBuffer[DOWNSAMPLE_TAPS];
static SampleSet_t              sampleRingBuffer[PROC_DEPTH];

/******************************************************************************
 * Accumulators
 *****************************************************************************/

static Accumulator_t    accumBuffer[2];
static Accumulator_t    *accumCollecting = accumBuffer;
static Accumulator_t    *accumProcessing = accumBuffer + 1;
static ECMCycle_t       ecmCycle;
static ECMPerformance_t perfCounter[2];
static ECMPerformance_t *perfActive = perfCounter;
static ECMPerformance_t *perfIdle   = perfCounter + 1;

/******************************************************************************
 * Functions
 *****************************************************************************/

/*! @brief Zero crossing detection, software
 *  @param [in] smpV : current voltage sample
 *  @return 1 for positive crossing, 0 otherwise
 */
RAMFUNC static int
zeroCrossingSW(q15_t smpV)
{
    Polarity_t          polarityNow;
    static Polarity_t   polarityLast = POL_POS;
    static int          hystCnt = ZC_HYST;

    polarityNow = (smpV < 0) ? POL_NEG : POL_POS;

    if (polarityNow != polarityLast)
    {
        hystCnt--;
        if (0 == hystCnt)
        {
            hystCnt = ZC_HYST;
            polarityLast = polarityNow;
            if (POL_POS == polarityNow)
            {
                return 1;
            }
        }
    }
    return 0;
}


float
ecmCalibrationCalculate(float cal)
{
    const unsigned int adcWidth = 12u;
    return qfp_fdiv(cal, qfp_uint2float(1 << (adcWidth - 1u)));
}


PhaseXY_t
ecmPhaseCalculate(float phase, int idxCT)
{
    PhaseXY_t phaseXY;

    /* REVISIT : parts of the correction can be made constant for each CT */
    float phaseShift = qfp_fdiv(phase, 360.0f);
    int phCorr_i = (idxCT + NUM_V) * ecmCfg.mainsFreq * samplePeriodus;
    float phCorr_f = qfp_fdiv(qfp_int2float(phCorr_i),
                              1000000.0f);
    phaseShift = qfp_fadd(phaseShift, phCorr_f);
    phaseShift = qfp_fmul(phaseShift, twoPi);

    phaseXY.phaseY = qfp_fdiv(qfp_fsin(phaseShift),
                              qfp_fsin(sampleIntervalRad));

    phaseXY.phaseX = qfp_fsub(qfp_fcos(phaseShift),
                              (qfp_fmul(phaseXY.phaseY,
                                        qfp_fcos(sampleIntervalRad))));
    return phaseXY;
}


float
ecmPhaseCalibrate(unsigned int idx)
{
    /* REVISIT Automatic phase calibration */
    (void)idx;
    return 0.0f;
}


void
ecmFlush(void)
{
    discardCycles = EQUIL_CYCLES;

    memset(accumBuffer,         0, (2 * sizeof(Accumulator_t)));
    memset(sampleRingBuffer,    0, (PROC_DEPTH * sizeof(SampleSet_t)));
    memset(dspBuffer,           0, (DOWNSAMPLE_TAPS * sizeof(RawSampleSetUnpacked_t)));
}


RAMFUNC void
ecmFilterSample(SampleSet_t *pDst)
{
    if (0 == ecmCfg.downsample)
    {
        /* No filtering, discard the second sample in the set */
        for (unsigned int idxV = 0; idxV < NUM_V; idxV++)
        {
            pDst->smpV[idxV] = adcProc->samples[0].smp[idxV];
        }

        for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
        {
            pDst->smpCT[idxCT] = adcProc->samples[0].smp[idxCT + NUM_V];
        }
    }
    else
    {
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
        static unsigned int idxInj = 0;
        int32_t             intRes[VCT_TOTAL]   = {0};

        const unsigned int downsample_taps = DOWNSAMPLE_TAPS;
        const unsigned int idxInjPrev =   (0 == idxInj)
                                        ? (downsample_taps - 1u)
                                        : (idxInj - 1u);

        /* Copy the packed raw ADC value into the unpacked buffer; index 1 is the
         * most recent sample.
         */
        for (unsigned int idxSmp = 0; idxSmp < VCT_TOTAL; idxSmp++)
        {
            dspBuffer[idxInjPrev].smp[idxSmp]   = adcProc->samples[0].smp[idxSmp];
            dspBuffer[idxInj].smp[idxSmp]       = adcProc->samples[1].smp[idxSmp];
        }

        /* For an ODD number of taps, take the unique middle value to start. As
         * the filter is symmetric, this is the final element in the array.
         */
        const q15_t     coeff = firCoeffs[numCoeffUnique - 1u];
        unsigned int    idxMid = idxInj + (downsample_taps / 2) + 1u;
        if (idxMid >= downsample_taps) idxMid -= downsample_taps;

        for (unsigned int idxChannel = 0; idxChannel < VCT_TOTAL; idxChannel++)
        {
            intRes[idxChannel] += coeff * dspBuffer[idxMid].smp[idxChannel];
        }

        /* Loop over the FIR coefficients, sub loop through channels. The filter
         * is folded so the symmetric FIR coefficients are used for both samples.
         */
        unsigned int idxSmpStart = idxInj;
        unsigned int idxSmpEnd = ((downsample_taps - 1u) == idxInj) ? 0 : (idxInj + 1u);
        if (idxSmpEnd >= downsample_taps) idxSmpEnd -= downsample_taps;

        for (unsigned int idxCoeff = 0; idxCoeff < (numCoeffUnique - 1u); idxCoeff++)
        {
            const q15_t coeff = firCoeffs[idxCoeff];
            for (unsigned int idxChannel = 0; idxChannel < VCT_TOTAL; idxChannel++)
            {
                intRes[idxChannel] +=   coeff
                                    * (  dspBuffer[idxSmpStart].smp[idxChannel]
                                        + dspBuffer[idxSmpEnd].smp[idxChannel]);
            }

            /* Converge toward the middle, check for over/underflow */
            idxSmpStart -= 2u;
            if (idxSmpStart > downsample_taps) idxSmpStart += downsample_taps;

            idxSmpEnd += 2u;
            if (idxSmpEnd >= downsample_taps) idxSmpEnd -= downsample_taps;
        }

        /* Truncate with rounding to nearest LSB and place into field */
        /* REVISIT This is a fixed implementation for V/CT unpacking; abstract this */
        for (unsigned int idxChannel = 0; idxChannel < VCT_TOTAL; idxChannel++)
        {
            const q15_t resTrunc = __STRUNCATE(intRes[idxChannel]);
            if (idxChannel < NUM_V)
            {
                pDst->smpV[idxChannel] = resTrunc;
            }
            else
            {
                pDst->smpCT[idxChannel - NUM_V] = resTrunc;
            }
        }

        /* Each injection is 2 samples */
        idxInj += 2u;
        if (idxInj > (downsample_taps - 1))
        {
            idxInj -= (downsample_taps);
        }
    }
}


RAMFUNC ECM_STATUS_t
ecmInjectSample(void)
{
    unsigned int        zerox_flag = 0;
    static unsigned int idxInject;
    uint32_t            t_start = 0;

    if (0 != ecmCfg.timeMicros)
    {
        t_start = (*ecmCfg.timeMicros)();
    }

    SampleSet_t smpProc;
    SampleSet_t *pSmpProc = &smpProc;

    /* Copy the pre-processed sample data into the ring buffer */
    ecmFilterSample(pSmpProc);
    memcpy((void *)(sampleRingBuffer + idxInject), (const void *)pSmpProc, sizeof(SampleSet_t));

    /* Do power calculations */
    accumCollecting->numSamples++;

    /* TODO this is only for single phase currently, loop is there for later */
    const unsigned int idxLast = (idxInject - 1u) & (PROC_DEPTH - 1u);
    const q15_t thisV = sampleRingBuffer[idxInject].smpV[0];
    const q15_t lastV = sampleRingBuffer[idxLast].smpV[0];
    for (unsigned int idxV = 0; idxV < NUM_V; idxV++)
    {
        accumCollecting->processV[idxV].sumV_sqr    += (q22_t) thisV * thisV;
        accumCollecting->processV[idxV].sumV_deltas += (int32_t) thisV;
    }

    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        const q15_t lastCT = sampleRingBuffer[idxLast].smpCT[idxCT];
        accumCollecting->processCT[idxCT].sumPA         += (q22_t) lastCT * lastV;
        accumCollecting->processCT[idxCT].sumPB         += (q22_t) lastCT * thisV;
        accumCollecting->processCT[idxCT].sumI_sqr      += (q22_t) lastCT * lastCT;
        accumCollecting->processCT[idxCT].sumI_deltas   += (int32_t) lastCT;
    }

    /* Flag if there has been a (-) -> (+) crossing */
    if (0 == ecmCfg.zx_hw_stat)
    {
        zerox_flag = zeroCrossingSW(thisV);
    }
    else
    {
        zerox_flag = (*ecmCfg.zx_hw_stat)();
    }

    /* Check for zero crossing, swap buffers and pend event */
    if (1 == zerox_flag)
    {
        ecmSwapPtr((void **)&accumCollecting, (void **)&accumProcessing);
        memset((void *)accumCollecting, 0, sizeof(Accumulator_t));

        /* Clear the hardware zero crossing, if in use. */
        if (0 != ecmCfg.zx_hw_clr)
        {
            (*ecmCfg.zx_hw_clr)();
        }

        /* If out of the "warm up" period, then indicate a full cycle */
        if (0 == discardCycles)
        {
            return ECM_CYCLE_COMPLETE;
        }
        else
        {
            discardCycles--;
        }
    }

    /* Advance injection point, masking for overflow */
    idxInject = (idxInject + 1u) & (PROC_DEPTH - 1u);

    if (0 != ecmCfg.timeMicrosDelta)
    {
        perfActive->numSlices++;
        perfActive->microsSlices += (*ecmCfg.timeMicrosDelta)(t_start);
    }

    return ECM_CYCLE_ONGOING;
}


ECMPerformance_t *
ecmPerformance(void)
{
    ecmSwapPtr((void **)&perfActive, (void **)&perfIdle);
    memset(perfActive, 0, sizeof(ECMPerformance_t));

    return perfIdle;
}


RAMFUNC ECM_STATUS_t
ecmProcessCycle(void)
{
    uint32_t t_start = 0;

    if (0 != ecmCfg.timeMicros)
    {
        t_start = (*ecmCfg.timeMicros)();
    }

    ecmCycle.cycleCount++;

    /* Reused constants */
    const uint32_t numSamples       = accumProcessing->numSamples;
    const uint32_t numSamplesSqr    = numSamples * numSamples;

    /* RMS for V channels */
    for (unsigned int idxV = 0; idxV < NUM_V; idxV++)
    {
        /* Calculate RMS, subtracting off fine offset */
        accumProcessing->processV[idxV].sumV_deltas *= accumProcessing->processV[idxV].sumV_deltas;

        int meanSqr = accumProcessing->processV[idxV].sumV_sqr / numSamples;
        int dcCorr  = accumProcessing->processV[idxV].sumV_deltas / numSamplesSqr;
        meanSqr -= dcCorr;

        ecmCycle.rmsV[idxV] = qfp_fadd(ecmCycle.rmsV[idxV],
                                       qfp_fsqrt(qfp_int2float(meanSqr)));
    }

    /* CT channels */
    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        if (0 != ecmCfg.ctCfg[idxCT].active)
        {
            float   powerNow;
            int32_t ctCurrent;
            int32_t deltasScaled;
            int32_t sumI_deltas_sqr =   accumProcessing->processCT[idxCT].sumI_deltas
                                      * accumProcessing->processCT[idxCT].sumI_deltas;

            /* Apply phase calibration for CT interpolated between V samples */
            float sumRealPower = qfp_fmul(qfp_int2float(accumProcessing->processCT[idxCT].sumPA),
                                          ecmCfg.ctCfg[idxCT].phaseX);
            sumRealPower = qfp_fadd(sumRealPower,
                                    qfp_fmul(qfp_int2float(accumProcessing->processCT[idxCT].sumPB),
                                             ecmCfg.ctCfg[idxCT].phaseY));

            deltasScaled = sumI_deltas_sqr / numSamplesSqr;
            powerNow = qfp_fdiv(sumRealPower, qfp_int2float(numSamples));
            powerNow = qfp_fsub(powerNow, qfp_int2float(deltasScaled));

            ctCurrent = accumProcessing->processCT[idxCT].sumI_sqr / numSamples;
            ctCurrent -= deltasScaled;

            ecmCycle.valCT[idxCT].powerNow = qfp_fadd(ecmCycle.valCT[idxCT].powerNow,
                                                      powerNow);
            ecmCycle.valCT[idxCT].rmsCT = qfp_fadd(ecmCycle.valCT[idxCT].rmsCT,
                                                   qfp_fsqrt(qfp_int2float(ctCurrent)));
        }
        else
        {
            ecmCycle.valCT[idxCT].powerNow = 0.0f;
            ecmCycle.valCT[idxCT].rmsCT = 0.0f;
        }
    }

    if (0 != ecmCfg.timeMicrosDelta)
    {
        perfActive->numCycles++;
        perfActive->microsCycles += (*ecmCfg.timeMicrosDelta)(t_start);
    }

    if ((ecmCycle.cycleCount >= ecmCfg.reportCycles) || processTrigger)
    {
        processTrigger = 0;
        return ECM_REPORT_COMPLETE;
    }
    return ECM_REPORT_ONGOING;
}


RAMFUNC void
ecmProcessSet(ECMDataset_t *pData)
{
    uint32_t    t_start = 0;
    float       vCal;

    if (0 != ecmCfg.timeMicros)
    {
        t_start = (*ecmCfg.timeMicros)();
    }

    /* Mean value for each RMS voltage */
    for (unsigned int idxV = 0; idxV < NUM_V; idxV++)
    {
        vCal = ecmCfg.voltageCal[idxV];
        pData->rmsV[idxV] = qfp_fdiv(ecmCycle.rmsV[idxV],
                                     qfp_uint2float(ecmCycle.cycleCount));
        pData->rmsV[idxV] = qfp_fmul(pData->rmsV[idxV], vCal);
    }

    /* CT channels */
    vCal = ecmCfg.voltageCal[0];
    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        float   energyNow;
        float   powerNow;
        float   wattHoursRecent;

        if (0 != ecmCfg.ctCfg[idxCT].active)
        {
            powerNow = ecmCycle.valCT[idxCT].powerNow;
            powerNow = qfp_fmul(powerNow, qfp_fmul(ecmCfg.voltageCal[0],
                                                   ecmCfg.ctCfg->ctCal));
            pData->CT[idxCT].realPower  = qfp_float2int(qfp_fadd(qfp_fdiv(powerNow,
                                                                          ecmCycle.cycleCount),
                                                        0.5f));
            /* TODO add frequency deviation scaling */
            energyNow                       = qfp_fadd(powerNow,
                                                       pData->CT[idxCT].residualEnergy);
            wattHoursRecent                 = qfp_fdiv(energyNow, 3600.0f);
            pData->CT[idxCT].wattHour       += qfp_float2int(wattHoursRecent);
            pData->CT[idxCT].residualEnergy = qfp_fsub(energyNow,
                                                       qfp_fmul(wattHoursRecent,
                                                                3600.0f));
        }
        else
        {
            pData->CT[idxCT].wattHour = 0;
            pData->CT[idxCT].residualEnergy = 0.0f;
        }
    }

    /* Zero out cycle accummulator */
    memset((void *)&ecmCycle, 0, sizeof(ECMCycle_t));

    if (0 != ecmCfg.timeMicrosDelta)
    {
        perfActive->numDatasets++;
        perfActive->microsDatasets += (*ecmCfg.timeMicrosDelta)(t_start);
    }
}

void
ecmProcessSetTrigger(void)
{
    processTrigger = 1;
}
