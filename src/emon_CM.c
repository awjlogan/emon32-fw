#include <string.h>

#include "emon_CM.h"

#ifndef HOSTED

#include "qfplib-m0-full.h"

#else

#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include "emonCM_test.h"

#endif /* HOSTED */

/* Number of samples available for power calculation. must be power of 2 */
#define PROC_DEPTH      32u     /* REVISIT only need  to store V values */
#define ZC_HYST         3u      /* Zero crossing hysteresis */
#define EQUIL_CYCLES    5u      /* Number of cycles to discard at POR */

/*************************************
 * Function prototypes
 *************************************/

static inline uint8_t   __CLZ           (uint32_t data) RAMFUNC;
static inline q15_t     __STRUNCATE     (int32_t val)   RAMFUNC;
static q15_t            sqrt_q15        (q15_t in)      RAMFUNC;
static void             ecmSwapPtr      (void **pIn1, void **pIn2);
static int              zeroCrossingSW  (q15_t smpV)    RAMFUNC;

/******** FIXED POINT MATHS FUNCTIONS ********
 *
 * Adapted from Arm CMSIS-DSP: https://github.com/ARM-software/CMSIS-DSP
 */

#define Q12QUARTER  0x2000
/* Source/CommonTables/arm_common_tables.c:70534 */
const q15_t sqrt_initial_lut[16] = {
    8192, 7327, 6689, 6193,
    5793, 5461, 5181, 4940,
    4730, 4544, 4379, 4230,
    4096, 3974, 3862, 3759
};

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


/*! @brief Count the number of leading 0s
 *  @param[in] data : input value
 *  @return Number of leading 0s in data
 */
/* TODO ARMv7-M has a CLZ instruction, use intrinsic */
static RAMFUNC inline uint8_t
__CLZ(uint32_t data)
{
    uint32_t count = 0u;
    uint32_t mask = 0x80000000u;

    if (0 == data)
    {
        return 32u;
    }

    while ((data & mask) == 0U)
    {
        count += 1U;
        mask = mask >> 1U;
    }
    return count;
}

/*! @brief Square root of Q15 number
 *  @details Modified CMSIS-DSP: Source/FastMathFunctions/arm_sqrt_q15.c
 *  @param [in] in : input vaue in range [0 +1)
 *  @return square root of the input value
 */
static RAMFUNC q15_t
sqrt_q15(q15_t in)
{
    q15_t number, var1, signBits1, temp;
    number = in;

    signBits1 = __CLZ(number) - 17;
    if (0 == (signBits1 % 2))
    {
        number = number << signBits1;
    }
    else
    {
        number = number << (signBits1 - 1);
    }

    /* Start value for 1/sqrt(x) for Newton-Raphson */
    var1 = sqrt_initial_lut[(number >> 11) - (Q12QUARTER >> 11)];

    /* TODO Loop is unrolled, can compact if needed */
    temp = ((q31_t) var1 * var1) >> 12;
    temp = ((q31_t) number * temp) >> 15;
    temp = 0x3000 - temp;
    var1 = ((q31_t) var1 * temp) >> 13;

    temp = ((q31_t) var1 * var1) >> 12;
    temp = ((q31_t) number * temp) >> 15;
    temp = 0x3000 - temp;
    var1 = ((q31_t) var1 * temp) >> 13;

    temp = ((q31_t) var1 * var1) >> 12;
    temp = ((q31_t) number * temp) >> 15;
    temp = 0x3000 - temp;
    var1 = ((q31_t) var1 * temp) >> 13;

    /* Multiply the inverse sqrt with the original, and shift down */
    var1 = ((q15_t) (((q31_t) number * var1) >> 12));
    if (0 == (signBits1 % 2))
    {
        var1 = var1 >> (signBits1 / 2);
    }
    else
    {
        var1 = var1 >> ((signBits1 - 1) / 2);
    }
    return var1;
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


ECMCfg_t *
ecmGetConfig(void)
{
    return &ecmCfg;
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
 * Power accumulators
 *****************************************************************************/

static Accumulator_t    accumBuffer[2];
static Accumulator_t *  accumCollecting = accumBuffer;
static Accumulator_t *  accumProcessing = accumBuffer + 1;
static ECMCycle_t       ecmCycle;

/******************************************************************************
 * Functions
 *****************************************************************************/

/*! @brief Zero crossing detection, software
 *  @param [in] smpV : current voltage sample
 *  @return 1 for positive crossing, 0 otherwise
 */
RAMFUNC int
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


PhaseXY_t
ecmPhaseCalculate(float phase)
{
    PhaseXY_t phaseXY;

    float sampleRate = 2400.0f;
    float phaseShift = phase;

    phaseXY.phaseY = qfp_fdiv(qfp_fsin(phaseShift),
                              qfp_fsin(sampleRate));
    phaseXY.phaseX = qfp_fsub(qfp_fcos(phaseShift),
                              (qfp_fmul(phaseXY.phaseY,
                                        qfp_fcos(sampleRate))));

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
        const unsigned int  numCoeffUnique      = 6u;
        const int16_t       firCoeffs[6]        = {   92,  -279,   957,
                                                   -2670, 10113, 16339};

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
        unsigned int idxSmpEnd = ((downsample_taps - 1u) == idxInj) ? 0 : idxInj + 1u;
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
        accumCollecting->processV[idxV].sumV_sqr += (q31_t) thisV * thisV;
        accumCollecting->processV[idxV].sumV_deltas += (q31_t) thisV;
    }

    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        const q15_t lastCT = sampleRingBuffer[idxLast].smpCT[idxCT];
        accumCollecting->processCT[idxCT].sumPA += (q31_t) lastCT * lastV;
        accumCollecting->processCT[idxCT].sumPB += (q31_t) lastCT * thisV;
        accumCollecting->processCT[idxCT].sumI_sqr += (q31_t) lastCT * lastCT;
        accumCollecting->processCT[idxCT].sumI_deltas += (q31_t) lastCT;
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

    return ECM_CYCLE_ONGOING;
}

RAMFUNC ECM_STATUS_t
ecmProcessCycle(void)
{
    ecmCycle.cycleCount++;

    /* Reused constants */
    const uint32_t numSamples       = accumProcessing->numSamples;
    const uint32_t numSamplesSqr    = numSamples * numSamples;

    /* RMS for V channels */
    for (unsigned int idxV = 0; idxV < NUM_V; idxV++)
    {
        /* Truncate and calculate RMS, subtracting off fine offset */
        accumProcessing->processV[idxV].sumV_sqr       = __STRUNCATE(accumProcessing->processV[idxV].sumV_sqr);
        accumProcessing->processV[idxV].sumV_deltas    *= accumProcessing->processV[idxV].sumV_deltas;
        accumProcessing->processV[idxV].sumV_deltas    = __STRUNCATE(accumProcessing->processV[idxV].sumV_deltas);

        q15_t thisRms = sqrt_q15(
                        (accumProcessing->processV[idxV].sumV_sqr / numSamples)
                      - (accumProcessing->processV[idxV].sumV_deltas / (numSamplesSqr)));
        ecmCycle.rmsV[idxV] += thisRms;
    }

    /* CT channels */
    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        if (0 != ecmCfg.ctCfg[idxCT].active)
        {
            accumProcessing->processCT[idxCT].sumI_sqr    = __STRUNCATE(accumProcessing->processCT[idxCT].sumI_sqr);
            int32_t sumI_deltas_sqr =   accumProcessing->processCT[idxCT].sumI_deltas
                                      * accumProcessing->processCT[idxCT].sumI_deltas;
            sumI_deltas_sqr = __STRUNCATE(sumI_deltas_sqr);


            /* Apply phase calibration for CT interpolated between V samples */
            int32_t sumRealPower =   accumProcessing->processCT[idxCT].sumPA * ecmCfg.ctCfg[idxCT].phaseX
                                   + accumProcessing->processCT[idxCT].sumPB * ecmCfg.ctCfg[idxCT].phaseY;

            ecmCycle.valCT[idxCT].powerNow += (sumRealPower / numSamples) - (sumI_deltas_sqr / numSamplesSqr);

            ecmCycle.valCT[idxCT].rmsCT +=   sqrt_q15(((accumProcessing->processCT[idxCT].sumI_sqr / numSamples)
                                           - (sumI_deltas_sqr / numSamplesSqr)));
        }
        else
        {
            ecmCycle.valCT[idxCT].powerNow = 0;
            ecmCycle.valCT[idxCT].rmsCT = 0;
        }
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
    float vCal;
    /* Mean value for each RMS voltage */
    for (unsigned int idxV = 0; idxV < NUM_V; idxV++)
    {
        vCal = ecmCfg.voltageCal[idxV];
        pData->rmsV[idxV] = qfp_fdiv(ecmCycle.rmsV[idxV], (float)ecmCycle.cycleCount);
        pData->rmsV[idxV] = qfp_fmul(pData->rmsV[idxV], vCal);
    }

    /* CT channels */
    vCal = ecmCfg.voltageCal[0];
    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        int     wattHoursRecent;
        float   energyNow;
        float   scaledPower;

        if (0 != ecmCfg.ctCfg[idxCT].active)
        {
            scaledPower                 =   qfp_fmul(qfp_fmul(ecmCycle.valCT[idxCT].powerNow,
                                                              vCal),
                                                     ecmCfg.ctCfg[idxCT].ctCal);
            pData->CT[idxCT].realPower  = qfp_fadd(scaledPower, 0.5f);

            /* TODO add frequency deviation scaling */
            energyNow                       = qfp_fadd(scaledPower, pData->CT[idxCT].residualEnergy);
            wattHoursRecent                 = (int)energyNow / 3600;
            pData->CT[idxCT].wattHour       += wattHoursRecent;
            pData->CT[idxCT].residualEnergy = qfp_fsub(energyNow,
                                                       qfp_fmul(wattHoursRecent, 3600.0f));
        }
        else
        {
            pData->CT[idxCT].wattHour = 0;
            pData->CT[idxCT].residualEnergy = 0.0f;
        }
    }

    /* Zero out cycle accummulator */
    memset((void *)&ecmCycle, 0, sizeof(ECMCycle_t));
}

void
ecmProcessSetTrigger(void)
{
    processTrigger = 1;
}
