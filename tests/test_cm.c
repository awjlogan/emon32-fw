#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "emon32.h"
#include "emon_CM.h"
#include "emon_CM_coeffs.h"

#define SAMPLE_RATE     4800u
#define MAINS_FREQ      50u
#define REPORT_TIME     9.8f
#define SMP_TICK        1000000u / SAMPLE_RATE / (VCT_TOTAL)
#define TEST_TIME       50E6    /* Time to run in microseconds */
#define REPORT_V        1       /* Number of V channels to report */
#define REPORT_CT       3       /* Number of CT channels to report */

typedef struct wave_ {
    double  omega;  /* Angular velocity */
    double  phi;    /* Phase (rad) */
    double  s;      /* Scale (0 < s <= 1.0) */
    int     offset; /* Constant offset, clamped if outside range */
} wave_t;


/*! @brief Generates a Q11 [-2048, 2047] wave with configurable parameters
 *  @param [in] w       : pointer to wave information
 *  @param [in] tMicros : time in microseconds
 */
q15_t generateWave(wave_t *w, int tMicros);


int
main(int argc, char *argv[])
{
    int time        = 0;
    int reportNum   = 0;

    FILE            *fptr;
    ECMDataset_t    dataset;
    ECMCfg_t        *pEcmCfg;
    PhaseXY_t       phase;
    ECM_STATUS_t    status;
    wave_t          wave[VCT_TOTAL];

    /* Copy and fold the half band coefficients */
    const int lutDepth = (numCoeffUnique - 1) * 2;
    int16_t coeffLut[lutDepth];
    for (int i = 0; i < (lutDepth / 2); i++)
    {
        coeffLut[i] = firCoeffs[i];
        coeffLut[(lutDepth - 1 - i)] = firCoeffs[i];
    }

    volatile RawSampleSetPacked_t *volatile smpRaw[2];
    SampleSet_t                             smpProc;
    unsigned int                            smpIdx = 0;

    /* Set all waves to 50 Hz, all CTs to 5 deg offset. The maximum amplitude
     * corresponds to 1.024 V at the emon32 input.
     */
    for (int i = 0; i < VCT_TOTAL; i++)
    {
        wave[i].omega = 2 * M_PI * 50.0;
        /* 230 V_rms ~(325 / 405) and 2.84 A_rms ~(4 / 5) */
        wave[i].s     = 0.803f;
    }
    for (int i = NUM_V; i < VCT_TOTAL; i++)
    {
        wave[i].phi = 5.0/180.0;
    }

    pEcmCfg = ecmConfigGet();

    /* ecmDataBuffer returns a pointer to the buffer which the DMA is putting
     * data into.
     */
    memset(&smpProc, 0, sizeof(SampleSet_t));
    smpRaw[0] = ecmDataBuffer();
    ecmDataBufferSwap();
    smpRaw[1] = ecmDataBuffer();
    ecmDataBufferSwap();

    pEcmCfg->downsample = 1u;
    pEcmCfg->reportCycles = (unsigned int)(REPORT_TIME * MAINS_FREQ);
    pEcmCfg->mainsFreq = 50;
    pEcmCfg->sampleRateHz = (SAMPLE_RATE / 2);
    ecmConfigInit();

    for (int i = 0; i < NUM_V; i++)
    {
        pEcmCfg->voltageCal[i] = ecmCalibrationCalculate(405.0f);
    }

    for (int i = 0; i < NUM_CT; i++)
    {
        pEcmCfg->ctCfg[i].active    = 1;
        pEcmCfg->ctCfg[i].ctCal     = ecmCalibrationCalculate(5.0f);
        phase = ecmPhaseCalculate(5.0f, i);
        pEcmCfg->ctCfg[i].phaseX    = phase.phaseX;
        pEcmCfg->ctCfg[i].phaseY    = phase.phaseY;
        pEcmCfg->ctCfg[i].vChan     = 0;
    }

    printf("---- emon32 CM test ----\n\n");

    /* Sanity check by dumping a CSV of 10 cycles @ mains freq */
    printf("  Generating test sine...");
    fptr = fopen("cm-test-sine.csv", "w");
    for (int t = 0; t < ((1000000 * 10) / MAINS_FREQ); t += (SMP_TICK * (VCT_TOTAL)))
    {
        q15_t a = generateWave(&wave[0], t);
        fprintf(fptr, "%d,%d\n", t, a);
    }
    fclose(fptr);
    printf(" Done!\n\n");

    /* Print out test information */
    printf("  Test configuration:\n");
    printf("    - Number of V     : %d\n", NUM_V),
    printf("    - Number of CT    : %d\n", NUM_CT);
    printf("    - Mains frequency : %d\n", MAINS_FREQ);
    printf("    - DSP enabled     : %s\n", pEcmCfg->downsample ? "Yes" : "No");
    printf("    - Report time     : %.2f\n", REPORT_TIME);
    printf("    - Sample tick (us): %d\n", SMP_TICK);
    printf("\n");

    /* ============ START : HALF BAND TEST ============ */
    /* Reference : https://dspguru.com/dsp/faqs/fir/implementation/ */

    /* IMPULSE TEST
     * Inject an implulse, should get all the coefficients (except middle) out
     * TODO parameterise this for any size of filter
     */
    if (pEcmCfg->downsample)
    {
        printf("  Half band filter tests:\n");
        printf("    - Impulse: ");

        for (unsigned int i = 0; i < VCT_TOTAL; i++)
        {
            smpRaw[smpIdx]->samples[0].smp[i] = 0;
            smpRaw[smpIdx]->samples[1].smp[i] = INT16_MAX;
        }

        ecmDataBufferSwap();
        ecmFilterSample(&smpProc);
        if (coeffLut[0] != smpProc.smpV[0])
        {
            printf("\nsmpRaw[smpIdx]->samples[0]: %d\n", smpRaw[smpIdx]->samples[0].smp[0]);
            printf("smpRaw[smpIdx]->samples[1]: %d\n", smpRaw[smpIdx]->samples[1].smp[0]);
            printf("smpProc.smpV[0]: %d\n", smpProc.smpV[0]);

            printf("Gold: %d Test: %d\n", coeffLut[0], smpProc.smpV[0]);
            assert(0);
        }

        for (unsigned int i = 0; i < VCT_TOTAL; i++)
        {
            smpRaw[smpIdx]->samples[0].smp[0] = 0;
            smpRaw[smpIdx]->samples[1].smp[0] = 0;
        }
        for (unsigned int idxCoeff = 0; idxCoeff < 9u; idxCoeff++)
        {
            ecmFilterSample(&smpProc);
            assert(coeffLut[idxCoeff + 1u] == smpProc.smpV[0]);
        }
        printf("Complete\n\n");
    }

    printf("  Dynamic test...\n\n");
    while (time < TEST_TIME)
    {
        /* Increment through the sample channels (2x for oversampling)
         * and generate the wave for each channel at each point.
         */
        for (int j = 0; j < 2; j++)
        {
            for (int i = 0; i < VCT_TOTAL; i++)
            {
                smpRaw[smpIdx]->samples[j].smp[i] = generateWave(&wave[i], time);
                time += SMP_TICK;
            }
        }
        smpIdx = !smpIdx;
        ecmDataBufferSwap();
        status = ecmInjectSample();

        if (ECM_CYCLE_COMPLETE == status)
        {
            status = ecmProcessCycle();
        }

        if (ECM_REPORT_COMPLETE == status)
        {
            printf("    Report %d: ", reportNum++);
            ecmProcessSet(&dataset);
            for (int i = 0; i < REPORT_V; i++)
            {
                printf("v%d:%.2f,", i, dataset.rmsV[i]);
            }
            for (int i = 0; i < REPORT_CT; i++)
            {
                printf("P%d:%d,E%d:%d",
                       i, dataset.CT[i].realPower,
                       i, dataset.CT[i].wattHour);
                printf("%c", ((i != (REPORT_CT - 1)) ? ',' : '\n'));
            }
        }
    }
    printf("Done!\n\n");
}

q15_t generateWave(wave_t *w, int tMicros)
{
    assert((w->s > 0.0) && (w->s <= 1.0));
    q15_t wave;
    double a = sin(((w->omega * tMicros)/1000000.0) + w->phi) * w->s;
    wave = (q15_t)(a * 2048);
    wave += w->offset;

    /* Clip if the offset exceeds the bounds */
    if (wave < -2048)
    {
        wave = -2048;
    }
    else if (wave > 2047)
    {
        wave = 2047;
    }
    return wave;
}
