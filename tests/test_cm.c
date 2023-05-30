#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>

#include "emon_CM.h"

#define SAMPLE_RATE     4800u
#define MAINS_FREQ      50u
#define SMP_PER_CYCLE   SAMPLE_RATE / MAINS_FREQ

double
generateSine(int16_t *sine, unsigned int pts, double phi)
{
    double rmsAccum = 0;
    for (unsigned int i = 0; i < pts; i++)
    {
        double i_fp = (double)i;
        double theta = 2 * M_PI * i_fp / 96.0;
        double sineRes = sin(theta + phi);
        if (i % 2) rmsAccum += sineRes * sineRes;
        *sine++ = (int16_t)(sineRes * 2048.0);
    }

    return sqrt(rmsAccum / (double)(pts / 2));
}

int
main(int argc, char *argv[])
{

    const int16_t coeffLut[10] = {
        92, -279, 957, -2670, 10113, 10113, -2670, 957, -279, 92
    };

    int16_t sine_q15[SMP_PER_CYCLE];

    volatile RawSampleSetPacked_t *volatile smpRaw;
    SampleSet_t                             smpProc;

    Emon32Config_t  cfg;
    ECMSet_t        dataset;

    double rms = generateSine(sine_q15, SMP_PER_CYCLE, 0.0) * 325.22f;
    printf("Generated sine with RMS: %.2f\n", rms);


    /* ecmDataBuffer returns a pointer to the buffer which the DMA is putting
     * data into.
     */
    memset(&smpProc, 0, sizeof(SampleSet_t));
    smpRaw = ecmDataBuffer();

    /* Initialise dataset values */
    dataset.msgNum = 0;
    dataset.pulseCnt = 0;
    for (unsigned int i = 0; i < NUM_CT; i++)
    {
        dataset.CT[i].wattHour = 0;
    }

    /* Configuration for ECM */
    cfg.baseCfg.nodeID              = 17u;
    cfg.baseCfg.mainsFreq           = 50u;
    cfg.baseCfg.reportCycles        = 5u;
    cfg.voltageCfg[0].voltageCal    = 325.22f / 2048.0f;
    for (unsigned int i = 0; i < NUM_CT; i++)
    {
        cfg.ctCfg[i].ctCal  = 90.9f / 2048.0f;
        cfg.ctCfg[i].phaseX = 16419;
        cfg.ctCfg[i].phaseY = 16419;
    }
    ecmInit(&cfg);

    printf("---- emon32 CM test ----\n\n");

    /* Half band tests : https://dspguru.com/dsp/faqs/fir/implementation/ */

    /* IMPULSE TEST
     * Inject an implulse, should get all the coefficients (except middle) out
     * TODO parameterise this for any size of filter
     */
    printf("  Half band filter tests:\n");
    printf("    - Impulse: ");

    for (unsigned int i = 0; i < VCT_TOTAL; i++)
    {
        smpRaw->samples[0].smp[i] = 0;
        smpRaw->samples[1].smp[i] = INT16_MAX;
    }

    ecmFilterSample(&smpProc);
    if (coeffLut[0] != smpProc.smpV[0])
    {
        printf("Gold: %d Test: %d\n", coeffLut[0], smpProc.smpV[0]);
        assert(0);
    }

    for (unsigned int i = 0; i < VCT_TOTAL; i++)
    {
        smpRaw->samples[0].smp[0] = 0;
        smpRaw->samples[1].smp[0] = 0;
    }
    for (unsigned int idxCoeff = 0; idxCoeff < 9u; idxCoeff++)
    {
        ecmFilterSample(&smpProc);
        assert(coeffLut[idxCoeff + 1u] == smpProc.smpV[0]);
    }
    printf("Complete\n\n");

    /* Generate a Q1.11 sine wave and use the smpRaw buffer to inject this
     * into the ecmInjectSample routine.
     */
    printf("  Inject sample test (full amplitude):\n");
    printf("    - Number of samples per cycle (2f): %d\n", SMP_PER_CYCLE);

    unsigned int smpCnt = 0;
    do
    {
        do
        {
            for (unsigned int j = 0; j < VCT_TOTAL; j++)
            {
                smpRaw->samples[0].smp[j] = sine_q15[smpCnt];
                smpRaw->samples[1].smp[j] = sine_q15[smpCnt+1];
            }
            smpCnt += 2;
            if (smpCnt >= SMP_PER_CYCLE) smpCnt = 0;

        } while (ECM_CYCLE_COMPLETE != ecmInjectSample());

    } while (ECM_REPORT_COMPLETE != ecmProcessCycle());

    ecmProcessSet(&dataset);
    printf("MSGNUM: %d\t", dataset.msgNum);
    printf("V RMS: %.2f\t", dataset.rmsV[0]);
    printf("Wh[0]: %d\t", dataset.CT[0].wattHour);
    printf("P[0]: %.2f", dataset.CT[0].realPower);
    printf("\n");
}
