#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "emon_CM.h"
#include "emon32.h"

#define SAMPLE_RATE     4800u
#define MAINS_FREQ      50u
#define REPORT_TIME     9.8f
#define SMP_TICK        1000000u / SAMPLE_RATE / VCT_TOTAL
#define TEST_TIME       50E6    /* Time to run in microseconds */

typedef struct {
    double omega;
    double phi;
} wave_t;


/*! @param [in] w       : pointer to wave information
 *  @param [in] tMicros : time in microseconds
 *  @param [in] s       : scaling factor (0 < s <= 1.0)
 */
q15_t generateWave(wave_t *w, int tMicros, double s);


int
main(int argc, char *argv[])
{
    int time        = 0;
    int reportNum   = 0;

    ECMDataset_t    dataset;
    ECMCfg_t        *pEcmCfg;
    PhaseXY_t       phase;
    ECM_STATUS_t    status;
    wave_t          wave[VCT_TOTAL];

    const int16_t coeffLut[10] = {
        92, -279, 957, -2670, 10113, 10113, -2670, 957, -279, 92
    };

    volatile RawSampleSetPacked_t *volatile smpRaw;
    SampleSet_t                             smpProc;

    /* Set all waves to 50 Hz, all CTs to 5 deg offset */
    for (int i = 0; i < VCT_TOTAL; i++)
    {
        wave[i].omega = 2 * M_PI * 50.0;
    }
    for (int i = NUM_V; i < VCT_TOTAL; i++)
    {
        wave[i].phi = 5.0/180.0;
    }

    pEcmCfg = ecmGetConfig();

    /* ecmDataBuffer returns a pointer to the buffer which the DMA is putting
     * data into.
     */
    memset(&smpProc, 0, sizeof(SampleSet_t));
    smpRaw = ecmDataBuffer();

    pEcmCfg->downsample = 1u;
    pEcmCfg->reportCycles = (unsigned int)(REPORT_TIME * MAINS_FREQ);

    for (int i = 0; i < NUM_V; i++)
    {
        pEcmCfg->voltageCal[i] = 268.97f;
    }

    phase = ecmPhaseCalculate(5.0f);
    for (int i = 0; i < NUM_CT; i++)
    {
        pEcmCfg->ctCfg[i].active    = 1;
        pEcmCfg->ctCfg[i].ctCal     = 90.91f;
        pEcmCfg->ctCfg[i].phaseX    = phase.phaseX;
        pEcmCfg->ctCfg[i].phaseY    = phase.phaseY;
        pEcmCfg->ctCfg[i].vChan     = 0;
    }

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

    printf("Dynamic test...\n\n");
    while (time < TEST_TIME)
    {
        /* Increment through the sample channels (2x for oversampling)
         * and generate the wave for each channel at each point.
         */
        for (int j = 0; j < 2; j++)
        {
            for (int i = 0; i < VCT_TOTAL; i++)
            {
                smpRaw->samples[j].smp[i] = generateWave(&wave[i], time, 1.0);
                time += SMP_TICK;
            }
        }
        ecmDataBufferSwap();
        status = ecmInjectSample();

        if (ECM_CYCLE_COMPLETE == status)
        {
            status = ecmProcessCycle();
        }

        if (ECM_REPORT_COMPLETE == status)
        {
            printf("Report %d: ", reportNum++);
            ecmProcessSet(&dataset);
            for (int i = 0; i < 1; i++)
            {
                printf("v%d:%.2f,", i, dataset.rmsV[i]);
            }
            for (int i = 0; i < 6; i++)
            {
                printf("P%d:%.2f,E%d:%d",
                       i, dataset.CT[i].realPower,
                       i, dataset.CT[i].wattHour);
                printf("%c", ((i != 5) ? ',' : '\n'));
            }
        }
    }
    printf("Done!\n\n");
}

q15_t generateWave(wave_t *w, int tMicros, double s)
{
    double a = sin(((w->omega * tMicros)/1000000.0) + w->phi) * s;
    return (q15_t)(a * 2048);
}
