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
#define SMP_TICK        1000000u / SAMPLE_RATE / VCT_TOTAL

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
    int time = 0;

    Emon32Config_t  cfg;
    ECMDataset_t    dataset;
    ECMCfg_t        *pEcmCfg;

    const int16_t coeffLut[10] = {
        92, -279, 957, -2670, 10113, 10113, -2670, 957, -279, 92
    };

    volatile RawSampleSetPacked_t *volatile smpRaw;
    SampleSet_t                             smpProc;

    pEcmCfg = ecmGetConfig();

    /* ecmDataBuffer returns a pointer to the buffer which the DMA is putting
     * data into.
     */
    memset(&smpProc, 0, sizeof(SampleSet_t));
    smpRaw = ecmDataBuffer();

    /* Configuration for ECM */
    cfg.baseCfg.nodeID              = 17u;
    cfg.baseCfg.mainsFreq           = 50u;
    cfg.baseCfg.reportTime          = 9.8;
    cfg.voltageCfg[0].voltageCal    = 325.22f / 2048.0f;
    for (unsigned int i = 0; i < NUM_CT; i++)
    {
        cfg.ctCfg[i].ctCal  = 90.9f / 2048.0f;
        cfg.ctCfg[i].phase = 5.0;
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

    // /* Generate a Q1.11 sine wave and use the smpRaw buffer to inject this
    //  * into the ecmInjectSample routine.
    //  */
    // printf("  Inject sample test (full amplitude):\n");
    // printf("    - Number of samples per cycle (2f): %d\n", SMP_PER_CYCLE);

    // unsigned int smpCnt = 0;
    // do
    // {
    //     do
    //     {
    //         for (unsigned int j = 0; j < VCT_TOTAL; j++)
    //         {
    //             smpRaw->samples[0].smp[j] = sine_q15[smpCnt];
    //             smpRaw->samples[1].smp[j] = sine_q15[smpCnt+1];
    //         }
    //         smpCnt += 2;
    //         if (smpCnt >= SMP_PER_CYCLE) smpCnt = 0;

    //     } while (ECM_CYCLE_COMPLETE != ecmInjectSample());

    // } while (ECM_REPORT_COMPLETE != ecmProcessCycle());

    ecmProcessSet(&dataset);
}

q15_t generateWave(wave_t *w, int tMicros, double s)
{
    double a = sin(((w->omega * tMicros)/1000000.0) + w->phi) * s;
    return (q15_t)a * 2048;
}
