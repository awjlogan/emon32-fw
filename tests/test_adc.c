#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>

struct CorrPair {
    int32_t gain;
    int32_t offset;
};

float
calcGain_f(int32_t y0tick, int32_t y1tick)
{
    float ret = (float)(1024 - 3083) / (float)(y0tick - y1tick);
    return ret;
}

int32_t
calcOffset(int32_t y, float gain)
{
    return (int32_t)(0.5f + (float)y / gain) - 1024;
}

void
calcCorrections(struct CorrPair *pCorr, int32_t y0tick, int32_t y1tick)
{
    float g_fp = calcGain_f(y0tick, y1tick);
    pCorr->offset = calcOffset(y0tick, g_fp);
    pCorr->gain = g_fp * (1 << 11);

    printf("y0tick  : %d  y1tick:     %d\n", y0tick, y1tick);
    printf("GAINCORR: 0x%X OFFSETCORE: %d\n\n", pCorr->gain, pCorr->offset);
}

int
main(int argc, char *argv[])
{
    printf("Testing ADC calibration routine\n");
    printf("Due to the calls to external registers, this is verbatim from <driver_ADC.c>\n\n");

    struct CorrPair corr;

    calcCorrections(&corr, 1034, 3093);
    assert(corr.gain == 0x800 && corr.offset == 10);
}
