#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "emon32.h"
#include "emon_CM.h"
#include "emon_CM_coeffs.h"

#define MAINS_FREQ  50.0
#define REPORT_CT   3 /* Number of CT channels to report */
#define REPORT_TIME 9.8f
#define REPORT_V    1 /* Number of V channels to report */
#define SMP_TICK    1000000u / SAMPLE_RATE / (VCT_TOTAL)
#define TEST_TIME   100E6 /* Time to run in microseconds */
#define VRMS_GOLD   235.0f

typedef struct wave_ {
  double omega;  /* Angular velocity */
  double phi;    /* Phase (rad) */
  double s;      /* Scale (0 < s <= 1.0) */
  int    offset; /* Constant offset, clamped if outside range */
} wave_t;

/*! @brief Convert a current in CT to a wave description
 *  @param [in] IRMS : RMS current
 *  @param [in] scaleCT : current to produce 333 mV RMS output
 *  @param [in] phase : CT phase
 *  @param [out] pW  : pointer to the wave struct
 */
static void currentToWave(double IRMS, int scaleCT, double phase, wave_t *w);

static void dynamicRun(int reports, bool prtReports);

/*! @brief Generates a Q11 [-1024, 1023] wave with configurable parameters
 *  @param [in] w       : pointer to wave information
 *  @param [in] tMicros : time in microseconds
 */
static q15_t generateWave(wave_t *w, int tMicros);

/*! @brief Print the output from the emonCM report
 *  @param [in] reportNum   Report number
 *  @param [in] time        Simulation time in us
 *  @param [in] pDataset    Pointer to the dataset
 */
static void printReport(int reportNum, int64_t time, ECMDataset_t *pDataset);

/*! @brief Convert a voltage into a wave description
 *  @param [in] vRMS : RMS voltage
 *  @param [out] pW  : pointer to the wave struct
 */
static void voltageToWave(double vRMS, wave_t *w);

static uint32_t time = 0;
ECM_STATUS_t    status;
SampleSet_t     smpProc;
unsigned int    smpIdx = 0;
ECMDataset_t    dataset;

volatile RawSampleSetPacked_t *volatile smpRaw[2];
wave_t wave[VCT_TOTAL];

static uint32_t timeMicros(void) { return time; }
static uint32_t timeMicrosDelta(uint32_t timePrev) { return time - timePrev; }

static void dynamicRun(int reports, bool prtReport) {
  int reportNum = 0;

  while (reportNum < reports) {

    for (int j = 0; j < 2; j++) {
      for (int i = 0; i < VCT_TOTAL; i++) {
        smpRaw[smpIdx]->samples[j].smp[i] = generateWave(&wave[i], time);
        time += SMP_TICK;
      }
    }
    smpIdx = !smpIdx;
    ecmDataBufferSwap();
    status = ecmInjectSample();

    if (ECM_REPORT_COMPLETE == status) {
      ecmProcessSet(&dataset);
      if (prtReport) {
        printReport(reportNum, time, &dataset);
      }
      reportNum++;
    }
  }
  ecmFlush();
}

int main(int argc, char *argv[]) {

  FILE     *fptr;
  ECMCfg_t *pEcmCfg;

  /* Copy and fold the half band coefficients */
  const int lutDepth = (numCoeffUnique - 1) * 2;
  int16_t   coeffLut[lutDepth];
  for (int i = 0; i < (lutDepth / 2); i++) {
    coeffLut[i]                  = firCoeffs[i];
    coeffLut[(lutDepth - 1 - i)] = firCoeffs[i];
  }

  /* Set all waves to 50 Hz, all CTs to 5 deg offset. The maximum amplitude
   * corresponds to 1.024 V at the emon32 input.
   */
  for (int i = 0; i < NUM_V; i++) {
    voltageToWave(230.0, &wave[i]);
  }

  for (int i = NUM_V; i < VCT_TOTAL; i++) {
    currentToWave(3.5, 5, 5.0, &wave[i]);
  }

  pEcmCfg             = ecmConfigGet();
  pEcmCfg->zx_hw_stat = 0;
  pEcmCfg->zx_hw_clr  = 0;

  /* ecmDataBuffer returns a pointer to the buffer which the DMA is putting
   * data into.
   */
  memset(&smpProc, 0, sizeof(SampleSet_t));
  smpRaw[0] = ecmDataBuffer();
  ecmDataBufferSwap();
  smpRaw[1] = ecmDataBuffer();
  ecmDataBufferSwap();

  pEcmCfg->downsample      = 1u;
  pEcmCfg->reportCycles    = (unsigned int)(REPORT_TIME * MAINS_FREQ);
  pEcmCfg->mainsFreq       = 50;
  pEcmCfg->samplePeriod    = 13;
  pEcmCfg->timeMicros      = &timeMicros;
  pEcmCfg->timeMicrosDelta = &timeMicrosDelta;

  for (int i = 0; i < NUM_V; i++) {
    pEcmCfg->vCfg[i].voltageCalRaw = 100.0f;
    pEcmCfg->vCfg[i].vActive       = (i == 0);
  }

  for (int i = 0; i < NUM_CT; i++) {
    pEcmCfg->ctCfg[i].active   = (i < 2);
    pEcmCfg->ctCfg[i].ctCalRaw = 20.0f;
    pEcmCfg->ctCfg[i].phCal    = 4.2f;
    pEcmCfg->ctCfg[i].vChan1   = 0;
    pEcmCfg->ctCfg[i].vChan2   = 0;
  }

  pEcmCfg->correction.valid  = true;
  pEcmCfg->correction.offset = 0;
  pEcmCfg->correction.gain   = (1 << 11);

  ecmConfigInit();

  printf("---- emon32 CM test ----\n\n");

  /* Sanity check by dumping a CSV of 10 cycles @ mains freq */
  printf("  Generating test sine...");
  fptr = fopen("cm-test-sine.csv", "w");
  for (int t = 0; t < ((1000000 * 10) / MAINS_FREQ);
       t += (SMP_TICK * (VCT_TOTAL))) {
    q15_t a = generateWave(&wave[0], t);
    fprintf(fptr, "%d,%d\n", t, a);
  }
  fclose(fptr);
  printf(" Done!\n\n");

  /* Print out test information */
  printf("  Test configuration:\n");
  printf("    - Number of V     : %d\n", NUM_V);
  printf("    - Number of CT    : %d\n", NUM_CT);
  printf("    - Mains frequency : %.0f Hz\n", MAINS_FREQ);
  printf("    - DSP enabled     : %s\n", pEcmCfg->downsample ? "Yes" : "No");
  printf("    - Report time     : %.2f s\n", REPORT_TIME);
  printf("    - Sample tick     : %d us\n", SMP_TICK);
  printf("\n");

  /* ============ START : HALF BAND TEST ============ */
  /* Reference : https://dspguru.com/dsp/faqs/fir/implementation/ */

  /* IMPULSE TEST
   * Inject an implulse, should get all the coefficients (except middle) out
   * TODO parameterise this for any size of filter
   */
  if (pEcmCfg->downsample) {
    printf("  Half band filter tests:\n");
    printf("    - Impulse: ");

    for (unsigned int i = 0; i < VCT_TOTAL; i++) {
      smpRaw[smpIdx]->samples[0].smp[i] = 0;
      smpRaw[smpIdx]->samples[1].smp[i] = INT16_MAX;
    }

    ecmDataBufferSwap();
    ecmFilterSample(&smpProc);
    if (coeffLut[0] != smpProc.smpV[0]) {
      printf("\nsmpRaw[smpIdx]->samples[0]: %d\n",
             smpRaw[smpIdx]->samples[0].smp[0]);
      printf("smpRaw[smpIdx]->samples[1]: %d\n",
             smpRaw[smpIdx]->samples[1].smp[0]);
      printf("smpProc.smpV[0]: %d\n", smpProc.smpV[0]);

      printf("Gold: %d Test: %d\n", coeffLut[0], smpProc.smpV[0]);
      return 1;
    }

    for (unsigned int i = 0; i < VCT_TOTAL; i++) {
      smpRaw[smpIdx]->samples[0].smp[0] = 0;
      smpRaw[smpIdx]->samples[1].smp[0] = 0;
    }
    for (unsigned int idxCoeff = 0; idxCoeff < 9u; idxCoeff++) {
      ecmFilterSample(&smpProc);
      if (!(coeffLut[idxCoeff + 1u] == smpProc.smpV[0])) {
        printf("\nGold: %d Test: %d\n", coeffLut[idxCoeff + 1u],
               smpProc.smpV[0]);
        return 1;
      }
    }
    printf("Complete\n\n");
  }

  /* Increment through the sample channels (2x for oversampling)
   * and generate the wave for each channel at each point.
   */
  printf("  Dynamic test...\n\n");
  printf("    - Phase 0°, PF = 1 ... ");
  dynamicRun(4, false);

  if ((dataset.CT[0].pf > 1.01f) || (dataset.CT[0].pf < 0.99f)) {
    printf("\nPF Gold: %.2f Test: %.2f\n", 1.00f, dataset.CT[0].pf);
    return 1;
  }
  if ((dataset.rmsV[0] > (VRMS_GOLD + 1.0f)) ||
      (dataset.rmsV[0] < (VRMS_GOLD - 1.0f))) {
    printf("\nVrms Gold: %.2f Test: %.2f\n", VRMS_GOLD, dataset.rmsV[0]);
    return 1;
  }
  printf("Done!\n");

  printf("    - Phase 90°, PF = 0 ... ");
  wave[NUM_V].phi = M_PI / 2;
  time            = 0;
  dynamicRun(4, false);
  printf("Done!\n");

  printf("    - Phase 180°, PF = -1 ... ");
  wave[NUM_V].phi = M_PI;
  time            = 0;
  dynamicRun(4, false);
  if ((dataset.CT[0].pf < -1.01f) || (dataset.CT[0].pf > -0.99f)) {
    printf("Gold: %.2f Test: %.2f\n", 1.00f, dataset.CT[0].pf);
    return 1;
  }
  printf("Done!\n");

  printf("\n  Finished!\n\n");
  return 0;
}

static void currentToWave(double IRMS, int scaleCT, double phase, wave_t *w) {
  double iPk = IRMS * sqrt(2);
  w->offset  = 0;
  w->omega   = 2 * M_PI * MAINS_FREQ;
  w->phi     = phase / 180;
  w->s       = iPk / scaleCT;
}

static q15_t generateWave(wave_t *w, int tMicros) {
  assert((w->s > 0.0) && (w->s <= 1.0));
  q15_t  wave;
  double a = sin(((w->omega * tMicros) / 1000000.0) + w->phi) * w->s;
  wave     = (q15_t)(a * 2048);
  wave += w->offset;

  /* Clip if the offset exceeds the bounds */
  if (wave < -2048) {
    wave = -2048;
  } else if (wave > 2047) {
    wave = 2047;
  }
  return wave;
}

static void printReport(int reportNum, int64_t time, ECMDataset_t *pDataset) {
  static int prevE = 0;
  int        thisE;

  thisE = pDataset->CT[0].wattHour;
  printf("    Report %d (t = %.2f s):\r\n", reportNum++, (time / 1000000.0));
  printf("      Vrms (V) : %.2f\r\n", pDataset->rmsV[0]);
  printf("      Irms (A) : %.2f\r\n", pDataset->CT[0].rmsI);
  printf("      P    (W) : %d\r\n", pDataset->CT[0].realPower);
  printf("      E    (Wh): %d (delta: %d)\r\n", thisE, (thisE - prevE));
  printf("      pF       : %.2f\r\n", pDataset->CT[0].pf);
  prevE = thisE;
}

static void voltageToWave(double vRMS, wave_t *w) {
  double vPk = vRMS * sqrt(2);
  w->offset  = 0;
  w->omega   = 2 * M_PI * MAINS_FREQ;
  w->phi     = 0;
  w->s       = vPk / 405.0;
}
