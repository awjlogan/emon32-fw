#include "driver_ADC.h"
#include "driver_DMAC.h"
#include "driver_PORT.h"
#include "driver_SAMD.h"
#include "emon32_samd.h"

#include "emon32.h"
#include "emon_CM.h"
#include "qfplib-m0-full.h"

static int16_t correctionGain;
static int16_t correctionOffset;
static bool    correctionValid;

static void    adcCalibrate(void);
static int16_t adcCalibrateSmp(int pin);
static void    adcConfigureDMAC(void);
static void    adcSync(void);

/*! @brief Load gain and offset registers for automatic compensation. Only
 *         available when using SAMD21 with sufficient ADC pins.
 */
static void adcCalibrate(void) {
  /* Expected ADC values for 1/4 and 3/4 scale, /2 for differential */
  const int32_t refScale14 = -16383 / 2;
  const int32_t refScale34 = 16382 / 2;

  /* Real values from ADC conversion */
  int16_t expScale14;
  int16_t expScale34;

  /* Calibration values */
  int32_t offset_inter[2];
  int32_t offset;

  float gain_fp;
  int   gain;

  /* Set up ADC for maximum sampling length and averaging. This results in a
   * 16 bit signed value in RESULT.
   */
  ADC->SAMPCTRL.reg = 0x1Fu;
  ADC->AVGCTRL.reg  = ADC_AVGCTRL_SAMPLENUM_1024;
  ADC->CTRLB.reg =
      ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_DIFFMODE | ADC_CTRLB_RESSEL_16BIT;
  adcSync();

  /* Read 1/4 and 3/4 scale readings. The 1/4 scale is read twice as the first
   * read from the ADC is not defined through reset.
   */
  ADC->CTRLA.reg |= ADC_CTRLA_ENABLE;
  adcSync();

  expScale14 = adcCalibrateSmp(ADC_INPUTCTRL_MUXPOS_PIN18);
  expScale14 = adcCalibrateSmp(ADC_INPUTCTRL_MUXPOS_PIN18);
  expScale34 = adcCalibrateSmp(ADC_INPUTCTRL_MUXPOS_PIN17);

  ADC->CTRLA.reg &= ~ADC_CTRLA_ENABLE;
  adcSync();

  /* The corrected value is:
   * (Conversion + -OFFSET) * GAINCORR
   * Solve two equations in two unknowns for OFFSETCORR and GAINCORR.
   *  : G = (y0 - y1) / (y0' - y1') [y' is the actual conversion value]
   *  : C = y' - y / G
   * OFFSETCORR is a 12bit signed value (33.1.18 Offset Correction).
   * GAINCORR is 1 unsigned bit + 11 fractional bits (33.8.17 Gain Correction)
   *  : 1/2 < GAINCORR < 2.
   */
  gain_fp = qfp_fdiv(qfp_int2float((refScale14 - refScale34)),
                     qfp_int2float((expScale14 - expScale34)));
  gain    = qfp_float2fix(gain_fp, 11);

  offset_inter[0] = qfp_float2int(qfp_fadd(
                        0.5f, qfp_fdiv(qfp_int2float(refScale14), gain_fp))) -
                    expScale14;
  offset_inter[1] = qfp_float2int(qfp_fadd(
                        0.5f, qfp_fdiv(qfp_int2float(refScale34), gain_fp))) -
                    expScale34;
  offset = (offset_inter[0] + offset_inter[1]) / 2;

  /* Registers are 12 bit, shift 16 bit offet intermediate offset 4. The gain
   * value is in Q1.11 format already.
   */
  correctionOffset = (int16_t)offset >> 4;
  correctionGain   = (int16_t)gain;
  correctionValid  = true;
}

static int16_t adcCalibrateSmp(int pin) {
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_PIN0 | pin;
  ADC->INTFLAG.reg |= ADC_INTFLAG_RESRDY;
  ADC->SWTRIG.reg = ADC_SWTRIG_START;
  while (0 == (ADC->INTFLAG.reg & ADC_INTFLAG_RESRDY))
    ;
  return ADC->RESULT.reg;
}

/*! @brief Load gain and offset registers for automatic compensation. Only
 *         available when using SAMD21 with sufficient ADC pins.
 */
static void adcConfigureDMAC(void) {
  DMACCfgCh_t              dmacConfig;
  volatile DmacDescriptor *dmacDesc[2];
  unsigned int             dmaChan[2] = {DMA_CHAN_ADC0, DMA_CHAN_ADC1};

  volatile RawSampleSetPacked_t *adcBuffer[2];

  /* Get the contiguous data buffers */
  adcBuffer[0] = ecmDataBuffer();
  adcBuffer[1] = adcBuffer[0] + 1;

  dmacConfig.ctrlb = DMAC_CHCTRLB_LVL(3u) |
                     DMAC_CHCTRLB_TRIGSRC(ADC_DMAC_ID_RESRDY) |
                     DMAC_CHCTRLB_TRIGACT_BEAT;

  for (unsigned int i = 0; i < 2; i++) {
    dmacDesc[i] = dmacGetDescriptor(dmaChan[i]);

    /* DSTADDR is the last address, rather than first! */
    dmacDesc[i]->DSTADDR.reg =
        (uint32_t)adcBuffer[i] + (2 * VCT_TOTAL * OVERSAMPLING_RATIO);
    dmacDesc[i]->SRCADDR.reg = (uint32_t)&ADC->RESULT;
    /* Capture a full sample set before interrupt to start downsampling */
    dmacDesc[i]->BTCNT.reg   = (VCT_TOTAL * OVERSAMPLING_RATIO);
    dmacDesc[i]->BTCTRL.reg  = DMAC_BTCTRL_VALID
                              /* Raise interrupt on block transfer */
                              | DMAC_BTCTRL_BLOCKACT_INT |
                              DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_DSTINC |
                              DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_STEPSIZE_X1;

    dmacChannelConfigure(dmaChan[i], &dmacConfig);
    dmacEnableChannelInterrupt(dmaChan[i]);
  }

  /* Link the descriptors so sampling is continuous */
  dmacDesc[0]->DESCADDR.reg = (uint32_t)dmacDesc[1];
  dmacDesc[1]->DESCADDR.reg = (uint32_t)dmacDesc[0];
}

int16_t adcCorrectionGain(void) { return correctionGain; }
int16_t adcCorrectionOffset(void) { return correctionOffset; }
bool    adcCorrectionValid(void) { return correctionValid; }

void adcDMACStart(void) {
  dmacChannelEnable(DMA_CHAN_ADC0);

  /* Enable ADC; requires synchronisation (30.6.13) */
  if (!(ADC->CTRLA.reg & ADC_CTRLA_ENABLE)) {
    ADC->CTRLA.reg |= ADC_CTRLA_ENABLE;
    adcSync();
  }
}

void adcDMACStop(void) { dmacChannelDisable(DMA_CHAN_ADC0); }

void adcSetup(void) {
  extern uint8_t pinsADC[][2];

  for (unsigned int i = 0; pinsADC[i][0] != 0xFF; i++) {
    portPinMux(pinsADC[i][0], pinsADC[i][1], PORT_PMUX_PMUXE_B_Val);
  }

  /* APB bus clock is enabled by default (Table 15-1). Connect GCLK 3 */
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_ID(ADC_GCLK_ID) | GCLK_CLKCTRL_GEN(3u) | GCLK_CLKCTRL_CLKEN;

  /* Reset all the ADC registers */
  ADC->CTRLA.reg = ADC_CTRLA_SWRST;
  while (ADC->CTRLA.reg & ADC_CTRLA_SWRST)
    ;
  ADC->CTRLA.reg |= ADC_CTRLA_RUNSTDBY;

  ADC->CALIB.reg = (samdCalibration(CAL_ADC_BIAS) << 8u) |
                   samdCalibration(CAL_ADC_LINEARITY);

  /* Enable reference buffer and set to external VREF */
  ADC->REFCTRL.reg = ADC_REFCTRL_REFCOMP | ADC_REFCTRL_REFSEL_AREFA;

  adcCalibrate();

  /* Differential mode, /4 prescale of F_PERIPH, right aligned, enable
   * averaging. Requires synchronisation after write (33.6.15)
   */
  ADC->CTRLB.reg =
      ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_DIFFMODE | ADC_CTRLB_RESSEL_16BIT;
  adcSync();

  /* Conversion time is 3.5 us, target 6 us total conversion time
   * Setup 2.5 us conversion time: SAMPLEN = (2T * f_clk) - 1
   * (2 * 2.5E-6 * 2E6) - 1 = 9
   */
  ADC->SAMPCTRL.reg = 0x9u;

  /* 2x oversampling */
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_2;

  /* Input control - requires synchronisation (33.6.15) */
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS_PIN2 |
                       ADC_INPUTCTRL_MUXNEG_PIN0
                       /* INPUTSCAN is number of channels - 1 */
                       | ADC_INPUTCTRL_INPUTSCAN(VCT_TOTAL - 1u);
  adcSync();

  /* ADC is triggered by an event from TIMER_ADC with no CPU intervention */
  ADC->EVCTRL.reg = ADC_EVCTRL_STARTEI;

  adcConfigureDMAC();
}

int16_t adcSingleConversion(const unsigned int ch) {
  /* Save the scan and positive mux positions, do the conversion, and
   * restore the ADC state before returning the result.
   */
  int16_t      result      = 0;
  unsigned int enabledFlag = 0;

  const unsigned int inputCtrl = ADC->INPUTCTRL.reg;

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_PIN0 | ADC_INPUTCTRL_MUXPOS(ch);
  adcSync();

  ADC->INTFLAG.reg |= ADC_INTFLAG_RESRDY;

  if (!(ADC->CTRLA.reg & ADC_CTRLA_ENABLE)) {
    enabledFlag = 1u;
    ADC->CTRLA.reg |= ADC_CTRLA_ENABLE;
    adcSync();
  }

  ADC->SWTRIG.reg = ADC_SWTRIG_START;
  while (0 == (ADC->INTFLAG.reg & ADC_INTFLAG_RESRDY))
    ;
  result = ADC->RESULT.reg;

  ADC->INTFLAG.reg |= ADC_INTFLAG_RESRDY;
  ADC->INPUTCTRL.reg = inputCtrl;
  adcSync();

  /* Disable the ADC if it was enabled for a single conversion */
  if (enabledFlag) {
    ADC->CTRLA.reg &= ~ADC_CTRLA_ENABLE;
    adcSync();
  }

  return result;
}

static void adcSync(void) {
  while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY)
    ;
}
