#include "emon32_samd.h"
#include "driver_ADC.h"
#include "driver_DMAC.h"
#include "driver_SAMD.h"
#include "driver_PORT.h"

#include "emon32.h"
#include "qfplib.h"
#include "printf.h"

static void adcCalibrate();


/*! @brief Load gain and offset registers for automatic compensation. Only
 *         available when using SAMD21 with sufficient ADC pins.
 * */
static void
adcCalibrate()
{
    /* Expected ADC values for 1/4 and 3/4 scale */
    /* REVISIT for 0.1 onwards, this is correct. The dev board is 1/3, 2/3
     * REVISIT seem to have outstanding factor of 2
     * const int32_t normalQuarter         = 0xC000;
     * const int32_t normalThreeQuarter    = 0x3FFF;
     */
    const int32_t normalQuarter         = -10923 / 2;
    const int32_t normalThreeQuarter    = 10922 / 2;

    /* Real values from ADC conversion */
    int16_t actualQuarter;
    int16_t actualThreeQuarter;

    /* Calibration values */
    int32_t offset_inter[2];
    int32_t offset;

    float   gain_fp;
    int     gain;

    /* Set up ADC for maximum sampling length and averaging.
     * This results in a 16 bit unsigned value in RESULT.
     */
    ADC->SAMPCTRL.reg   = 0x1Fu;
    ADC->AVGCTRL.reg    = ADC_AVGCTRL_SAMPLENUM_1024;
    ADC->CTRLB.reg      =   ADC_CTRLB_PRESCALER_DIV8
                          | ADC_CTRLB_DIFFMODE
                          | ADC_CTRLB_RESSEL_16BIT;
    while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY);

    /* Read 1/4 and 3/4 scale readings */
    ADC->CTRLA.reg |= ADC_CTRLA_ENABLE;
    while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY);

    ADC->INPUTCTRL.reg =   ADC_INPUTCTRL_MUXNEG_PIN0
                         | ADC_INPUTCTRL_MUXPOS_PIN18;
    ADC->INTFLAG.reg   |= ADC_INTFLAG_RESRDY;
    ADC->SWTRIG.reg    = ADC_SWTRIG_START;
    while (0 == (ADC->INTFLAG.reg & ADC_INTFLAG_RESRDY));
    actualQuarter = ADC->RESULT.reg;

    ADC->INPUTCTRL.reg =   ADC_INPUTCTRL_MUXNEG_PIN0
                         | ADC_INPUTCTRL_MUXPOS_PIN17;
    ADC->INTFLAG.reg   |= ADC_INTFLAG_RESRDY;
    ADC->SWTRIG.reg    = ADC_SWTRIG_START;
    while (0 == (ADC->INTFLAG.reg & ADC_INTFLAG_RESRDY));
    actualThreeQuarter = ADC->RESULT.reg;

    ADC->CTRLA.reg &= ~ADC_CTRLA_ENABLE;
    while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY);

    /* The corrected value is:
     * (Conversion + -OFFSET) * GAINCORR
     * Solve two equations in two unknowns for OFFSETCORR and GAINCORR.
     *  : G = (y0 - y1) / (y0' - y1') [y' is the actual conversion value]
     *  : C = y' - y / G
     * OFFSETCORR is a 12bit signed value (33.1.18 Offset Correction).
     * GAINCORR is 1 unsigned bit + 11 fractional bits (33.8.17 Gain Correction)
     *  : 1/2 < GAINCORR < 2.
     */
    gain_fp = qfp_fdiv  ((float)(normalQuarter - normalThreeQuarter),
                         (float)(actualQuarter - actualThreeQuarter));
    gain = qfp_float2fix(gain_fp, 11);

    offset_inter[0] = (int)qfp_fadd(0.5f,
                                    qfp_fdiv((float)normalQuarter, gain_fp)) - actualQuarter;
    offset_inter[1] = (int)qfp_fadd(0.5f,
                                    qfp_fdiv((float)normalThreeQuarter, gain_fp)) - actualThreeQuarter;
    offset = (offset_inter[0] + offset_inter[1]) / 2;

    /* Registers are 12 bit, shift 16 bit intermediate offset 4 */
    ADC->OFFSETCORR.reg = (int16_t)offset >> 4;
    ADC->GAINCORR.reg   = (int16_t)gain;
}


void
adcSetup()
{
    extern uint8_t          pinsADC[][2];
    DMACCfgCh_t             dmacConfig;
    volatile DmacDescriptor *dmacDesc;

    for (unsigned int i = 0; pinsADC[i][0] != 0xFF; i++)
    {
        portPinMux(pinsADC[i][0], pinsADC[i][1], PORT_PMUX_PMUXE_B_Val);
    }

    /* APB bus clock is enabled by default (Table 15-1). Connect GCLK 3 */
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(ADC_GCLK_ID)
                        | GCLK_CLKCTRL_GEN(3u)
                        | GCLK_CLKCTRL_CLKEN;

    /* Reset all the ADC registers */
    ADC->CTRLA.reg = ADC_CTRLA_SWRST;
    while (ADC->CTRLA.reg & ADC_CTRLA_SWRST);

    ADC->CALIB.reg =   (samdCalibration(CAL_ADC_BIAS) << 8u)
                     | samdCalibration(CAL_ADC_LINEARITY);

    /* Enable reference buffer and set to external VREF
     * REVISIT Unclear if the buffer is for external ref, or only internal
     */
    ADC->REFCTRL.reg =   ADC_REFCTRL_REFCOMP
                       | ADC_REFCTRL_REFSEL_AREFA;

    adcCalibrate();

    /* Differential mode, /8 prescale of F_PERIPH, right aligned, enable
     * averaging. Requires synchronisation after write (30.6.13)
     */
    ADC->CTRLB.reg =   ADC_CTRLB_PRESCALER_DIV8
                     | ADC_CTRLB_DIFFMODE
                     | ADC_CTRLB_CORREN
                     | ADC_CTRLB_RESSEL_12BIT;
    while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY);

    /* Setup 3 us conversion time - allows up to ~333 ksps @ 1 MHz CLK
     * SAMPLEN = (2T / T_clk) - 1 = 5
     */
    ADC->SAMPCTRL.reg = 0x5u;

    /* Setup 4X oversampling. This can be used for up to 16 ADC channels at
     * 4.8 kHz sampling (4.8 * 16 * 4 = 307 kHz). The intermediate 14 bit
     * result is right shifted by 2 to provide the final 12 bit result.
     */
    ADC->AVGCTRL.reg =   ADC_AVGCTRL_SAMPLENUM(2)
                       | ADC_AVGCTRL_ADJRES(2);

    /* Input control - requires synchronisation (30.6.13) */
    ADC->INPUTCTRL.reg =   ADC_INPUTCTRL_MUXPOS_PIN2
                         | ADC_INPUTCTRL_MUXNEG_PIN0
                         /* INPUTSCAN is number of channels - 1 */
                         | ADC_INPUTCTRL_INPUTSCAN(VCT_TOTAL - 1u);
    while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY);

    /* ADC is triggered by an event from TC1 with no CPU intervention */
    ADC->EVCTRL.reg = ADC_EVCTRL_STARTEI;

    /* DMA channel */
    dmacDesc = dmacGetDescriptor(DMA_CHAN_ADC);
    dmacDesc->DESCADDR.reg  = 0u;
    dmacDesc->SRCADDR.reg   = (uint32_t)&ADC->RESULT;
    /* Capture a full sample set before interrupt to start downsampling */
    dmacDesc->BTCNT.reg     = (VCT_TOTAL) * OVERSAMPLING_RATIO;
    dmacDesc->BTCTRL.reg    =   DMAC_BTCTRL_VALID
                              | DMAC_BTCTRL_BLOCKACT_NOACT
                              | DMAC_BTCTRL_BEATSIZE_HWORD
                              | DMAC_BTCTRL_DSTINC
                              | DMAC_BTCTRL_STEPSIZE_X1;

    dmacConfig.ctrlb =    DMAC_CHCTRLB_LVL(0u)
                        | DMAC_CHCTRLB_TRIGSRC(ADC_DMAC_ID_RESRDY)
                        | DMAC_CHCTRLB_TRIGACT_BEAT;
    dmacChannelConfigure        (DMA_CHAN_ADC, &dmacConfig);
    dmacEnableChannelInterrupt  (DMA_CHAN_ADC);

    /* Enable requires synchronisation (30.6.13) */
    ADC->CTRLA.reg |= ADC_CTRLA_ENABLE;
    while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY);
}


int16_t
adcSingleConversion(const unsigned int ch)
{
    /* Save the scan and positive mux positions, do the conversion, and
     * restore the ADC state before returning the result.
     */
    int16_t result = 0;
    const unsigned int inputCtrl = ADC->INPUTCTRL.reg;

    ADC->INPUTCTRL.reg =   ADC_INPUTCTRL_MUXNEG_PIN0
                         | ADC_INPUTCTRL_MUXPOS(ch);
    while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY);

    ADC->INTFLAG.reg |= ADC_INTFLAG_RESRDY;
    ADC->SWTRIG.reg  = ADC_SWTRIG_START;
    while (0 == (ADC->INTFLAG.reg & ADC_INTFLAG_RESRDY));
    result = ADC->RESULT.reg;

    ADC->INTFLAG.reg |= ADC_INTFLAG_RESRDY;
    ADC->INPUTCTRL.reg = inputCtrl;
    while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY);

    return result;
}


void
adcStartDMAC(uint32_t buf)
{
    volatile DmacDescriptor *dmaDesc = dmacGetDescriptor(DMA_CHAN_ADC);
    dmaDesc->BTCTRL.reg     |= DMAC_BTCTRL_VALID;
    dmaDesc->DSTADDR.reg    = buf;

    dmacStartTransfer(DMA_CHAN_ADC);
}
