#include <stddef.h>
#include <string.h>

#include "emon32_samd.h"

#include "driver_ADC.h"
#include "driver_CLK.h"
#include "driver_EIC.h"
#include "driver_EVSYS.h"
#include "driver_PORT.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "driver_USB.h"
#include "driver_WDT.h"

#include "configuration.h"
#include "dataPack.h"
#include "eeprom.h"
#include "emon32.h"
#include "emon32_assert.h"
#include "emon_CM.h"
#include "periph_DS18B20.h"
#include "periph_SSD1306.h"
#include "periph_rfm69.h"
#include "pulse.h"
#include "temperature.h"
#include "ui.h"
#include "util.h"

#include "printf.h"

typedef struct TransmitOpt_ {
  bool    json;
  bool    useRFM;
  bool    logSerial;
  uint8_t node;
} TransmitOpt_t;

/*************************************
 * Persistent state variables
 *************************************/

static volatile uint32_t evtPend;
static unsigned int      lastStoredWh;
AssertInfo_t             g_assert_info;

/*************************************
 * Static function prototypes
 *************************************/

static void cumulativeNVMLoad(Emon32Cumulative_t *pPkt, Emon32Dataset_t *pData);
static void cumulativeNVMStore(Emon32Cumulative_t    *pPkt,
                               const Emon32Dataset_t *pData);
static void cumulativeProcess(Emon32Cumulative_t    *pPkt,
                              const Emon32Dataset_t *pData,
                              const unsigned int     whDeltaStore);
static void datasetAddPulse(Emon32Dataset_t *pDst);
static RFMOpt_t *dataTxConfigure(const Emon32Config_t *pCfg);
static void      ecmConfigure(const Emon32Config_t *pCfg);
static void      ecmDmaCallback(void);
static void      evtKiloHertz(void);
static uint32_t  evtPending(EVTSRC_t evt);
static void      pulseConfigure(const Emon32Config_t *pCfg);
void             putchar_(char c);
static void      putsDbgNonBlocking(const char *const s, uint16_t len);
static void      ssd1306Setup(void);
static uint32_t  tempSetup(void);
static uint32_t  totalEnergy(const Emon32Dataset_t *pData);
static void      transmitData(const Emon32Dataset_t *pSrc,
                              const TransmitOpt_t   *pOpt);
static void      ucSetup(void);

/*************************************
 * Functions
 *************************************/

/*! @brief Load cumulative energy and pulse values
 *  @param [in] pEEPROM : pointer to EEPROM configuration
 *  @param [in] pData : pointer to current dataset
 */
static void cumulativeNVMLoad(Emon32Cumulative_t *pPkt,
                              Emon32Dataset_t    *pData) {
  EMON32_ASSERT(pPkt);
  EMON32_ASSERT(pData);
  eepromWLStatus_t wlStatus = EEPROM_WL_OK;

  eepromWLReset(sizeof(*pPkt));
  wlStatus = eepromReadWL(pPkt);

  if ((EEPROM_WL_OK == wlStatus) || (EEPROM_WL_CRC_BAD == wlStatus)) {
    if (EEPROM_WL_CRC_BAD == wlStatus) {
      dbgPuts("> Accumulator possibly corrupt. Reverted to older value.\r\n");
    }

    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++) {
      pData->pECM->CT[idxCT].wattHour = pPkt->wattHour[idxCT];
    }

    for (unsigned int idxPulse = 0; idxPulse < NUM_OPA; idxPulse++) {
      pData->pulseCnt[idxPulse] = pPkt->pulseCnt[idxPulse];
    }
  } else {
    dbgPuts("> All accumulators corrupt, all reset.\r\n");
  }
}

/*! @brief Store cumulative energy and pulse values
 *  @param [in] pRes : pointer to cumulative values
 */
static void cumulativeNVMStore(Emon32Cumulative_t    *pPkt,
                               const Emon32Dataset_t *pData) {
  EMON32_ASSERT(pPkt);
  EMON32_ASSERT(pData);

  for (int idxCT = 0; idxCT < NUM_CT; idxCT++) {
    pPkt->wattHour[idxCT] = pData->pECM->CT[idxCT].wattHour;
  }

  for (int idxPulse = 0; idxPulse < NUM_OPA; idxPulse++) {
    pPkt->pulseCnt[0] = pData->pulseCnt[0];
  }

  (void)eepromWriteWL(pPkt);
}

/*! @brief Calculate the cumulative energy consumption and store if the delta
 *         since last storage is greater than a configurable threshold
 *  @param [in] : pPkt : pointer to an NVM packet
 *  @param [in] : pData : pointer to the current dataset
 */
static void cumulativeProcess(Emon32Cumulative_t    *pPkt,
                              const Emon32Dataset_t *pData,
                              const unsigned int     whDeltaStore) {
  EMON32_ASSERT(pPkt);
  EMON32_ASSERT(pData);

  int      energyOverflow;
  uint32_t latestWh;
  uint32_t deltaWh;

  /* Store cumulative values if over threshold */
  latestWh = totalEnergy(pData);

  /* Catch overflow of energy. This corresponds to ~4 MWh(!), so unlikely to
   * but handle safely.
   */
  energyOverflow = (latestWh < lastStoredWh);
  deltaWh        = latestWh - lastStoredWh;
  if ((deltaWh > whDeltaStore) || energyOverflow) {
    // cumulativeNVMStore(pPkt, pData);
    lastStoredWh = latestWh;
  }
}

/*! @brief Add pulse counting information to the dataset to be sent
 *  @param [out] pDst : pointer to the data struct
 */
static void datasetAddPulse(Emon32Dataset_t *pDst) {
  EMON32_ASSERT(pDst);

  pDst->msgNum++;
  for (unsigned int i = 0; i < NUM_OPA; i++) {
    pDst->pulseCnt[i] = pulseGetCount(i);
  }
}

/*! @brief Configure the data transmission output.
 *  @param [in] pCfg : pointer to the configuration struct
 *  @return : pointer to an RFM packet if using RFM, 0 if not.
 */
static RFMOpt_t *dataTxConfigure(const Emon32Config_t *pCfg) {
  EMON32_ASSERT(pCfg);

  RFMOpt_t *rfmOpt = 0;
  if (DATATX_RFM69 == (TxType_t)pCfg->dataTxCfg.txType) {
    rfmOpt            = rfmGetHandle();
    rfmOpt->node      = pCfg->baseCfg.nodeID;
    rfmOpt->grp       = pCfg->baseCfg.dataGrp; /* Fixed for OpenEnergyMonitor */
    rfmOpt->rf_pwr    = pCfg->dataTxCfg.rfmPwr;
    rfmOpt->threshold = 0u;
    rfmOpt->timeout   = 1000u;
    rfmOpt->n         = 23u;
    if (sercomExtIntfEnabled()) {
      if (rfmInit((RFM_Freq_t)pCfg->dataTxCfg.rfmFreq)) {
        rfmSetAESKey("89txbe4p8aik5kt3"); /* Default OEM AES key */
      }
    }
  }
  return rfmOpt;
}

void dbgPuts(const char *s) {
  EMON32_ASSERT(s);

  if (usbCDCIsConnected()) {
    usbCDCPutsBlocking(s);
  } else {
    uartPutsBlocking(SERCOM_UART_DBG, s);
  }
}

/*! @brief Configure the continuous energy monitoring system
 *  @param [in] pCfg : pointer to the configuration struct
 */
void ecmConfigure(const Emon32Config_t *pCfg) {
  /* Makes the continuous monitoring setup agnostic to the data strcuture
   * used for storage, and avoids any awkward alignment from packing.
   */
  EMON32_ASSERT(pCfg);

  extern const int_fast8_t ainRemap[NUM_CT];

  ECMCfg_t *ecmCfg = ecmConfigGet();

  ecmCfg->downsample      = DOWNSAMPLE_DSP;
  ecmCfg->reportCycles    = pCfg->baseCfg.reportCycles;
  ecmCfg->mainsFreq       = pCfg->baseCfg.mainsFreq;
  ecmCfg->samplePeriod    = timerADCPeriod();
  ecmCfg->timeMicros      = &timerMicros;
  ecmCfg->timeMicrosDelta = &timerMicrosDelta;

  if (adcCorrectionValid()) {
    ecmCfg->correction.valid  = true;
    ecmCfg->correction.gain   = adcCorrectionGain();
    ecmCfg->correction.offset = adcCorrectionOffset();
  } else {
    ecmCfg->correction.valid = false;
  }

  for (unsigned int i = 0; i < NUM_V; i++) {
    ecmCfg->vCfg[i].voltageCalRaw = pCfg->voltageCfg[i].voltageCal;
    ecmCfg->vCfg[i].vActive       = pCfg->voltageCfg[i].vActive;
  }

  for (unsigned int i = 0; i < NUM_CT; i++) {
    ecmCfg->ctCfg[i].phCal    = pCfg->ctCfg[i].phase;
    ecmCfg->ctCfg[i].ctCalRaw = pCfg->ctCfg[i].ctCal;
    ecmCfg->ctCfg[i].active   = pCfg->ctCfg[i].ctActive;
    ecmCfg->ctCfg[i].vChan1   = pCfg->ctCfg[i].vChan1;
    ecmCfg->ctCfg[i].vChan2   = pCfg->ctCfg[i].vChan2;
  }

  for (int i = 0; i < NUM_CT; i++) {
    ecmCfg->mapCTLog[i] = ainRemap[i];
  }

  ecmConfigInit();
}

void ecmDmaCallback(void) {
  ECM_STATUS_t injectStatus;
  ecmDataBufferSwap();
  injectStatus = ecmInjectSample();
  switch (injectStatus) {
  case ECM_REPORT_COMPLETE:
    emon32EventSet(EVT_ECM_SET_CMPL);
    break;
  case ECM_PEND_1S:
    emon32EventSet(EVT_ECM_PEND_1S);
    break;
  default:
    break;
  }
}

void emon32EventClr(const EVTSRC_t evt) {
  /* Disable interrupts during RMW update of event status */
  uint32_t evtDecode = ~(1u << evt);
  __disable_irq();
  evtPend &= evtDecode;
  __enable_irq();
}

void emon32EventSet(const EVTSRC_t evt) {
  /* Disable interrupts during RMW update of event status */
  uint32_t evtDecode = (1u << evt);
  __disable_irq();
  evtPend |= evtDecode;
  __enable_irq();
}

/*! @brief This function is called when the 1 ms timer fires.
 *         Latency is not guaranteed, so only non-timing critical things
 *         should be done here (UI update, watchdog etc)
 */
static void evtKiloHertz(void) {
  int                      extEnabled;
  uint32_t                 msDelta;
  static volatile uint32_t msLast = 0;
  int                      ndisable_ext;
  static unsigned int      statLedOff_time = 0;

  /* Feed watchdog - placed in the event handler to allow reset of stuck
   * processing rather than entering the interrupt reliably.
   */
  wdtFeed();

  /* Update the pulse counters, looking on different edges */
  pulseUpdate();

  /* Check for nDISABLE_EXT_INTF */
  /* REVISIT in board 0.2, this will be handled by EIC */
  extEnabled   = sercomExtIntfEnabled();
  ndisable_ext = portPinValue(GRP_nDISABLE_EXT, PIN_nDISABLE_EXT);
  if (extEnabled && !ndisable_ext) {
    sercomExtIntfDisable();
  } else if (!extEnabled && ndisable_ext) {
    sercomExtIntfEnable();
  }

  /* When there is a TX to the outside world, blink the STATUS LED for
   * time TX_INDICATE_T to show there is activity.
   */
  if (portPinValue(GRP_LED_STATUS, PIN_LED_STATUS) && (0 == statLedOff_time)) {
    statLedOff_time = timerMillis();
  }
  if (timerMillisDelta(statLedOff_time) > TX_INDICATE_T) {
    statLedOff_time = 0;
    uiLedOn(LED_STATUS);
  }

  /* Track milliseconds to indicate uptime */
  msDelta = timerMillisDelta(msLast);
  if (msDelta >= 1000) {
    timerUptimeIncr();
    msLast = timerMillis();
    /* Account for any jitter in the 1 ms tick */
    if (msDelta > 1000) {
      msDelta -= 1000;
      msLast -= msDelta;
    }
  }
}

/*! @brief Check if an event source is active
 *  @param [in] : event source to check
 *  @return : 1 if pending, 0 otherwise
 */
static uint32_t evtPending(EVTSRC_t evt) {
  return (evtPend & (1u << evt)) ? 1u : 0;
}

/*! @brief Configure any pulse counter interfaces
 *  @param [in] pCfg : pointer to the configuration struct
 */
static void pulseConfigure(const Emon32Config_t *pCfg) {
  EMON32_ASSERT(pCfg);

  uint8_t pinsPulse[][2] = {{GRP_OPA, PIN_OPA1}, {GRP_OPA, PIN_OPA2}};

  for (unsigned int i = 0; i < NUM_OPA; i++) {
    PulseCfg_t *pulseCfg = pulseGetCfg(i);

    if (0 != pulseCfg) {
      pulseCfg->edge    = (PulseEdge_t)pCfg->pulseCfg[i].func;
      pulseCfg->grp     = pinsPulse[i][0];
      pulseCfg->pin     = pinsPulse[i][1];
      pulseCfg->periods = pCfg->pulseCfg[i].period;
      pulseCfg->active  = pCfg->pulseCfg[i].pulseActive;

      pulseInit(i);
    }
  }
}

/*! @brief Allows the printf function to print to the debug console. If the USB
 *         CDC is connected, characters should be routed there.
 */
void putchar_(char c) {
  if (usbCDCIsConnected()) {
    usbCDCTxChar(c);
  } else {
    uartPutcBlocking(SERCOM_UART_DBG, c);
  }
}

static void putsDbgNonBlocking(const char *const s, uint16_t len) {
  if (usbCDCIsConnected()) {
    usbCDCPutsBlocking(s);
  } else {
    uartPutsNonBlocking(DMA_CHAN_UART_DBG, s, len);
  }
}

/*! @brief Setup the SSD1306 display, if present. Display a basic message */
static void ssd1306Setup(void) {
  SSD1306_Status_t s;
  PosXY_t          a = {44, 0};
  s                  = ssd1306Init(SERCOM_I2CM_EXT);
  if (SSD1306_SUCCESS == s) {
    ssd1306SetPosition(a);
    ssd1306DrawString("emonPi3");
    ssd1306DisplayUpdate();
  }
}

/*! @brief Initialises the temperature sensors
 *  @return : number of temperature sensors found
 */
static uint32_t tempSetup(void) {
  unsigned int   numTempSensors = 0;
  DS18B20_conf_t dsCfg          = {0};

  dsCfg.grp       = GRP_OPA;
  dsCfg.pin       = PIN_OPA1;
  dsCfg.t_wait_us = 5;

  numTempSensors = tempInitSensors(TEMP_INTF_ONEWIRE, &dsCfg);

  return numTempSensors;
}

/*! @brief Total energy across all CTs
 *  @param [in] pData : pointer to data setup
 *  @return : sum of Wh for all CTs
 */
static uint32_t totalEnergy(const Emon32Dataset_t *pData) {
  EMON32_ASSERT(pData);

  uint32_t totalEnergy = 0;
  for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++) {
    totalEnergy += pData->pECM->CT[idxCT].wattHour;
  }
  return totalEnergy;
}

static void transmitData(const Emon32Dataset_t *pSrc,
                         const TransmitOpt_t   *pOpt) {
  char txBuffer[TX_BUFFER_W] = {0};

  int nSerial = dataPackSerial(pSrc, txBuffer, TX_BUFFER_W, pOpt->json);

  if (pOpt->useRFM) {
    if (sercomExtIntfEnabled()) {
      int_fast8_t nPacked = dataPackPacked(pSrc, rfmGetBuffer(), PACKED_LOWER);
      rfmSetAddress(pOpt->node);
      if (RFM_SUCCESS == rfmSendBuffer(nPacked)) {
        nPacked = dataPackPacked(pSrc, rfmGetBuffer(), PACKED_UPPER);
        rfmSetAddress(pOpt->node + 1);
        rfmSendBuffer(nPacked);
      }
    }

    if (pOpt->logSerial) {
      putsDbgNonBlocking(txBuffer, nSerial);
    }
  } else {
    putsDbgNonBlocking(txBuffer, nSerial);
  }
}

/*! @brief Setup the microcontoller. This function must be called first. An
 *         implementation must provide all the functions that are called.
 *         These can be empty if they are not used.
 */
static void ucSetup(void) {
  clkSetup();
  timerSetup();
  portSetup();
  eicSetup();
  dmacSetup();
  sercomSetup();
  adcSetup();
  evsysSetup();
  usbSetup();
  // wdtSetup    (WDT_PER_4K);
}

int main(void) {

  Emon32Config_t    *pConfig        = 0;
  Emon32Dataset_t    dataset        = {0};
  unsigned int       numTempSensors = 0;
  Emon32Cumulative_t nvmCumulative  = {0};
  RFMOpt_t          *rfmOpt         = 0;
  unsigned int       tempCount      = 0;

  ucSetup();
  uiLedOn(LED_STATUS);
  ssd1306Setup();

  /* Load stored values (configuration and accumulated energy) from
   * non-volatile memory (NVM). If the NVM has not been used before then
   * store default configuration and 0 energy accumulator area.
   */
  dbgPuts("> Reading configuration and accumulators from NVM...\r\n");
  configLoadFromNVM();

  pConfig = configGetConfig();
  cumulativeNVMLoad(&nvmCumulative, &dataset);

  lastStoredWh = totalEnergy(&dataset);

  /* Set up data transmission interfaces and configuration */
  rfmOpt = dataTxConfigure(pConfig);

  /* Set up pulse and temperature sensors, if present */
  pulseConfigure(pConfig);
  numTempSensors         = tempSetup();
  dataset.numTempSensors = numTempSensors;

  /* Set up buffers for ADC data, configure energy processing, and start */
  ecmConfigure(pConfig);
  dmacCallbackBufferFill(&ecmDmaCallback);
  ecmFlush();
  adcDMACStart();
  dbgPuts("> Start monitoring...\r\n");

  timerDelay_ms(1000);
  configFirmwareBoardInfo();

  for (;;) {
    /* While there is an event pending (may be set while another is
     * handled), keep looping. Enter sleep (WFI) when done.
     */
    while (0 != evtPend) {
      /* 1 ms timer flag */
      if (evtPending(EVT_TICK_1kHz)) {
        evtKiloHertz();
        emon32EventClr(EVT_TICK_1kHz);
      }

      /* Configuration request to clear all accumulator values (energy and pulse
       * count). The NVM is overwritten with 0s and the index of the next
       * read/write is reset. Clear the running counters in the main loop, any
       * residual energy in the dataset, and all pulse counters.
       */
      if (evtPending(EVT_CLEAR_ACCUM)) {
        lastStoredWh = 0;
        /* REVISIT : may need to make this asynchronous as it will take 240 ms
         * (worst case) to clear the whole EEPROM area.
         */
        eepromWLClear();
        eepromWLReset(sizeof(nvmCumulative));
        ecmClearResidual();
        for (int i = 0; i < NUM_OPA; i++) {
          pulseSetCount(0, i);
        }
        emon32EventClr(EVT_CLEAR_ACCUM);
      }

      /* There has been a trigger request externally; the CM buffers will be
       * swapped on the next cycle. If there has been sufficient time between
       * the last temperature sample, start a temperature sample as well.
       */
      if (evtPending(EVT_ECM_TRIG)) {
        (void)tempStartSample(TEMP_INTF_ONEWIRE, tempCount);
        ecmProcessSetTrigger();
        emon32EventClr(EVT_ECM_TRIG);
      }

      /* Trigger a temperature sample 1 s before the report is due. */
      if (evtPending(EVT_ECM_PEND_1S)) {
        (void)tempStartSample(TEMP_INTF_ONEWIRE, tempCount);
        emon32EventClr(EVT_ECM_PEND_1S);
      }

      /* Readout has been requested, trigger a temperature read. */
      if (evtPending(EVT_ECM_SET_CMPL)) {
        emon32EventSet(EVT_TEMP_READ);
        emon32EventClr(EVT_ECM_SET_CMPL);
      }

      /* Read back samples from each DS18B20 present. This is a blocking
       * routine, and is lower priority than processing a cycle, so read
       * one sensor on each loop. When all are complete, send the data out
       * through the configured interface.
       */
      if (evtPending(EVT_TEMP_READ)) {
        if (numTempSensors > 0) {
          TempRead_t tempValue = tempReadSample(TEMP_INTF_ONEWIRE, tempCount);

          if (TEMP_OK == tempValue.status) {
            dataset.temp[tempCount] = tempValue.result;
          }

          tempCount++;
          if (tempCount == numTempSensors) {
            emon32EventSet(EVT_PROCESS_DATASET);
            emon32EventClr(EVT_TEMP_READ);
            tempCount = 0;
          }
        } else {
          emon32EventSet(EVT_PROCESS_DATASET);
          emon32EventClr(EVT_TEMP_READ);
        }
      }

      /* Report period elapsed; generate, pack, and send through the
       * configured channel. Echo on debug console, if enabled.
       */
      if (evtPending(EVT_PROCESS_DATASET)) {
        TransmitOpt_t opt;
        opt.json      = pConfig->baseCfg.useJson;
        opt.useRFM    = (0 != rfmOpt);
        opt.logSerial = pConfig->baseCfg.logToSerial;
        opt.node      = pConfig->baseCfg.nodeID;

        dataset.pECM = ecmProcessSet();
        datasetAddPulse(&dataset);
        transmitData(&dataset, &opt);

        /* If the energy used since the last storage is greater than the
         * configured energy delta (baseCfg.whDeltaStore), then save the
         * accumulated energy in NVM.
         */
        cumulativeProcess(&nvmCumulative, &dataset,
                          pConfig->baseCfg.whDeltaStore);

        /* Blink the STATUS LED, and clear the event. */
        uiLedOff(LED_STATUS);
        emon32EventClr(EVT_PROCESS_DATASET);
      }

      /* Configuration:
       *   - Process command
       *   - Change (set PROG LED)
       *   - Save (clear PROG LED)
       */
      if (evtPending(EVT_PROCESS_CMD)) {
        configProcessCmd();
        emon32EventClr(EVT_PROCESS_CMD);
      }
      if (evtPending(EVT_CONFIG_CHANGED)) {
        uiLedOn(LED_PROG);
        emon32EventClr(EVT_CONFIG_CHANGED);
      }
      if (evtPending(EVT_CONFIG_SAVED)) {
        uiLedOff(LED_PROG);
        emon32EventClr(EVT_CONFIG_SAVED);
      }

      if (evtPending(EVT_SAFE_RESET_REQ)) {
        /* REVISIT store the cumulative value safely here. Currently
         * uses a non-blocking timer callback to complete this, need
         * to ensure that everything is written successfully.
         */

        NVIC_SystemReset();
      }
    }

    /* REVISIT add invariant and behaviour assertions */

    /* Enter WFI until woken by an interrupt */
    __WFI();
  };
}
