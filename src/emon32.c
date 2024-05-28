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
#include "driver_WDT.h"

#include "configuration.h"
#include "dataPack.h"
#include "emon32.h"
#include "emon32_assert.h"
#include "emon_CM.h"
#include "eeprom.h"
#include "periph_DS18B20.h"
#include "periph_rfm69.h"
#include "periph_SSD1306.h"
#include "pulse.h"
#include "temperature.h"
#include "ui.h"
#include "util.h"

#include "printf.h"

/*************************************
 * Persistent state variables
 *************************************/

static volatile uint32_t    evtPend;
static unsigned int         lastStoredWh;
AssertInfo_t                g_assert_info;

/*************************************
 * Static function prototypes
 *************************************/

static void     cumulativeNVMLoad       (eepromPktWL_t *pPkt, Emon32Dataset_t *pData);
static void     cumulativeNVMStore      (eepromPktWL_t *pPkt, const Emon32Dataset_t *pData);
static void     cumulativeProcess       (eepromPktWL_t *pPkt, const Emon32Dataset_t *pData, const unsigned int whDeltaStore);
static void     datasetUpdate           (Emon32Dataset_t *pDst);
static RFMPkt_t *dataTxConfigure        (const Emon32Config_t *pCfg);
static void     ecmConfigure            (const Emon32Config_t *pCfg);
static void     evtKiloHertz            (void);
static uint32_t evtPending              (EVTSRC_t evt);
static void     pulseConfigure          (const Emon32Config_t *pCfg);
       void     putchar_                (char c);
static void     ssd1306Setup            (void);
static uint32_t tempSetup               (void);
static uint32_t totalEnergy             (const Emon32Dataset_t *pData);
static void     ucSetup                 (void);

/*************************************
 * Functions
 *************************************/

/*! @brief Load cumulative energy and pulse values
 *  @param [in] pEEPROM : pointer to EEPROM configuration
 *  @param [in] pData : pointer to current dataset
 */
static void
cumulativeNVMLoad(eepromPktWL_t *pPkt, Emon32Dataset_t *pData)
{
    Emon32CumulativeSave_t data;

    pPkt->pData         = &data;
    pPkt->dataSize      = sizeof(data);
    pPkt->idxNextWrite  = -1;
    memset      (&data, 0, sizeof(data));
    eepromReadWL(pPkt);

    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        pData->pECM->CT[idxCT].wattHour = data.report.wattHour[idxCT];
    }

    for (unsigned int idxPulse = 0; idxPulse < NUM_PULSECOUNT; idxPulse ++)
    {
        pData->pulseCnt[idxPulse] = data.report.pulseCnt[idxPulse];
    }
}


/*! @brief Store cumulative energy and pulse values
 *  @param [in] pRes : pointer to cumulative values
 */
static void
cumulativeNVMStore(eepromPktWL_t *pPkt, const Emon32Dataset_t *pData)
{
    Emon32CumulativeSave_t data;
    pPkt->pData = &data;

    /* Copy data and calculate CRC */
    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        data.report.wattHour[idxCT] = pData->pECM->CT[idxCT].wattHour;
    }

    for (unsigned int idxPulse = 0; idxPulse < NUM_PULSECOUNT; idxPulse++)
    {
        data.report.pulseCnt[0] = pData->pulseCnt[0];
    }

    data.crc = calcCRC16_ccitt(&data.report, sizeof(Emon32Cumulative_t));

    (void)eepromWriteWL(pPkt);
    emon32EventSet(EVT_EEPROM_STORE);
}


/*! @brief Calculate the cumulative energy consumption and store if the delta
 *         since last storage is greater than a configurable threshold
 *  @param [in] : pPkt : pointer to an NVM packet
 *  @param [in] : pData : pointer to the current dataset
 */
static void
cumulativeProcess(eepromPktWL_t *pPkt, const Emon32Dataset_t *pData, const unsigned int whDeltaStore)
{
    int         energyOverflow;
    uint32_t    latestWh;
    uint32_t    deltaWh;

    /* Store cumulative values if over threshold */
    latestWh = totalEnergy(pData);

    /* Catch overflow of energy. This corresponds to ~4 MWh(!), so unlikely to
     * but handle safely.
     */
    energyOverflow = (latestWh < lastStoredWh);
    deltaWh = latestWh - lastStoredWh;
    if ((deltaWh > whDeltaStore) || energyOverflow)
    {
        cumulativeNVMStore(pPkt, pData);
        lastStoredWh = latestWh;
    }
}


/*! @brief Add pulse counting information to the dataset to be sent
 *  @param [out] pDst : pointer to the data struct
 */
static void
datasetUpdate(Emon32Dataset_t *pDst)
{
    pDst->msgNum++;
    for (unsigned int i = 0; i < NUM_PULSECOUNT; i++)
    {
        pDst->pulseCnt[i] = pulseGetCount(i);

    }
}


/*! @brief Configure the data transmission output.
 *  @param [in] pCfg : pointer to the configuration struct
 *  @return : pointer to an RFM packet if using RFM, 0 if not.
 */
static RFMPkt_t *
dataTxConfigure(const Emon32Config_t *pCfg)
{
    RFMPkt_t *rfmPkt = 0;
    if (DATATX_RFM69 == pCfg->dataTxCfg.txType)
    {
        rfmPkt              = rfmGetHandle();
        rfmPkt->node        = pCfg->baseCfg.nodeID;
        rfmPkt->grp         = pCfg->baseCfg.dataGrp;    /* Fixed for OpenEnergyMonitor */
        rfmPkt->rf_pwr      = pCfg->dataTxCfg.rfmPwr;
        rfmPkt->threshold   = 0u;
        rfmPkt->timeout     = 1000u;
        rfmPkt->n           = 23u;
        if (sercomExtIntfEnabled())
        {
            rfmInit((RFM_Freq_t)pCfg->dataTxCfg.rfmFreq);
        }
    }
    /* REVISIT : dedicated UARTs for debug and data. */
    else
    {
        UART_Cfg_t uart_data_cfg;
        uart_data_cfg.sercom    = SERCOM_UART_DATA;
        uart_data_cfg.baud      = UART_DATA_BAUD;
        uart_data_cfg.apbc_mask = SERCOM_UART_DATA_APBCMASK;
        uart_data_cfg.gclk_id   = SERCOM_UART_DATA_GCLK_ID;
        uart_data_cfg.gclk_gen  = 3u;
        uart_data_cfg.pad_tx    = UART_DATA_PAD_TX;
        uart_data_cfg.pad_rx    = UART_DATA_PAD_RX;
        uart_data_cfg.port_grp  = GRP_SERCOM_UART_DATA;
        uart_data_cfg.pin_tx    = PIN_UART_DATA_TX;
        uart_data_cfg.pin_rx    = PIN_UART_DATA_RX;
        uart_data_cfg.pmux      = PMUX_UART_DATA;
        /*
         * sercomSetupUART(&uart_data_cfg);
         */
        (void)uart_data_cfg;
    }
    return rfmPkt;
}


void
dbgPuts(const char *s)
{
    uartPutsBlocking(SERCOM_UART_DBG, s);
}


/*! @brief Configure the continuous energy monitoring system
 *  @param [in] pCfg : pointer to the configuration struct
 */
void
ecmConfigure(const Emon32Config_t *pCfg)
{
    /* Makes the continuous monitoring setup agnostic to the data strcuture
     * used for storage, and avoids any awkward alignment from packing.
     */
    ECMCfg_t    *ecmCfg;
    PhaseXY_t   phaseXY;

    ecmCfg = ecmConfigGet();

    ecmCfg->downsample      = DOWNSAMPLE_DSP;
    ecmCfg->reportCycles    = pCfg->baseCfg.reportCycles;
    ecmCfg->mainsFreq       = pCfg->baseCfg.mainsFreq;
    ecmCfg->sampleRateHz    = (SAMPLE_RATE / OVERSAMPLING_RATIO);
    if (ZEROX_HW_SPT)
    {
        ecmCfg->zx_hw_stat  = &eicZeroXStat;
        ecmCfg->zx_hw_clr   = &eicZeroXClr;
    }

    if (PERF_ENABLED)
    {
        ecmCfg->timeMicros      = &timerMicros;
        ecmCfg->timeMicrosDelta = &timerMicrosDelta;
    }

    ecmConfigInit();

    for (unsigned int i = 0; i < NUM_V; i ++)
    {
        ecmCfg->voltageCal[i] = ecmCalibrationCalculate(pCfg->voltageCfg[i].voltageCal);
    }
    /* REVISIT : currently only doing single phase measurement */
    ecmCfg->vActive[0] = true;
    ecmCfg->vActive[1] = false;
    ecmCfg->vActive[2] = false;

    for (unsigned int i = 0; i < NUM_CT; i++)
    {
        uint32_t active = pCfg->ctActive & (1 << i);

        phaseXY = ecmPhaseCalculate(pCfg->ctCfg[i].phase, i);

        ecmCfg->ctCfg[i].phaseX = phaseXY.phaseX;
        ecmCfg->ctCfg[i].phaseY = phaseXY.phaseY;
        ecmCfg->ctCfg[i].ctCal  = ecmCalibrationCalculate(pCfg->ctCfg[i].ctCal);
        ecmCfg->ctCfg[i].active = active ? 1u : 0u;
        ecmCfg->ctCfg[i].vChan  = pCfg->ctCfg[i].vChan;
    }
}


void
emon32EventClr(const EVTSRC_t evt)
{
    /* Disable interrupts during RMW update of event status */
    uint32_t evtDecode = ~(1u << evt);
    __disable_irq();
    evtPend &= evtDecode;
    __enable_irq();
}


void
emon32EventSet(const EVTSRC_t evt)
{
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
static void
evtKiloHertz(void)
{
    int                         extEnabled;
    uint32_t                    msDelta;
    static volatile uint32_t    msLast = 0;
    int                         ndisable_ext;
    static unsigned int         statLedOff_time = 0;

    /* Feed watchdog - placed in the event handler to allow reset of stuck
     * processing rather than entering the interrupt reliably.
     */
    wdtFeed();

    /* Update the pulse counters, looking on different edges */
    pulseUpdate();

    /* Check for nDISABLE_EXT_INTF */
    /* REVISIT in board 0.2, this will be handled by EIC*/
    extEnabled      = sercomExtIntfEnabled();
    ndisable_ext    = portPinValue(GRP_nDISABLE_EXT, PIN_nDISABLE_EXT);
    if (extEnabled && !ndisable_ext)
    {
        sercomExtIntfDisable();
    }
    else if (!extEnabled && ndisable_ext)
    {
        sercomExtIntfEnable();
    }

    /* When there is a TX to the outside world, blink the STATUS LED for
     * time TX_INDICATE_T to show there is activity.
     */
    if (portPinValue(GRP_LED_STATUS, PIN_LED_STATUS) && (0 == statLedOff_time))
    {
        statLedOff_time = timerMillis();
    }
    if (timerMillisDelta(statLedOff_time) > TX_INDICATE_T)
    {
        statLedOff_time = 0;
        uiLedOn(LED_STATUS);
    }

    /* Track milliseconds to indicate uptime */
    msDelta = timerMillisDelta(msLast);
    if (msDelta >= 1000)
    {
        timerUptimeIncr();
        msLast = timerMillis();
        /* Account for any jitter in the 1 ms tick */
        if (msDelta > 1000)
        {
            msDelta -= 1000;
            msLast -= msDelta;
        }
    }
}


/*! @brief Check if an event source is active
 *  @param [in] : event source to check
 *  @return : 1 if pending, 0 otherwise
 */
static uint32_t
evtPending(EVTSRC_t evt)
{
    return (evtPend & (1u << evt)) ? 1u : 0;
}


/*! @brief Configure any pulse counter interfaces
 *  @param [in] pCfg : pointer to the configuration struct
 */
static void
pulseConfigure(const Emon32Config_t *pCfg)
{
    uint8_t pinsPulse[][2] = {
        {GRP_PULSE, PIN_PULSE1},
        {GRP_PULSE, PIN_PULSE2}
    };

    for (unsigned int i = 0; i < NUM_PULSECOUNT; i++)
    {
        PulseCfg_t *pulseCfg = pulseGetCfg(i);

        if (0 != pulseCfg)
        {
            pulseCfg->edge      = (PulseEdge_t)pCfg->pulseCfg[i].edge;
            pulseCfg->grp       = pinsPulse[i][0];
            pulseCfg->pin       = pinsPulse[i][1];
            pulseCfg->periods   = pCfg->pulseCfg[i].period;
            pulseCfg->active    =   (0 == (pCfg->pulseActive & (1 << i)))
                                  ? 0
                                  : 1;
            pulseInit(i);
        }
    }
}


/*! @brief Allows the printf function to print to the debug console */
void
putchar_(char c)
{
    uartPutcBlocking(SERCOM_UART_DBG, c);
}


/*! @brief Setup the SSD1306 display, if present. Display a basic message */
static void
ssd1306Setup(void)
{
    SSD1306_Status_t s;
    PosXY_t a = {44,0};
    s = ssd1306Init(SERCOM_I2CM_EXT);
    if (SSD1306_SUCCESS == s)
    {
        ssd1306SetPosition(a);
        ssd1306DrawString("emonPi3");
        ssd1306DisplayUpdate();
    }
}

/*! @brief Initialises the temperature sensors
 *  @return : number of temperature sensors found
 */
static uint32_t
tempSetup(void)
{
    unsigned int    numTempSensors  = 0;
    DS18B20_conf_t  dsCfg           = {0};

    dsCfg.grp       = GRP_ONEWIRE;
    dsCfg.pin       = PIN_ONEWIRE;
    dsCfg.t_wait_us = 5;

    numTempSensors = tempInitSensors(TEMP_INTF_ONEWIRE, &dsCfg);

    return numTempSensors;
}


/*! @brief Total energy across all CTs
 *  @param [in] pData : pointer to data setup
 *  @return : sum of Wh for all CTs
 */
static uint32_t
totalEnergy(const Emon32Dataset_t *pData)
{
    uint32_t totalEnergy = 0;
    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        totalEnergy += pData->pECM->CT[idxCT].wattHour;
    }
    return totalEnergy;
}


/*! @brief Setup the microcontoller. This function must be called first. An
 *         implementation must provide all the functions that are called.
 *         These can be empty if they are not used.
 */
static void
ucSetup(void)
{
    clkSetup    ();
    timerSetup  ();
    portSetup   ();
    dmacSetup   ();
    sercomSetup ();
    adcSetup    ();
    evsysSetup  ();
    // wdtSetup    (WDT_PER_4K);
}


int
main(void)
{
    unsigned int    cyclesProcessed         = 0u;
    Emon32Config_t  e32Config               = {0};
    ECMDataset_t    ecmDataset              = {0};
    eepromWrStatus_t eepromWrStatus         = EEPROM_WR_PEND;
    Emon32Dataset_t dataset                 = {0};
    unsigned int    numTempSensors          = 0;
    eepromPktWL_t   nvmCumulative           = {0};
    PackedData_t    packedData              = {0};
    RFMPkt_t        *rfmPkt                 = 0;
    unsigned int    tempCount               = 0;
    int16_t         tempValue               = 0;
    unsigned int    timeSinceTrigger        = 0;
    char            txBuffer[TX_BUFFER_W]   = {0};

    ucSetup     ();
    uiLedOn     (LED_STATUS);
    ssd1306Setup();

    /* Setup DMAC for non-blocking UART (this is optional, unlike ADC) */
    uartConfigureDMA    ();
    uartInterruptEnable (SERCOM_UART_DBG, SERCOM_USART_INTENSET_RXC);
    uartInterruptEnable (SERCOM_UART_DBG, SERCOM_USART_INTENSET_ERROR);
    NVIC_EnableIRQ      (SERCOM_UART_INTERACTIVE_IRQn);

    configFirmwareBoardInfo();

    /* Load stored values (configuration and accumulated energy) from
     * non-volatile memory (NVM). If the NVM has not been used before then
     * store default configuration and 0 energy accumulator area.
     * REVISIT add check that firmware version matches stored config.
     */
    dbgPuts                 ("> Reading configuration and accumulators from NVM...\r\n");
    configLoadFromNVM       (&e32Config);
    e32Config.baseCfg.reportCycles = configTimeToCycles(e32Config.baseCfg.reportTime,
                                                        e32Config.baseCfg.mainsFreq);

    dataset.pECM = &ecmDataset;
    cumulativeNVMLoad       (&nvmCumulative, &dataset);

    lastStoredWh = totalEnergy(&dataset);

    /* Set up data transmission interfaces and configuration */
    rfmPkt = dataTxConfigure(&e32Config);

    /* Set up pulse and temperature sensors, if present */
    pulseConfigure(&e32Config);
    numTempSensors = tempSetup();
    dataset.numTempSensors = numTempSensors;

    /* Set up buffers for ADC data, configure energy processing, and start */
    ecmConfigure    (&e32Config);
    ecmFlush        ();
    adcDMACStart    ();
    dbgPuts         ("> Start monitoring...\r\n");

    for (;;)
    {
        /* While there is an event pending (may be set while another is
         * handled), keep looping. Enter sleep (WFI) when done.
         */
        while(0 != evtPend)
        {
            /* 1 ms timer flag */
            if (evtPending(EVT_TICK_1kHz))
            {
                evtKiloHertz    ();
                emon32EventClr  (EVT_TICK_1kHz);
            }

            /* There has been a trigger request externally. This needs to come
             * > 1s before the report is nominally due to allow slow
             * temperature sensors to complete before processing the data. If
             * there are no temperature sensors immediately trigger the
             * processing, otherwise wait until the conversion time has
             * elapsed.
             */
            if (evtPending(EVT_PROCESS_DATASET))
            {
                if ((e32Config.baseCfg.reportCycles - cyclesProcessed) > e32Config.baseCfg.mainsFreq)
                {
                    timeSinceTrigger = timerMillis();
                    emon32EventSet(EVT_TEMP_SAMPLE);
                }

                if (  (timerMillisDelta(timeSinceTrigger) >= TEMP_CONVERSION_T)
                    || (numTempSensors == 0))
                {
                    ecmProcessSetTrigger();
                    emon32EventClr(EVT_PROCESS_DATASET);
                }
            }

            /* A full mains cycle has completed. Calculate power/energy. When
             * all samples are complete, trigger a read of any temperature
             * sensors.
             */
            if (evtPending(EVT_ECM_CYCLE_CMPL))
            {
                if (ECM_REPORT_COMPLETE == ecmProcessCycle())
                {
                    cyclesProcessed = 0;
                    emon32EventSet(EVT_TEMP_READ);
                }

                /* Trigger a temperature sample 1 s before the report is due.
                 * In 12bit mode (default), DS18B20 takes 750 ms to acquire.
                 * If there is a process trigger >1 s before the nominal report
                 * due time, then start a sample as well.
                 */
                cyclesProcessed++;
                if (((e32Config.baseCfg.reportCycles - cyclesProcessed) == e32Config.baseCfg.mainsFreq))
                {
                    emon32EventSet(EVT_TEMP_SAMPLE);
                }

                emon32EventClr(EVT_ECM_CYCLE_CMPL);
            }

            /* Start temperature sampling on OneWire interface. All sensors are
             * triggered simultaneously.
             */
            if (evtPending(EVT_TEMP_SAMPLE))
            {
                if (numTempSensors > 0)
                {
                    /* REVISIT : report failed conversion here? */
                    (void)tempStartSample(TEMP_INTF_ONEWIRE, tempCount);
                }
                emon32EventClr(EVT_TEMP_SAMPLE);
            }

            /* Read back samples from each DS18B20 present. This is a blocking
             * routine, and is lower priority than processing a cycle, so read
             * one sensor on each loop.
             */
            if (evtPending(EVT_TEMP_READ))
            {
                if (numTempSensors > 0)
                {
                    tempValue = tempReadSample(TEMP_INTF_ONEWIRE, tempCount);
                    dataset.temp[tempCount++] = tempAsFloat(TEMP_INTF_ONEWIRE,
                                                            tempValue);
                    if (tempCount == numTempSensors)
                    {
                        emon32EventSet(EVT_ECM_SET_CMPL);
                        emon32EventClr(EVT_TEMP_READ);
                        tempCount = 0;
                    }
                }
                else
                {
                    emon32EventSet(EVT_ECM_SET_CMPL);
                    emon32EventClr(EVT_TEMP_READ);
                }
            }

            /* Report period elapsed; generate, pack, and send through the
             * configured channel. Echo on debug console, if enabled.
             */
            if (evtPending(EVT_ECM_SET_CMPL))
            {
                int pktLength;

                ecmProcessSet(&ecmDataset);
                datasetUpdate(&dataset);

                pktLength = dataPackSerial(&dataset, txBuffer, TX_BUFFER_W,
                                           e32Config.baseCfg.useJson);

                if (0 == rfmPkt)
                {
                    uartPutsNonBlocking(DMA_CHAN_UART_DATA, txBuffer, pktLength);
                }
                else
                {
                    dataPackPacked(&dataset, &packedData);
                    if (sercomExtIntfEnabled())
                    {
                        /* Try to send in "clean" air. If failed, retry on
                         * next loop. Should not reach RFM_FAILED at all.
                         */
                        RFMSend_t res = rfmSendReady(5u);
                        if (RFM_NO_INIT == res)
                        {
                            rfmInit((RFM_Freq_t)e32Config.dataTxCfg.rfmFreq);
                        }
                        else if (RFM_SUCCESS == res)
                        {
                            rfmSend(&packedData);
                        }
                    }
                }

                if (e32Config.baseCfg.logToSerial)
                {
                    uartPutsNonBlocking(DMA_CHAN_UART_DBG, txBuffer, pktLength);
                }

                /* If the energy used since the last storage is greater than the
                 * configured energy delta (baseCfg.whDeltaStore), then save the
                 * accumulated energy in NVM.
                 */
                cumulativeProcess   (&nvmCumulative, &dataset,
                                     e32Config.baseCfg.whDeltaStore);

                /* Blink the STATUS LED, and clear the event. */
                uiLedOff        (LED_STATUS);
                emon32EventClr  (EVT_ECM_SET_CMPL);
            }

            if (evtPending(EVT_EEPROM_STORE))
            {
                eepromWrStatus = eepromWriteContinue();
                if (EEPROM_WR_COMPLETE == eepromWrStatus)
                {
                    emon32EventClr(EVT_EEPROM_STORE);
                }
            }

            /* Configuration:
             *   - Process command
             *   - Change (set PROG LED)
             *   - Save (clear PROG LED)
             */
            if (evtPending(EVT_PROCESS_CMD))
            {
                configProcessCmd();
                emon32EventClr(EVT_PROCESS_CMD);
            }
            if (evtPending(EVT_CONFIG_CHANGED))
            {
                uiLedOn(LED_PROG);
                emon32EventClr(EVT_CONFIG_CHANGED);
            }
            if (evtPending(EVT_CONFIG_SAVED))
            {
                uiLedOff(LED_PROG);
                emon32EventClr(EVT_CONFIG_SAVED);
            }

            if (evtPending(EVT_SAFE_RESET_REQ))
            {
                /* REVISIT store the cumulative value safely here. Currently
                 * uses a non-blocking timer callback to complete this, need
                 * to ensure that everything is written successfully.
                 */

                NVIC_SystemReset();
            }
        }
        /* Enter WFI until woken by an interrupt */
        __WFI();
    };
}
