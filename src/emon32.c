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
#include "emon_CM.h"
#include "eeprom.h"
#include "periph_DS18B20.h"
#include "periph_rfm69.h"
#include "periph_SSD1306.h"
#include "pulse.h"
#include "temperature.h"
#include "util.h"

#include "printf.h"


/*************************************
 * Persistent state variables
 *************************************/

static volatile uint32_t    evtPend;
static unsigned int         lastStoredWh;

/*************************************
 * Static function prototypes
 *************************************/

static void     datasetInit             (Emon32Dataset_t *pDst, ECMDataset_t *pECM);
static void     datasetUpdate           (Emon32Dataset_t *pDst);
static RFMPkt_t *dataTxConfigure        (const Emon32Config_t *pCfg);
static void     ecmConfigure            (const Emon32Config_t *pCfg, const unsigned int reportCycles);
static void     evtKiloHertz            (void);
static uint32_t evtPending              (EVTSRC_t evt);
static void     ledProgOff              (void);
static void     ledProgOn               (void);
static void     ledStatusOff            (void);
static void     ledStatusOn             (void);
static void     nvmCumulativeConfigure  (eepromPktWL_t *pPkt);
static void     nvmCumulativeLoad       (eepromPktWL_t *pPkt, Emon32Dataset_t *pData);
static void     nvmCumulativeStore      (eepromPktWL_t *pPkt, const Emon32Dataset_t *pData);
static void     processCumulative       (eepromPktWL_t *pPkt, const Emon32Dataset_t *pData, const unsigned int whDeltaStore);
static void     pulseConfigure          (const Emon32Config_t *pCfg);
static void     ssd1306Setup            (void);
static uint32_t tempSetup               (void);
static uint32_t totalEnergy             (const Emon32Dataset_t *pData);
static void     ucSetup                 (void);

/*************************************
 * Functions
 *************************************/


/*! @brief Initialise the data set
 *  @param [out] pDst : pointer to the data struct
 *  @param [in] pECM : pointer to the emon Continuous Monitoring struct
 */
static void
datasetInit(Emon32Dataset_t *pDst, ECMDataset_t *pECM)
{
    pDst->msgNum = 0;
    pDst->pECM = pECM;
    for (unsigned int i = 0; i < NUM_PULSECOUNT; i++)
    {
        pDst->pulseCnt[i] = 0;
    }
    for (unsigned int i = 0; i < TEMP_MAX_ONEWIRE; i++)
    {
        pDst->temp[i] = 0;
    }
    pDst->numTempSensors = 0;
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
    if (DATATX_RFM69 == pCfg->baseCfg.dataTx)
    {
        sercomSetupSPI();
        rfmPkt              = rfmGetHandle();
        rfmPkt->node        = pCfg->baseCfg.nodeID;
        rfmPkt->grp         = 210u; /* Fixed for OpenEnergyMonitor */
        rfmPkt->rf_pwr      = 0u;
        rfmPkt->threshold   = 0u;
        rfmPkt->timeout     = 1000u;
        rfmPkt->n           = 23u;
        if (sercomExtIntfEnabled())
        {
            rfmInit(RF12_868MHz);
        }
    }
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
        /* REVISIT : for emon32-Pi2 v0.1, configure the data UART. Not present in
         * 0.1dev
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


/* This allows the printf function to be used to print to the debug console */
void
putchar_(char c)
{
    uartPutcBlocking(SERCOM_UART_DBG, c);
}


/*! @brief Configure the continuous energy monitoring system
 *  @param [in] pCfg : pointer to the configuration struct
 */
void
ecmConfigure(const Emon32Config_t *pCfg, const unsigned int reportCycles)
{
    /* Makes the continuous monitoring setup agnostic to the data strcuture
     * used for storage, and avoids any awkward alignment from packing.
     */
    ECMCfg_t    *ecmCfg;
    PhaseXY_t   phaseXY;

    ecmCfg = ecmGetConfig();

    ecmCfg->downsample      = DOWNSAMPLE_DSP;
    ecmCfg->reportCycles    = reportCycles;
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

    for (unsigned int i = 0; i < NUM_V; i ++)
    {
        ecmCfg->voltageCal[i] = pCfg->voltageCfg[i].voltageCal;
    }
    for (unsigned int i = 0; i < NUM_CT; i++)
    {
        uint32_t active = pCfg->ctActive & (1 << i);

        phaseXY = ecmPhaseCalculate(pCfg->ctCfg[i].phase);

        ecmCfg->ctCfg[i].phaseX = phaseXY.phaseX;
        ecmCfg->ctCfg[i].phaseY = phaseXY.phaseY;
        ecmCfg->ctCfg[i].ctCal  = pCfg->ctCfg[i].ctCal;
        ecmCfg->ctCfg[i].active = active ? 1u : 0u;
        ecmCfg->ctCfg[i].vChan  = pCfg->ctCfg[i].vChan;
    }
}


/*! @brief Set a pending event
 *  @brief [in] evt : event to be set
 */
void
emon32EventSet(const EVTSRC_t evt)
{
    /* Disable interrupts during RMW update of event status */
    uint32_t evtDecode = (1u << evt);
    __disable_irq();
    evtPend |= evtDecode;
    __enable_irq();
}


/*! @brief Clear a pending event
 *  @brief [in] evt : event to be cleared
 */
void
emon32EventClr(const EVTSRC_t evt)
{
    /* Disable interrupts during RMW update of event status */
    uint32_t evtDecode = ~(1u << evt);
    __disable_irq();
    evtPend &= evtDecode;
    __enable_irq();
}


/*! @brief This function is called when the 1 ms timer fires.
 *         Latency is not guaranteed, so only non-timing critical things
 *         should be done here (UI update, watchdog etc)
 */
static void
evtKiloHertz(void)
{
    static volatile uint32_t    msLast          = 0;
    uint32_t                    msDelta;
    int                         extEnabled;
    int                         ndisable_ext;
    static unsigned int         statLedLast     = 0;
    static unsigned int         statLedTrack    = 0;
    unsigned int                statLed;

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
     * time TX_INDICATE_T to show there is activity. Find a falling edge of
     * the LED to start the timer.
     */
    statLed = portPinValue(GRP_LED_STATUS, PIN_LED_STATUS);
    if ((statLed != statLedLast) && !statLed)
    {
        statLedTrack = timerMillis();
    }
    if (statLedTrack)
    {
        if (timerMillisDelta(statLedLast) >= TX_INDICATE_T)
        {
            ledStatusOn();
            statLedTrack = 0;
        }
    }
    statLedLast = statLed;

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


/*! @brief Turn off the PROG LED */
static void
ledProgOff(void)
{
    /* For active LOW, change from PIN_DRV_CLR to PIN_DRV_SET */
    portPinDrv(GRP_LED_PROG, PIN_LED_PROG, PIN_DRV_SET);
}


/*! @brief Turn on the PROG LED */
static void
ledProgOn(void)
{
    /* For active LOW, change from PIN_DRV_SET to PIN_DRV_CLR */
    portPinDrv(GRP_LED_PROG, PIN_LED_PROG, PIN_DRV_CLR);
}


/*! @brief Turn off the STATUS LED */
static void
ledStatusOff(void)
{
    /* For active LOW, change from PIN_DRV_CLR to PIN_DRV_SET */
    portPinDrv(GRP_LED_STATUS, PIN_LED_STATUS, PIN_DRV_SET);
}


/*! @brief Turn on the STATUS LED */
static void
ledStatusOn(void)
{
    /* For active LOW, change from PIN_DRV_SET to PIN_DRV_CLR */
    portPinDrv(GRP_LED_STATUS, PIN_LED_STATUS, PIN_DRV_CLR);
}


/*! @brief Initialise the NVM packet with default values
 *  @param [out] : pPkt : pointer to the NVM packet
 */
static void
nvmCumulativeConfigure(eepromPktWL_t *pPkt)
{
    pPkt->dataSize      = sizeof(Emon32CumulativeSave_t);
    pPkt->idxNextWrite  = -1;
}


/*! @brief Load cumulative energy and pulse values
 *  @param [in] pEEPROM : pointer to EEPROM configuration
 *  @param [in] pData : pointer to current dataset
 */
static void
nvmCumulativeLoad(eepromPktWL_t *pPkt, Emon32Dataset_t *pData)
{
    Emon32CumulativeSave_t data;
    pPkt->pData = &data;

    memset      (&data, 0, sizeof(Emon32CumulativeSave_t));
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
nvmCumulativeStore(eepromPktWL_t *pPkt, const Emon32Dataset_t *pData)
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
processCumulative(eepromPktWL_t *pPkt, const Emon32Dataset_t *pData, const unsigned int whDeltaStore)
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
    if (energyOverflow)
    {
        dbgPuts("> Cumulative energy overflowed counter!\r\n");
    }
    deltaWh = latestWh - lastStoredWh;
    if ((deltaWh > whDeltaStore) || energyOverflow)
    {
        nvmCumulativeStore(pPkt, pData);
        lastStoredWh = latestWh;
    }
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
    unsigned int    reportCycles            = 0;
    RFMPkt_t        *rfmPkt                 = 0;
    unsigned int    tempCount               = 0;
    int16_t         tempValue               = 0;
    unsigned int    timeSinceTrigger        = 0;
    char            txBuffer[TX_BUFFER_W]   = {0};

    ucSetup     ();
    ledStatusOn ();
    ssd1306Setup();

    /* Setup DMAC for non-blocking UART (this is optional, unlike ADC) */
    uartConfigureDMA    ();
    uartInterruptEnable (SERCOM_UART_DBG, SERCOM_USART_INTENSET_RXC);
    uartInterruptEnable (SERCOM_UART_DBG, SERCOM_USART_INTENSET_ERROR);

    configFirmwareBoardInfo();

    /* Load stored values (configuration and accumulated energy) from
     * non-volatile memory (NVM). If the NVM has not been used before then
     * store default configuration and 0 energy accumulator area.
     * REVISIT add check that firmware version matches stored config.
     */
    dbgPuts                 ("> Reading configuration and accumulators from NVM...\r\n");
    configLoadFromNVM       (&e32Config);
    reportCycles = configTimeToCycles(e32Config.baseCfg.reportTime,
                                      e32Config.baseCfg.mainsFreq);

    datasetInit             (&dataset, &ecmDataset);
    nvmCumulativeConfigure  (&nvmCumulative);
    nvmCumulativeLoad       (&nvmCumulative, &dataset);
    NVIC_EnableIRQ          (SERCOM_UART_INTERACTIVE_IRQn);

    lastStoredWh = totalEnergy(&dataset);

    /* Set up data transmission interfaces and configuration */
    rfmPkt = dataTxConfigure(&e32Config);

    /* Set up pulse and temperature sensors, if present */
    pulseConfigure(&e32Config);
    numTempSensors = tempSetup();
    dataset.numTempSensors = numTempSensors;

    /* Set up buffers for ADC data, configure energy processing, and start */
    ecmConfigure    (&e32Config, reportCycles);
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
                if ((reportCycles - cyclesProcessed) > e32Config.baseCfg.mainsFreq)
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
                if (((reportCycles - cyclesProcessed) == e32Config.baseCfg.mainsFreq))
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
                    dataset.temp[tempCount++] = tempAsFloat(TEMP_INTF_ONEWIRE, tempValue);
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
                unsigned int pktLength;

                ecmProcessSet(&ecmDataset);
                datasetUpdate(&dataset);

                pktLength = dataPackageESP_n(&dataset, txBuffer, TX_BUFFER_W);

                if (0 == rfmPkt)
                {
                    uartPutsNonBlocking(DMA_CHAN_UART_DATA, txBuffer, pktLength);
                }
                else
                {
                    dataPackagePacked(&dataset, &packedData);
                    if (sercomExtIntfEnabled())
                    {
                        /* REVISIT handle failures (timeout, no init) */
                        if (RFM_SUCCESS == rfmSendReady(5u))
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
                processCumulative   (&nvmCumulative, &dataset,
                                     e32Config.baseCfg.whDeltaStore);

                /* Blink the STATUS LED, and clear the event. */
                ledStatusOff        ();
                emon32EventClr      (EVT_ECM_SET_CMPL);
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
                ledProgOn();
                emon32EventClr(EVT_CONFIG_CHANGED);
            }
            if (evtPending(EVT_CONFIG_SAVED))
            {
                ledProgOff();
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
