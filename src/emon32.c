#include <stddef.h>
#include <string.h>

#include "emon32_samd.h"

#include "driver_ADC.h"
#include "driver_CLK.h"
#include "driver_EVSYS.h"
#include "driver_PORT.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "driver_WDT.h"

#include "configuration.h"
#include "data_pack.h"
#include "emon32.h"
#include "emon_CM.h"
#include "eeprom.h"
#include "periph_DS18B20.h"
#include "periph_rfm69.h"
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
static void     evtKiloHertz            ();
static uint32_t evtPending              (EVTSRC_t evt);
static void     ledStatusOn             ();
static void     ledProgOn               ();
static void     ledProgOff              ();
static void     nvmCumulativeConfigure  (eepromPktWL_t *pPkt);
static void     nvmLoadCumulative       (eepromPktWL_t *pPkt, Emon32Dataset_t *pData);
static void     nvmStoreCumulative      (eepromPktWL_t *pPkt, const Emon32Dataset_t *pData);
static void     processCumulative       (eepromPktWL_t *pPkt, const Emon32Dataset_t *pData, const unsigned int whDeltaStore);
static void     pulseConfigure          (const Emon32Config_t *pCfg);
static uint32_t tempSetup               ();
static uint32_t totalEnergy             (const Emon32Dataset_t *pData);
static void     ucSetup                 ();

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

    #if (NUM_PULSECOUNT > 0)
    for (unsigned int i = 0; i < NUM_PULSECOUNT; i++)
    {
        pDst->pulseCnt[i] = 0;

    }
    #endif
}


/*! @brief Add pulse counting information to the dataset to be sent
 *  @param [out] pDst : pointer to the data struct
 */
static void
datasetUpdate(Emon32Dataset_t *pDst)
{
    pDst->msgNum++;
    #if (NUM_PULSECOUNT > 0)
    for (unsigned int i = 0; i < NUM_PULSECOUNT; i++)
    {
        pDst->pulseCnt[i] = pulseGetCount(i);

    }
    #endif
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
        rfmInit(RF12_868MHz);
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
    ECMCfg_t *ecmCfg;
    ecmCfg = ecmGetConfig();

    ecmCfg->downsample      = DOWNSAMPLE_DSP;
    ecmCfg->reportCycles    = reportCycles;

    for (unsigned int i = 0; i < NUM_V; i ++)
    {
        ecmCfg->voltageCal[i] = pCfg->voltageCfg[i].voltageCal;
    }
    for (unsigned int i = 0; i < NUM_CT; i++)
    {
        uint32_t active = pCfg->ctActive & (1 << i);

        ecmCfg->ctCfg[i].phaseX = pCfg->ctCfg[i].phaseX;
        ecmCfg->ctCfg[i].phaseY = pCfg->ctCfg[i].phaseY;
        ecmCfg->ctCfg[i].ctCal  = pCfg->ctCfg[i].ctCal;
        ecmCfg->ctCfg[i].active =   active
                                  ? 1u
                                  : 0u;
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
evtKiloHertz()
{
    /* Feed watchdog - placed in the event handler to allow reset of stuck
     * processing rather than entering the interrupt reliably.
     */
    wdtFeed();

    /* Update the pulse counters, looking on different edges */
    pulseUpdate();
}


/*! @brief Check if an event source is active
 *  @param [in] : event source to check
 *  @return : 1 if pending, 0 otherwise
 */
static uint32_t
evtPending(EVTSRC_t evt)
{
    return   (evtPend & (1u << evt))
           ? 1u
           : 0;
}


/*! @brief Turn on the PROG LED */
static void
ledProgOn()
{
    /* For active LOW, change from PIN_DRV_SET to PIN_DRV_CLR */
    portPinDrv(GRP_LED_PROG, PIN_LED_PROG, PIN_DRV_SET);
}


/*! @brief Turn off the PROG LED */
static void
ledProgOff()
{
    /* For active LOW, change from PIN_DRV_SET to PIN_DRV_CLR */
    portPinDrv(GRP_LED_PROG, PIN_LED_PROG, PIN_DRV_SET);
}


/*! @brief Turn on the STATUS LED */
static void
ledStatusOn()
{
    /* For active LOW, change from PIN_DRV_SET to PIN_DRV_CLR */
    portPinDrv(GRP_LED_STATUS, PIN_LED_STATUS, PIN_DRV_SET);
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
nvmLoadCumulative(eepromPktWL_t *pPkt, Emon32Dataset_t *pData)
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
nvmStoreCumulative(eepromPktWL_t *pPkt, const Emon32Dataset_t *pData)
{
    int timerStatus;

    Emon32CumulativeSave_t data;
    pPkt->pData = &data;

    /* Copy data and calculate CRC */
    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        data.report.wattHour[idxCT] = pData->pECM->CT[idxCT].wattHour;
    }

    #if (NUM_PULSECOUNT > 0)
    for (unsigned int idxPulse = 0; idxPulse < NUM_PULSECOUNT; idxPulse++)
    {
        data.report.pulseCnt[0] = pData->pulseCnt[0];
    }
    #endif

    data.crc = calcCRC16_ccitt(&data.report, sizeof(Emon32Cumulative_t));

    eepromWriteWL(pPkt);
    /* If the timer is in use, then add a pending event to finish the EEPROM
     * save if required */
    timerStatus = timerDelayNB_us(EEPROM_WR_TIME, &eepromWriteCB);
    if (-1 == timerStatus)
    {
        emon32EventSet(EVT_EEPROM_TMR);
    }
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

    /* Catch overflow of energy. This corresponds to ~4 MWh(!), so
     * unlikely to happen, but handle safely.
     */
    energyOverflow = (latestWh < lastStoredWh);
    if (energyOverflow)
    {
        dbgPuts("> Cumulative energy overflowed counter!\r\n");
    }
    deltaWh = latestWh - lastStoredWh;
    if ((deltaWh > whDeltaStore) || energyOverflow)
    {
        nvmStoreCumulative(pPkt, pData);
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

    dbgPuts("> Setting up pulse counters... ");
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
    dbgPuts("Done!\r\n");
}


/*! @brief Setup the microcontoller. This function must be called first. An
 *         implementation must provide all the functions that are called.
 *         These can be empty if they are not used.
 */
static void
ucSetup()
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


/*! @brief Initialises the temperature sensors
 *  @return : number of temperature sensors found
 */
static uint32_t
tempSetup()
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
main()
{
    Emon32Config_t  e32Config               = {0};
    ECMDataset_t    ecmDataset              = {0};
    Emon32Dataset_t dataset                 = {0};
    eepromPktWL_t   nvmCumulative           = {0};
    RFMPkt_t        *rfmPkt                 = 0;
    char            txBuffer[TX_BUFFER_W]   = {0};
    PackedData_t    packedData              = {0};
    unsigned int    cyclesProcessed         = 0u;
    unsigned int    numTempSensors          = 0;
    unsigned int    tempCount               = 0;
    unsigned int    reportCycles            = 0;

    ucSetup     ();
    ledStatusOn ();

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
    nvmLoadCumulative       (&nvmCumulative, &dataset);
    NVIC_EnableIRQ          (SERCOM_UART_INTERACTIVE_IRQn);

    lastStoredWh = totalEnergy(&dataset);

    /* Set up data transmission interfaces and configuration */
    rfmPkt = dataTxConfigure(&e32Config);
    if (0 == rfmPkt)
    {
        dbgPuts("> Data TX on UART\r\n");
    }
    else
    {
        dbgPuts("> Data TX on RFM\r\n");
    }

    /* Set up pulse and temperature sensors, if present */
    pulseConfigure(&e32Config);

    /* Set up buffers for ADC data, configure energy processing, and start */
    ecmConfigure    (&e32Config, reportCycles);
    // adcStartDMAC    ((uint32_t)ecmDataBuffer());
    dbgPuts("> Start monitoring...\r\n");

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
                 */
                cyclesProcessed++;
                if ((reportCycles - cyclesProcessed) == e32Config.baseCfg.mainsFreq)
                {
                    emon32EventSet(EVT_TEMP_SAMPLE);
                }

                emon32EventClr(EVT_ECM_CYCLE_CMPL);
            }

            /* Start temperature sampling on OneWire interface. All sensors can
             * be triggered simultaneously.
             */
            if (evtPending(EVT_TEMP_SAMPLE))
            {
                if (numTempSensors > 0)
                {
                    /* REVISIT : report failed conversion here */
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
                    tempReadSample(TEMP_INTF_ONEWIRE, tempCount);

                    tempCount++;
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
             * configured channel. Echo on debug console.
             */
            if (evtPending(EVT_ECM_SET_CMPL))
            {
                unsigned int pktLength;

                ecmProcessSet(&ecmDataset);
                datasetUpdate(&dataset);

                pktLength = dataPackageESP_n(&dataset, txBuffer, TX_BUFFER_W);

                if (pktLength >= TX_BUFFER_W)
                {
                    dbgPuts("> TX buffer overflowed!\r\n");
                }

                if (0 == rfmPkt)
                {
                    uartPutsNonBlocking(DMA_CHAN_UART_DATA, txBuffer, pktLength);
                }
                else
                {
                    dataPackagePacked   (&dataset, &packedData);
                    rfmSend             (&packedData);
                }
                uartPutsNonBlocking(DMA_CHAN_UART_DBG, txBuffer, pktLength);

                /* If the energy used since the last storage is greater than the
                 * configured energy delta (baseCfg.whDeltaStore), then save the
                 * accumulated energy in NVM.
                 */
                processCumulative   (&nvmCumulative, &dataset,
                                     e32Config.baseCfg.whDeltaStore);

                emon32EventClr      (EVT_ECM_SET_CMPL);
            }

            /* If timer for EEPROM was not available, then retry until it is free */
            if (evtPending(EVT_EEPROM_TMR))
            {
                if (0 == timerDelayNB_us(EEPROM_WR_TIME, &eepromWriteCB))
                {
                    emon32EventClr(EVT_EEPROM_TMR);
                }
            }

            /* Configuration change / save. Set or clear the PROG LED
             * respectively.
             */
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

        }
        /* Enter WFI until woken by an interrupt */
        __WFI();
    };
}
