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
#include "periph_rfm69.h"
#include "pulse.h"
#include "temperature.h"

/*************************************
 * Persistent state variables
 *************************************/

static volatile uint32_t    evtPend;
static volatile EmonState_t emonState = EMON_STATE_IDLE;

static unsigned int         lastStoredWh;

/*************************************
 * Static function prototypes
 *************************************/

static void     datasetInit             (Emon32Dataset_t *pDst, ECMDataset_t *pECM);
static void     datasetUpdate           (Emon32Dataset_t *pDst);
static RFMPkt_t *dataTxConfigure        (const Emon32Config_t *pCfg);
static void     dbgPutBoard             ();
static void     ecmConfigure            (const Emon32Config_t *pCfg);
static void     evtCycleComplete        ();
static void     evtKiloHertz            ();
static uint32_t evtPending              (EVTSRC_t evt);
static void     ledStatusOn             ();
static void     ledStatusToggle         ();
static void     nvmCumulativeConfigure  (eepromPktWL_t *pPkt);
static void     nvmLoadConfiguration    (Emon32Config_t *pCfg);
static void     nvmLoadCumulative       (eepromPktWL_t *pPkt, Emon32Dataset_t *pData);
static void     nvmStoreCumulative      (eepromPktWL_t *pPkt, const Emon32Dataset_t *pData);
static void     processCumulative       (eepromPktWL_t *pPkt, const Emon32Dataset_t *pData, const unsigned int whDeltaStore);
static void     pulseConfigure          (const Emon32Config_t *pCfg);
static void     setupMicrocontroller    ();
static uint32_t totalEnergy             (const Emon32Dataset_t *pData);

/*************************************
 * Functions
 *************************************/

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
        sercomSetupUART(&uart_data_cfg);
    }
    return rfmPkt;
}

static void
dbgPutBoard()
{
    char        wr_buf[8];
    const int   board_id = BOARD_ID;

    dbgPuts("\033c== Energy Monitor 32 ==\r\n\r\n");

    dbgPuts("Board:    ");
    switch (board_id)
    {
        case (BOARD_ID_LC):
            dbgPuts("emon32 Low Cost");
            break;
        case (BOARD_ID_STANDARD):
            dbgPuts("emon32 Standard");
            break;
        case (BOARD_ID_EMONPI):
            dbgPuts("emon32-Pi2");
            break;
        default:
            dbgPuts("Unknown");
    }
    dbgPuts("\r\n");

    dbgPuts("Firmware: ");
    (void)utilItoa(wr_buf, VERSION_FW_MAJ, ITOA_BASE10);
    dbgPuts(wr_buf);
    uartPutcBlocking(SERCOM_UART_DBG, '.');
    (void)utilItoa(wr_buf, VERSION_FW_MIN, ITOA_BASE10);
    dbgPuts(wr_buf);
    dbgPuts("\r\n\r\n");
}

void
dbgPuts(const char *s)
{
    uartPutsBlocking(SERCOM_UART_DBG, s);
}

void
ecmConfigure(const Emon32Config_t *pCfg)
{
    ECMCfg_t *ecmCfg;
    ecmCfg = ecmGetConfig();

    ecmCfg->reportCycles = pCfg->baseCfg.reportCycles;
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

void
emon32EventSet(EVTSRC_t evt)
{
    /* Disable interrupts during RMW update of event status */
    uint32_t evtDecode = (1u << evt);
    __disable_irq();
    evtPend |= evtDecode;
    __enable_irq();
}

void
emon32EventClr(EVTSRC_t evt)
{
    /* Disable interrupts during RMW update of event status */
    uint32_t evtDecode = ~(1u << evt);
    __disable_irq();
    evtPend &= evtDecode;
    __enable_irq();
}

EmonState_t
emon32StateGet()
{
    return emonState;
}

void
emon32StateSet(EmonState_t state)
{
    emonState = state;
}

/*! @brief This function is called when the 10 ms timer overflows (SYSTICK).
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

static void
ledStatusOn()
{
    /* For active LOW, change from PIN_DRV_SET to PIN_DRV_CLR */
    portPinDrv(GRP_LED_STATUS, PIN_LED_STATUS, PIN_DRV_SET);
}

static void
ledStatusToggle()
{
    portPinDrv(GRP_LED_STATUS, PIN_LED_STATUS, PIN_DRV_TGL);
}

static void
nvmCumulativeConfigure(eepromPktWL_t *pPkt)
{
    pPkt->addrBase      = EEPROM_WL_OFFSET;
    pPkt->blkCnt        = EEPROM_WL_NUM_BLK;
    pPkt->dataSize      = sizeof(Emon32CumulativeSave_t);
    pPkt->idxNextWrite  = -1;
}

/*! @brief This function handles loading of configuration data
 *  @param [in] pCfg : pointer to the configuration struct
 */
static void
nvmLoadConfiguration(Emon32Config_t *pCfg)
{
    unsigned int    systickCnt  = 0u;
    unsigned int    seconds     = 3u;

    configLoadFromNVM(pCfg);

    /* Wait for 3 s, if a key is pressed then enter interactive configuration */
    dbgPuts("> Press any key to enter configuration ");
    while (systickCnt < 400)
    {
        if (uartInterruptStatus(SERCOM_UART_DBG) & SERCOM_USART_INTFLAG_RXC)
        {
            emon32StateSet(EMON_STATE_CONFIG);
            (void)uartGetc(SERCOM_UART_DBG);
            configEnter(pCfg);
            break;
        }

        if (evtPend & (1u << EVT_SYSTICK_100Hz))
        {
            wdtFeed();
            emon32EventClr(EVT_SYSTICK_100Hz);
            systickCnt++;

            /* Countdown every second, tick every 200 ms to debug UART */
            if (0 == (systickCnt % 100))
            {
                ledStatusToggle();
                uartPutcBlocking(SERCOM_UART_DBG, '0' + seconds);
                seconds--;
            }
            else if (0 == (systickCnt % 20))
            {
                ledStatusToggle();
                uartPutcBlocking(SERCOM_UART_DBG, '.');
            }
        }
    }
    dbgPuts("\r\n");
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

    memset(&data, 0, sizeof(Emon32CumulativeSave_t));
    eepromReadWL(pPkt);

    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        pData->pECM->CT[idxCT].wattHour = data.report.wattHour[idxCT];
    }

    #if (NUM_PULSECOUNT > 0)
    for (unsigned int idxPulse = 0; idxPulse < NUM_PULSECOUNT; idxPulse ++)
    {
        pData->pulseCnt[idxPulse] = data.report.pulseCnt[idxPulse];
    }
    #endif
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
    energyOverflow = (latestWh < lastStoredWh) ? 1u : 0;
    if (0 != energyOverflow)
    {
        dbgPuts("\r\n> Cumulative energy overflowed counter!");
    }
    deltaWh = latestWh - lastStoredWh;
    if ((deltaWh > whDeltaStore) || energyOverflow)
    {
        nvmStoreCumulative(pPkt, pData);
        lastStoredWh = latestWh;
    }
}

static void
pulseConfigure(const Emon32Config_t *pCfg)
{
    (void)pCfg;
    #if (NUM_PULSECOUNT > 0)
    PulseCfg_t *pulseCfg = pulseGetCfg(0);
    /* REVISIT : make the 100 Hz time out periods and active configurable */
    if (0 != pulseCfg)
    {
        pulseCfg->edge      = PULSE_EDGE_FALLING;
        pulseCfg->grp       = GRP_PULSE0;
        pulseCfg->pin       = PIN_PULSE0;
        pulseCfg->periods   = 4u;
        pulseCfg->active    = 1u;
        pulseInit(0);
    }
    #endif
}

/*! @brief Setup the microcontoller. This function must be called first. An
 *         implementation must provide all the functions that are called.
 *         These can be empty if they are not used.
 */
static void
setupMicrocontroller()
{
    clkSetup();
    timerSetup();
    portSetup();
    dmacSetup();
    sercomSetup();

    dbgPutBoard();

    adcSetup();
    evsysSetup();
    wdtSetup(WDT_PER_4K);
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
    Emon32Config_t  e32Config;
    ECMDataset_t    ecmDataset;
    Emon32Dataset_t dataset;
    eepromPktWL_t   nvmCumulative;
    RFMPkt_t        *rfmPkt;
    char            txBuffer[TX_BUFFER_W] = {0};
    PackedData_t    packedData;
    uint32_t        cyclesProcessed = 0u;
    uint32_t        tempCount = 0;

    setupMicrocontroller();

    /* Setup DMAC for non-blocking UART (this is optional, unlike ADC) */
    uartConfigureDMA();
    uartInterruptEnable(SERCOM_UART_DBG, SERCOM_USART_INTENSET_RXC);
    uartInterruptEnable(SERCOM_UART_DBG, SERCOM_USART_INTENSET_ERROR);

    /* Load stored values (configuration and accumulated energy) from
     * non-volatile memory (NVM). If the NVM has not been used before then
     * store default configuration and 0 energy accumulator area.
     * REVISIT add check that firmware version matches stored config.
     */
    nvmLoadConfiguration(&e32Config);
    datasetInit(&dataset, &ecmDataset);
    nvmCumulativeConfigure(&nvmCumulative);
    nvmLoadCumulative(&nvmCumulative, &dataset);
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
    ledStatusOn();
    emon32StateSet(EMON_STATE_ACTIVE);
    ecmConfigure(&e32Config);
    adcStartDMAC((uint32_t)ecmDataBuffer());
    dbgPuts("> Start monitoring...\r\n");

    for (;;)
    {
        /* While there is an event pending (may be set while another is
         * handled), keep looping. Enter sleep (WFI) when done.
         */
        while(0 != evtPend)
        {
            /* 10 ms timer flag */
            if (evtPending(EVT_SYSTICK_100Hz))
            {
                evtKiloHertz();
                emon32EventClr(EVT_SYSTICK_100Hz);
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
                if ((e32Config.baseCfg.reportCycles - cyclesProcessed) == e32Config.baseCfg.mainsFreq)
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
                if (TEMP_MAX_ONEWIRE > 0)
                {
                    /* REVISIT : report failed conversion here */
                    (void)tempStartSample(TEMP_ONEWIRE, tempCount);
                }
                emon32EventClr(EVT_TEMP_SAMPLE);
            }

            /* Read back samples from each DS18B20 present. This is a blocking
             * routine, and is lower priority than processing a cycle, so read
             * one sensor on each loop.
             */
            if (evtPending(EVT_TEMP_READ))
            {
                if (TEMP_MAX_ONEWIRE > 0)
                {
                    tempReadSample(TEMP_ONEWIRE, tempCount);

                    tempCount++;
                    if (tempCount == TEMP_MAX_ONEWIRE)
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
                    dataPackagePacked(&dataset, &packedData);
                    rfmSend(&packedData);
                }
                else
                uartPutsNonBlocking(DMA_CHAN_UART_DBG, txBuffer, pktLength);

                /* If the energy used since the last storage is greater than the
                 * configured energy delta (baseCfg.whDeltaStore), then save the
                 * accumulated energy in NVM.
                 */
                processCumulative(&nvmCumulative, &dataset, e32Config.baseCfg.whDeltaStore);

                emon32EventClr(EVT_ECM_SET_CMPL);
            }

            /* If timer for EEPROM was not available, then retry until it is free */
            if (evtPending(EVT_EEPROM_TMR))
            {
                if (0 == timerDelayNB_us(EEPROM_WR_TIME, &eepromWriteCB))
                {
                    emon32EventClr(EVT_EEPROM_TMR);
                }
            }
        }
        /* Enter WFI until woken by an interrupt */
        __WFI();
    };
}
