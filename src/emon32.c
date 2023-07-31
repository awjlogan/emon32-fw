#include <stddef.h>
#include <string.h>

#include "emon32_samd.h"

#define EEPROM_WL_NUM_BLK   EEPROM_WL_SIZE / EEPROM_WL_SIZE_BLK

/*************************************
 * Persistent state variables
 *************************************/

static volatile uint32_t    evtPend;
static volatile EmonState_t emonState = EMON_STATE_IDLE;

static unsigned int         lastStoredWh;

/*************************************
 * Static function prototypes
 *************************************/

static void     dbgPutBoard();
static void     evtKiloHertz();
static uint32_t evtPending(EVTSRC_t evt);
static void     ledOn();
static void     ledToggle();
static void     loadConfiguration(Emon32Config_t *pCfg);
static void     loadCumulative(eepromPktWL_t *pPkt, ECMSet_t *pData);
static void     processCumulative(eepromPktWL_t *pPkt, const ECMSet_t *pData, const unsigned int whDeltaStore);
static void     setup_uc();
static void     storeCumulative(eepromPktWL_t *pPkt, const ECMSet_t *pData);
static uint32_t totalEnergy(const ECMSet_t *pData);

/*************************************
 * Functions
 *************************************/

void
dbgPuts(const char *s)
{
    uartPutsBlocking(SERCOM_UART_DBG, s);
}

void
emon32SetEvent(EVTSRC_t evt)
{
    /* Disable interrupts during RMW update of event status */
    uint32_t evtDecode = (1u << evt);
    __disable_irq();
    evtPend |= evtDecode;
    __enable_irq();
}

void
emon32ClrEvent(EVTSRC_t evt)
{
    /* Disable interrupts during RMW update of event status */
    uint32_t evtDecode = ~(1u << evt);
    __disable_irq();
    evtPend &= evtDecode;
    __enable_irq();
}

void
emon32StateSet(EmonState_t state)
{
    emonState = state;
}

EmonState_t
emon32StateGet()
{
    return emonState;
}

/*! @brief The default configuration state of the system */
void
emon32DefaultConfiguration(Emon32Config_t *pCfg)
{
    pCfg->key = CONFIG_NVM_KEY;

    /* Default configuration: single phase, 50 Hz, 240 VAC */
    pCfg->baseCfg.nodeID        = NODE_ID;  /* Node ID to transmit */
    pCfg->baseCfg.mainsFreq     = 50u;  /* Mains frequency */
    pCfg->baseCfg.reportCycles  = 500u; /* 10 s @ 50 Hz */
    pCfg->baseCfg.whDeltaStore  = DELTA_WH_STORE; /* 200 */
    pCfg->baseCfg.dataTx        = DATATX_RFM69;

    for (unsigned int idxV = 0u; idxV < NUM_V; idxV++)
    {
        pCfg->voltageCfg[idxV].voltageCal = 268.97;
    }

    /* 4.2 degree shift @ 50 Hz, 4 CTs */
    for (unsigned int idxCT = 0u; idxCT < NUM_CT; idxCT++)
    {
        pCfg->ctCfg[idxCT].ctCal    = 90.91;
        pCfg->ctCfg[idxCT].phaseX   = 13495;
        pCfg->ctCfg[idxCT].phaseY   = 19340;
    }
}

static void
ledOn()
{
    portPinDrv(GRP_PIN, PIN_LED_STATUS, PIN_DRV_CLR);
}

static void
ledToggle()
{
    portPinDrv(GRP_PIN, PIN_LED_STATUS, PIN_DRV_TGL);
}

/*! @brief This function handles loading of configuration data
 *  @param [in] pCfg : pointer to the configuration struct
 */
static void
loadConfiguration(Emon32Config_t *pCfg)
{
    unsigned int    systickCnt  = 0u;
    unsigned int    seconds     = 3u;
    uint32_t        key         = 0u;

    /* Load 32bit key from "static" part of EEPROM. If the key does not match
     * CONFIG_NVM_KEY, write the default configuration to the EEPROM and zero
     * wear levelled portion. Otherwise, read configuration from EEPROM.
     */
    eepromRead(0, (void *)&key, 4u);

    if (CONFIG_NVM_KEY != key)
    {
        dbgPuts("> Initialising NVM... ");
        eepromWrite(0, pCfg, sizeof(Emon32Config_t));
        while (EEPROM_WR_COMPLETE != eepromWrite(0, 0, 0))
        {
            timerDelay_us(EEPROM_WR_TIME);
        }
        (void)eepromInitBlocking(EEPROM_WL_OFFSET, 0, EEPROM_WL_SIZE);
        dbgPuts("Done\r\n");
    }
    else
    {
        eepromRead(0, (void *)pCfg, sizeof(Emon32Config_t));
    }

    /* Wait for 3 s, if a key is pressed then enter interactive configuration */
    dbgPuts("\r\n> Hit any key to enter configuration ");
    while (systickCnt < 4095)
    {
        if (uartInterruptStatus(SERCOM_UART_DBG) & SERCOM_USART_INTFLAG_RXC)
        {
            emon32StateSet(EMON_STATE_CONFIG);
            (void)uartGetc(SERCOM_UART_DBG);
            configEnter(pCfg);
            break;
        }

        if (evtPend & (1u << EVT_SYSTICK_1KHz))
        {
            wdtFeed();
            emon32ClrEvent(EVT_SYSTICK_1KHz);
            systickCnt++;

            /* Countdown every second, tick every 1/4 second to debug UART */
            if (0 == (systickCnt & 0x3FF))
            {
                ledToggle();
                uartPutcBlocking(SERCOM_UART_DBG, '0' + seconds);
                seconds--;
            }
            else if (0 == (systickCnt & 0xFF))
            {
                ledToggle();
                uartPutcBlocking(SERCOM_UART_DBG, '.');
            }
        }
    }
    dbgPuts("\r\n");
}

/*! @brief Total energy across all CTs
 *  @param [in] pData : pointer to data setup
 *  @return : sum of Wh for all CTs
 */
static uint32_t
totalEnergy(const ECMSet_t *pData)
{
    uint32_t totalEnergy = 0;
    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        totalEnergy += pData->CT[idxCT].wattHour;
    }
    return totalEnergy;
}

/*! @brief Load cumulative energy and pulse values
 *  @param [in] pEEPROM : pointer to EEPROM configuration
 *  @param [in] pData : pointer to current dataset
 */
static void
loadCumulative(eepromPktWL_t *pPkt, ECMSet_t *pData)
{
    Emon32Cumulative_t data;
    pPkt->pData = &data;

    memset(&data, 0, sizeof(Emon32Cumulative_t));
    eepromReadWL(pPkt);

    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        pData->CT[idxCT].wattHour = data.report.wattHour[idxCT];
    }

    pData->pulseCnt = data.report.pulseCnt;
}

/*! @brief Store cumulative energy and pulse values
 *  @param [in] pRes : pointer to cumulative values
 */
static void
storeCumulative(eepromPktWL_t *pPkt, const ECMSet_t *pData)
{
    int timerStatus;

    Emon32Cumulative_t data;
    pPkt->pData = &data;

    /* Copy data and calculate CRC */
    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        data.report.wattHour[idxCT] = pData->CT[idxCT].wattHour;
    }
    data.report.pulseCnt = pData->pulseCnt;

    data.crc = crc16_ccitt(&data.report, sizeof(Emon32Report_t));

    eepromWriteWL(pPkt);
    /* If the timer is in use, then add a pending event to finish the EEPROM
     * save if required */
    timerStatus = timerDelayNB_us(EEPROM_WR_TIME, &eepromWriteCB);
    if (-1 == timerStatus)
    {
        emon32SetEvent(EVT_EEPROM_TMR);
    }
}

static void
processCumulative(eepromPktWL_t *pPkt, const ECMSet_t *pData, const unsigned int whDeltaStore)
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
        storeCumulative(pPkt, pData);
        lastStoredWh = latestWh;
    }
}

/*! @brief This function is called when the 1 ms timer overflows (SYSTICK).
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

static void
dbgPutBoard()
{
    char        wr_buf[8];
    const int   board_id = BOARD_ID;

    dbgPuts("\033c== Energy Monitor 32 ==\r\n");
    dbgPuts("Board:    ");
    switch (board_id)
    {
        case (BOARD_ID_LC):
            dbgPuts("emon32 Low Cost");
            break;
        case (BOARD_ID_STANDARD):
            dbgPuts("emon32 Standard");
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
    dbgPuts("\r\n");
}


/*! @brief Setup the microcontoller. This function must be called first. An
 *         implementation must provide all the functions that are called.
 *         These can be empty if they are not used.
 */
static void
setup_uc()
{
    clkSetup();
    timerSetup();
    portSetup();
    dmacSetup();
    sercomSetup();
    adcSetup();
    evsysSetup();
    eicSetup();
    wdtSetup(WDT_PER_4K);
}

int
main()
{
    Emon32Config_t  e32Config;
    ECMSet_t        dataset;
    eepromPktWL_t   eepromPkt;
    RFMPkt_t        *rfmPkt;
    char            txBuffer[TX_BUFFER_W];

    setup_uc();

    /* Setup DMAC for non-blocking UART (this is optional, unlike ADC) */
    uartConfigureDMA();
    uartInterruptEnable(SERCOM_UART_DBG, SERCOM_USART_INTENSET_RXC);
    uartInterruptEnable(SERCOM_UART_DBG, SERCOM_USART_INTENSET_ERROR);

    dbgPutBoard();

    /* Load stored values (configuration and accumulated energy) from
     * non-volatile memory (NVM). If the NVM has not been used before then
     * store default configuration and 0 energy accumulator area.
     * REVISIT add check that firmware version matches stored config.
     */
    eepromPkt.addr_base     = EEPROM_WL_OFFSET;
    eepromPkt.blkCnt        = EEPROM_WL_NUM_BLK;
    eepromPkt.dataSize      = sizeof(Emon32Cumulative_t);
    eepromPkt.idxNextWrite  = -1;

    emon32DefaultConfiguration(&e32Config);
    loadConfiguration(&e32Config);
    loadCumulative(&eepromPkt, &dataset);
    lastStoredWh = totalEnergy(&dataset);

    /* Set up data transmission interfaces and configuration */
    if (DATATX_RFM69 == e32Config.baseCfg.dataTx)
    {
        sercomSetupSPI();
        rfmPkt              = rfmGetHandle();
        rfmPkt->node        = e32Config.baseCfg.nodeID;
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
        uart_data_cfg.gclk_id   = SERCOM_UART_DATA_GCLK_ID;
        uart_data_cfg.gclk_gen  = 3u;
        uart_data_cfg.pad_tx    = UART_DATA_PAD_TX;
        uart_data_cfg.pad_rx    = UART_DATA_PAD_RX;
        uart_data_cfg.port_grp  = GRP_SERCOM_UART_DATA;
        uart_data_cfg.pin_tx    = PIN_UART_DATA_TX;
        uart_data_cfg.pin_rx    = PIN_UART_DATA_RX;
        sercomSetupUART(&uart_data_cfg);
    }

    /* Set up buffers for ADC data, and configure energy processing */
    ledOn();
    emon32StateSet(EMON_STATE_ACTIVE);
    ecmInit(&e32Config);
    adcStartDMAC((uint32_t)ecmDataBuffer());
    dbgPuts("> Start monitoring...\r\n");

    for (;;)
    {
        /* While there is an event pending (may be set while another is
         * handled), keep looping. Enter sleep (WFI) when done.
         */
        while(0 != evtPend)
        {
            /* 1 ms timer flag */
            if (evtPending(EVT_SYSTICK_1KHz))
            {
                evtKiloHertz();
                emon32ClrEvent(EVT_SYSTICK_1KHz);
            }

            /* A full mains cycle has completed. Calculate power/energy */
            if (evtPending(EVT_ECM_CYCLE_CMPL))
            {
                ecmProcessCycle();
                emon32ClrEvent(EVT_ECM_CYCLE_CMPL);
            }

            /* Report period elapsed; generate, pack, and send through the
             * configured channel. Echo on debug console.
             */
            if (evtPending(EVT_ECM_SET_CMPL))
            {
                unsigned int pktLength;

                ecmProcessSet(&dataset);
                pktLength = dataPackageESP_n(&dataset, txBuffer, TX_BUFFER_W);

                if (pktLength >= TX_BUFFER_W)
                {
                    dbgPuts("> TX buffer overflowed!\r\n");
                }

                if (DATATX_RFM69 == e32Config.baseCfg.dataTx)
                {
                    /* TODO Data into RFM compatible format */
                    rfmSend(&dataset);
                }
                else
                {
                    uartPutsNonBlocking(DMA_CHAN_UART_DATA, txBuffer, pktLength);
                }
                uartPutsNonBlocking(DMA_CHAN_UART_DBG, txBuffer, pktLength);

                /* If the energy used since the last storage is greater than the
                 * configured energy delta (baseCfg.whDeltaStore), then save the
                 * accumulated energy in NVM.
                 */
                processCumulative(&eepromPkt, &dataset, e32Config.baseCfg.whDeltaStore);

                emon32ClrEvent(EVT_ECM_SET_CMPL);
            }

            /* If timer for EEPROM was not available, then retry until it is free */
            if (evtPending(EVT_EEPROM_TMR))
            {
                if (0 == timerDelayNB_us(EEPROM_WR_TIME, &eepromWriteCB))
                {
                    emon32ClrEvent(EVT_EEPROM_TMR);
                }
            }
        }
        /* Enter WFI until woken by an interrupt */
        __WFI();
    };
}
