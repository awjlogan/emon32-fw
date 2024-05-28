#include <inttypes.h>
#include <string.h>


#include "emon32_assert.h"

#include "driver_ADC.h"
#include "driver_PORT.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"

#include "configuration.h"
#include "eeprom.h"
#include "emon_CM.h"
#include "emon32.h"
#include "emon32_build_info.h"
#include "util.h"

#include "printf.h"
#include "qfplib-m0-full.h"


/*************************************
 * Types
 *************************************/

typedef enum {
    RCAUSE_SYST     = 0x40,
    RCAUSE_WDT      = 0x20,
    RCAUSE_EXT      = 0x10,
    RCAUSE_BOD33    = 0x04,
    RCAUSE_BOD12    = 0x02,
    RCAUSE_POR      = 0x01
} RCAUSE_t;


/*************************************
 * Prototypes
 *************************************/

static void     configDefault       (void);
static void     configInitialiseNVM (void);
static void     configureAnalog     (void);
static void     configurePulse      (void);
static uint32_t getBoardRevision    (void);
static char*    getLastReset        (void);
static uint32_t getUniqueID         (unsigned int idx);
static void     phaseAutoCalibrate  (void);
static void     printSettings       (void);
static char     waitForChar         (void);
static void     zeroAccumulators    (void);


/*************************************
 * Local variables
 *************************************/

#define                 IN_BUFFER_W 64u
static char             inBuffer[IN_BUFFER_W];
static unsigned int     inBufferIdx = 0;
static Emon32Config_t   *pCfg;
static unsigned int     resetReq = 0;


/*! @brief Set all configuration values to defaults */
static void
configDefault(void)
{
    pCfg->key = CONFIG_NVM_KEY;

    /* Single phase, 50 Hz, 240 VAC, 10 s report period */
    pCfg->baseCfg.nodeID        = NODE_ID;  /* Node ID to transmit */
    pCfg->baseCfg.mainsFreq     = 50u;      /* Mains frequency */
    pCfg->baseCfg.reportTime    = 9.8f;
    pCfg->baseCfg.whDeltaStore  = DELTA_WH_STORE; /* 200 */
    pCfg->baseCfg.dataGrp       = 210u;
    pCfg->baseCfg.logToSerial   = true;
    pCfg->baseCfg.useJson       = false;
    pCfg->dataTxCfg.txType      = DATATX_UART;
    pCfg->dataTxCfg.rfmPwr      = 0x19;
    pCfg->dataTxCfg.rfmFreq     = 0;

    for (unsigned int idxV = 0u; idxV < NUM_V; idxV++)
    {
        /* This is the peak mains voltage producing 1.024 V at the input
         * to the emon32 system.
         */
        pCfg->voltageCfg[idxV].voltageCal = 405.0f;
    }

    /* 4.2 degree shift @ 50 Hz */
    for (unsigned int idxCT = 0u; idxCT < NUM_CT; idxCT++)
    {
        pCfg->ctCfg[idxCT].ctCal    = 20.0f;
        pCfg->ctCfg[idxCT].phase    = 4.2f;
        pCfg->ctCfg[idxCT].vChan    = 0;
    }
    pCfg->ctActive = (1 << NUM_CT_ACTIVE_DEF) - 1u;

    /* Pulse counters:
     *   - Period: 100 ms
     *   - Rising edge trigger
     *   - All disabled
    */
    pCfg->pulseActive = 0u;
    for (unsigned int i = 0u; i < NUM_PULSECOUNT; i++)
    {
        pCfg->pulseCfg[i].period = 100u;
        pCfg->pulseCfg[i].edge   = 0u;
    }

    pCfg->crc16_ccitt = calcCRC16_ccitt(pCfg, (sizeof(*pCfg) - 2u));
}


/*! @brief Write the configuration values to index 0, and zero the
 *         accumulator space to.
 */
static void
configInitialiseNVM(void)
{
    unsigned int eepromSize = 0;

    dbgPuts                 ("  - Initialising NVM... ");

    configDefault           ();
    (void)eepromInitBlock   (0, 0, 256);
    eepromInitConfig        (pCfg, sizeof(*pCfg));

    // eepromSize = eepromDiscoverSize();
    eepromSize = 1024;
    (void)eepromInitBlock(EEPROM_WL_OFFSET, 0,
                          (eepromSize - EEPROM_WL_OFFSET));
    dbgPuts("Done!\r\n");
}


/*! @brief Configure an analog channel. */
static void
configureAnalog(void)
{
    /* String format: k<x> <yy.y> <zz.z>
     * Find space delimiters, then convert to null and a->i/f
     */
    unsigned int    ch          = 0;
    unsigned int    posCalib    = 0;
    float           cal         = 0.0f;
    unsigned int    posPhase    = 0;

    for (unsigned int i = 0; i < IN_BUFFER_W; i++)
    {
        if (' ' == inBuffer[i])
        {
            inBuffer[i] = 0;
            if (0 == posCalib)
            {
                posCalib = i + 1u;
            }
            else
            {
                posPhase = i + 1u;
                break;
            }
        }
    }

    /* Didn't find a space, so exit early */
    if (0 == posCalib)
    {
        return;
    }

    /* Voltage channels are [0..2], CTs are [3..]. */
    ch = utilAtoi(inBuffer + 1u, ITOA_BASE10);
    cal = utilAtof(inBuffer + posCalib);

    if (3 > ch)
    {
        pCfg->voltageCfg[ch].voltageCal = cal;
        printf_("> V%d calibration set to: %.02f\r\n",
                (ch + 1), qfp_float2double(pCfg->voltageCfg[ch].voltageCal));
        return;
    }
    else
    {
        pCfg->ctCfg[ch - 3u].ctCal = cal;
        printf_("> CT%d calibration set to: %.02f\r\n",
                (ch - 2u), qfp_float2double(pCfg->ctCfg[ch - 3u].ctCal));
    }

    /* Didn't find a space for value "z", so exit early */
    if (0 == posPhase)
    {
        return;
    }

    cal = utilAtof(inBuffer + posPhase);
    pCfg->ctCfg[ch - 3u].phase = cal;
    printf_("> CT%d phase set to: %.02f\r\n",
            (ch - 2u), qfp_float2double(pCfg->ctCfg[ch - 3u].phase));
}

/*! @brief Configure a pulse channel. */
static void
configurePulse(void)
{
    /* String format in inBuffer:
     *      [1] -> ch;
     *      [3] -> active;
     *      [5] -> edge (rising, falling, both)
     *      [7] -> NULL: blank time
     */
    const unsigned int ch       = (inBuffer[1] - 48u) - 1u;
    const unsigned int active   = inBuffer[3] - 48u;
    const unsigned int edgePos  = 5u;
    const unsigned int timePos  = 7u;

    /* If inactive, clear active flag, no decode for the rest */
    if (0 == active)
    {
        pCfg->pulseActive &= ~(1 << ch);
        printf_("> Pulse channel %d disabled.\r\n", (ch + 1u));
        return;
    }
    else
    {
        pCfg->pulseActive |= (1 << ch);
        printf_("> Pulse channel %d: ", (ch + 1u));
        switch (inBuffer[edgePos])
        {
            case 'r':
                printf_("Rising, ");
                pCfg->pulseCfg[ch].edge = 0u;
                break;
            case 'f':
                printf_("Falling, ");
                pCfg->pulseCfg[ch].edge = 1u;
                break;
            case 'b':
                printf_("Both, ");
                pCfg->pulseCfg[ch].edge = 2u;
                break;
        }
        pCfg->pulseCfg[ch].period = utilAtoi((inBuffer + timePos), ITOA_BASE10);
        printf_("%d ms\r\n", pCfg->pulseCfg[ch].period);
    }
}


/*! @brief Get the board revision, software visible changes only
 *  @return : board revision, 0-7
 */
static uint32_t
getBoardRevision(void)
{
    uint32_t boardRev = 0;
    boardRev |= portPinValue(GRP_REV, PIN_REV0);
    boardRev |= portPinValue(GRP_REV, PIN_REV1) << 1;
    boardRev |= portPinValue(GRP_REV, PIN_REV2) << 2;
    return boardRev;
}


/*! @brief Get the last reset cause (16.8.14)
 *  @return : null-terminated string with the last cause.
 */
static char*
getLastReset(void)
{
    const RCAUSE_t lastReset = (RCAUSE_t)PM->RCAUSE.reg;
    switch (lastReset)
    {
        case RCAUSE_SYST:
            return "Reset request";
            break;
        case RCAUSE_WDT:
            return "Watchdog timeout";
            break;
        case RCAUSE_EXT:
            return "External reset";
            break;
        case RCAUSE_BOD33:
            return "3V3 brownout";
            break;
        case RCAUSE_BOD12:
            return "1V2 brownout";
            break;
        case RCAUSE_POR:
            return "Power on cold reset";
            break;
    }
    return "Unknown";
}


/*! @brief Fetch the SAMD's 128bit unique ID
 *  @param [in] idx : index of 32bit word
 *  @return : 32bit word from index
 */
static uint32_t
getUniqueID(unsigned int idx)
{
    /* Section 10.3.3 Serial Number */
    const uint32_t id_addr_lut[4] = {
        0x0080A00C, 0x0080A040, 0x0080A044, 0x0080A048
    };
    return *(volatile uint32_t *)id_addr_lut[idx];
}


/*! @brief Start auto calibration for CT lead */
static void
phaseAutoCalibrate(void)
{
    unsigned int ch = utilAtoi(inBuffer + 1u, ITOA_BASE10);

    char c;
    dbgPuts("> Begin phase auto calibration? 'y' to proceed.\r\n");

    c = waitForChar();
    if ('y' == c)
    {
        adcDMACStop();
        ecmPhaseCalibrate(ch);
    }
    else
    {
        dbgPuts("    - Cancelled.\r\n");
    }
}


/*! @brief Print the emon32's configuration settings */
static void
printSettings(void)
{
    dbgPuts("\r\n\r\n==== Settings ====\r\n\r\n");
    printf_("Mains frequency (Hz)       %d\r\n",
            pCfg->baseCfg.mainsFreq);
    printf_("Data log time (s):         %.02f\r\n",
            qfp_float2double(pCfg->baseCfg.reportTime));
    printf_("Minimum accumulation (Wh): %d\r\n",
            pCfg->baseCfg.whDeltaStore);
    printf_("Data transmission:         ");
    if (DATATX_RFM69 == pCfg->dataTxCfg.txType)
    {
        dbgPuts("RFM69\r\n");
        switch (pCfg->dataTxCfg.rfmFreq)
        {
            case 0:
                dbgPuts("868");
                break;
            case 1:
                dbgPuts("915");
                break;
            case 2:
                dbgPuts("433");
                break;
        }
        dbgPuts(" MHz\r\n");
        printf_("Power:     %d\r\n", pCfg->dataTxCfg.rfmPwr);
    }
    else
    {
        dbgPuts("Serial\r\n");
        printf_("Data transmission:         %s\r\n",
                pCfg->baseCfg.useJson ? "JSON" : "K:V");
    }
    dbgPuts("\r\n");

    for (unsigned int i = 0; i < NUM_PULSECOUNT; i++)
    {
        unsigned int enabled = pCfg->pulseActive & (1 << i);
        printf_("Pulse Channel %d:\r\n", i);
        printf_("  - Enabled:         %c\r\n", enabled ? 'Y' : 'N');
        printf_("  - Hysteresis (ms): %d\r\n", pCfg->pulseCfg[i].period);
        dbgPuts("  - Edge:            ");
        switch (pCfg->pulseCfg[i].edge)
        {
            case 0:
                dbgPuts("Rising");
                break;
            case 1:
                dbgPuts("Falling");
                break;
            case 2:
                dbgPuts("Both");
                break;
            default:
                dbgPuts("Unknown");
        }
        dbgPuts("\r\n");
    }

    dbgPuts("\r\n==== Calibration ====\r\n\r\n");
    for (unsigned int i = 0; i < NUM_V; i++)
    {
        printf_("Voltage Channel %d\r\n", (i + 1u));
        printf_("  - Conversion: %.02f\r\n",
                qfp_float2double(pCfg->voltageCfg[i].voltageCal));
    }
    dbgPuts("\r\n");
    for (unsigned int i = 0; i < NUM_CT; i++)
    {
        printf_("CT Channel %d\r\n", (i + 1u));
        printf_("  - Conversion:      %.02f\r\n",
                qfp_float2double(pCfg->ctCfg[i].ctCal));
        printf_("  - Phase:           %.02f\r\n",
                qfp_float2double(pCfg->ctCfg[i].phase));
        printf_("  - Voltage channel: %d\r\n", (pCfg->ctCfg[i].vChan + 1u));
    }
}


/*! @brief Blocking wait for a key from the serial link. */
static char
waitForChar(void)
{
    /* Disable the NVIC for the interrupt if needed while waiting for the
     * character otherwise it is handled by the configuration buffer.
     */
    char c;

    unsigned int irqEnabled = (NVIC->ISER[0] & (1 << ((uint32_t)(SERCOM_UART_INTERACTIVE_IRQn) & 0x1F))) ? 1 : 0;
    if (irqEnabled) NVIC_DisableIRQ(SERCOM_UART_INTERACTIVE_IRQn);

    while (0 == (uartInterruptStatus(SERCOM_UART_DBG) & SERCOM_USART_INTFLAG_RXC));
    c = uartGetc(SERCOM_UART_DBG);

    if (irqEnabled) NVIC_EnableIRQ(SERCOM_UART_INTERACTIVE_IRQn);
    return c;
}


/*! @brief Zero the accumulator portion of the NVM */
static void
zeroAccumulators(void)
{
    char c;
    dbgPuts("> Zero accumulators. This can not be undone. 'y' to proceed.\r\n");

    c = waitForChar();
    if ('y' == c)
    {
        (void)eepromInitBlock(EEPROM_WL_OFFSET, 0,
                              (1024 - EEPROM_WL_OFFSET));
        dbgPuts("    - Accumulators cleared.\r\n");
    }
    else
    {
        dbgPuts("    - Cancelled.\r\n");
    }
}


void
configCmdChar(const uint8_t c)
{
    if ('\n' == c)
    {
        emon32EventSet(EVT_PROCESS_CMD);
    }
    else if (inBufferIdx < IN_BUFFER_W)
    {
        inBuffer[inBufferIdx++] = c;
    }
}


void
configFirmwareBoardInfo(void)
{
    dbgPuts("\033c==== emon32 ====\r\n\r\n");

    dbgPuts("> Board:\r\n");
    printf_("  - emonPi3     (arch. rev. %"PRIu32")\r\n", getBoardRevision());
    printf_("  - Serial:     0x%02x%02x%02x%02x\r\n",
            (unsigned int)getUniqueID(0), (unsigned int)getUniqueID(1),
            (unsigned int)getUniqueID(2), (unsigned int)getUniqueID(3));
    printf_("  - Last reset: %s\r\n", getLastReset());
    printf_("  - Uptime (s): %"PRIu32"\r\n", timerUptime());

    dbgPuts("> Firmware:\r\n");
    printf_("  - Version:    %d.%d.%d\r\n",
            VERSION_FW_MAJ, VERSION_FW_MIN, VERSION_FW_REV);
    dbgPuts("  - Build:      ");
    dbgPuts(emon32_build_info_string());
    dbgPuts("\r\n\r\n");
    dbgPuts("  - Distributed under GPL3 license, see COPYING.md\r\n");
    dbgPuts("  - emon32 Copyright (C) 2023-24 Angus Logan\r\n");
    dbgPuts("  - For Bear and Moose\r\n\r\n");
}


void
configLoadFromNVM(Emon32Config_t *pConfig)
{
    EMON32_ASSERT(pConfig);

    const uint32_t  cfgSize     = sizeof(*pConfig);
    uint16_t        crc16_ccitt = 0;
    char            c           = 0;

    pCfg = pConfig;

    /* Load from "static" part of EEPROM. If the key does not match
     * CONFIG_NVM_KEY, write the default configuration to the EEPROM and zero
     * wear levelled portion. Otherwise, read configuration from EEPROM.
     */
    eepromRead(0, pCfg, cfgSize);

    if (CONFIG_NVM_KEY != pCfg->key)
    {
        configInitialiseNVM();
    }
    else
    {
        /* Check the CRC and raise a warning if not matched. -2 from the base
         * size to account for the stored 16 bit CRC.
         */
        crc16_ccitt = calcCRC16_ccitt(pCfg, cfgSize - 2u);
        if (crc16_ccitt != pCfg->crc16_ccitt)
        {
            printf_("  - CRC mismatch. Found: 0x%04x -- Expected: 0x%04x\r\n",
                    pCfg->crc16_ccitt, crc16_ccitt);
            dbgPuts("    - NVM may be corrupt. Overwrite with default? (y/n)\r\n");
            while ('y' != c && 'n' != c)
            {
                c = waitForChar();
            }
            if ('y' == c)
            {
                configInitialiseNVM();
            }
        }
    }
}


void
configProcessCmd(void)
{
    unsigned int arglen = 0;
    unsigned int termFound = 0;

    /* Help text - serves as documentation interally as well */
    const char helpText[] = "\r\n"
    "emon32 information and configuration commands\r\n\r\n"
    " - ?           : show this text again\r\n"
    " - b<n>        : set RF band. n = 4 -> 433 MHz, 8 -> 868 MHz, 9 -> 915 MHz\r\n"
    " - c<n>        : log to serial output. n = 0: OFF, n = 1: ON\r\n"
    " - d<x.x>      : data log period (s)\r\n"
    " - f<n>        : line frequency (Hz)\r\n"
    " - g<n>        : set network group (default = 210)\r\n"
    " - j<n>        : JSON serial format. n = 0: OFF, n = 1: ON\r\n"
    " - k<x> <y.y> <z.z>\r\n"
    "   - Calibrate an analogue input\r\n"
    "   - x:        : channel (0-2 -> V; 3... -> CT)\r\n"
    "   - y.y       : V/CT calibration constant\r\n"
    "   - z.z       : CT phase calibration value\r\n"
    " - l           : list settings\r\n"
    " - m<w> <x> <y> <z>\r\n"
    "   - Pulse counting.\r\n"
    "     - w : pulse channel index\r\n"
    "     - x = 0: OFF, x = 1, ON.\r\n"
    "     - y : edge sensitivity (r,f,b). Ignored if x = 0\r\n"
    "     - z : minimum period (ms). Ignored if x = 0\r\n"
    " - n<n>        : set node ID [1..60]\r\n"
    " - o<x>        : automatic phase calibration for CT[x]\r\n"
    " - p<n>        : set the RF power level\r\n"
    " - r           : restore defaults\r\n"
    " - s           : save settings to NVM\r\n"
    " - t           : trigger report on next cycle\r\n"
    " - v           : firmware and board information\r\n"
    " - w<n>        : minimum difference in energy before saving (Wh)\r\n"
    " - z           : zero energy accumulators\r\n\r\n";

    /* Convert \r or \n to 0, and get the length until then. */
    while (arglen < IN_BUFFER_W)
    {
        const uint8_t c = inBuffer[arglen];
        if (('\r' == c) || ('\n' == c))
        {
            inBuffer[arglen] = 0;
            termFound = 1u;
            break;
        }
        arglen++;
    }

    /* Early exit if no terminator found */
    if (0 == termFound)
    {
        return;
    }

    /* Decode on first character in the buffer */
    switch (inBuffer[0])
    {
        case '?':
            /* Print help text */
            dbgPuts(helpText);
            break;
        case 'c':
            /* Log to serial output, default TRUE
             * Format: c0 | c1
             */
            if (2u == arglen)
            {
                pCfg->baseCfg.logToSerial = utilAtoi(inBuffer + 1, ITOA_BASE10);
                printf_("> Log to serial: %c\r\n",
                        pCfg->baseCfg.logToSerial ? 'Y' : 'N');
                emon32EventSet(EVT_CONFIG_CHANGED);
            }
            break;
        case 'd':
            /* Set the datalog period (s) */
            pCfg->baseCfg.reportTime = utilAtof(inBuffer + 1);
            printf_("> Data log report time set to: %.02f\r\n",
                    qfp_float2double(pCfg->baseCfg.reportTime));
            resetReq = 1u;
            emon32EventSet(EVT_CONFIG_CHANGED);
            break;
        case 'f':
            /* Set line frequency.
             * Format: f50 | f60
             */
            if (3u == arglen)
            {
                pCfg->baseCfg.mainsFreq = utilAtoi(inBuffer + 1,
                                                   ITOA_BASE10);
                printf_("> Mains frequency set to: %d\r\n",
                        pCfg->baseCfg.mainsFreq);
                resetReq = 1u;
                emon32EventSet(EVT_CONFIG_CHANGED);
            }
            break;
        case 'j':
            if (2u == arglen)
            {
                pCfg->baseCfg.useJson = utilAtoi(inBuffer + 1, ITOA_BASE10);
                printf_("> Use JSON: %c\r\n",
                        pCfg->baseCfg.useJson ? 'Y' : 'N');
                emon32EventSet(EVT_CONFIG_CHANGED);
            }
            break;
        case 'k':
            /* Configure analog channel */
            configureAnalog();
            resetReq = 1u;
            emon32EventSet(EVT_CONFIG_CHANGED);
            break;
        case 'l':
            /* Print settings */
            printSettings();
            break;
        case 'm':
            /* Configure pulse channel */
            configurePulse();
            resetReq = 1u;
            emon32EventSet(EVT_CONFIG_CHANGED);
            break;
        case 'o':
            /* Start auto calibration of CT<x> lead */
            phaseAutoCalibrate();
            resetReq = 1u;
            emon32EventSet(EVT_CONFIG_CHANGED);
            break;
        case 'p':
            /* Configure RF power */
            resetReq = 1u;
            emon32EventSet(EVT_CONFIG_CHANGED);
            break;
        case 'r':
            /* Restore defaults */
            configDefault();
            printf_("> Restored default values.\r\n");
            resetReq = 1u;
            emon32EventSet(EVT_CONFIG_CHANGED);
            break;
        case 's':
            /* Save to EEPROM config space after recalculating CRC and
             * indicate if a reset is required.
             */
            pCfg->crc16_ccitt = calcCRC16_ccitt(pCfg, (sizeof(*pCfg) - 2));
            dbgPuts("> Saving configuration to NVM... ");
            eepromInitConfig(pCfg, sizeof(*pCfg));
            dbgPuts("Done!\r\n");
            if (0 == resetReq)
            {
                emon32EventSet(EVT_CONFIG_SAVED);
            }
            else
            {
                emon32EventSet(EVT_SAFE_RESET_REQ);
            }
            break;
        case 't':
            /* Trigger processing on set on next cycle complete */
            emon32EventSet(EVT_PROCESS_DATASET);
            break;
        case 'v':
            /* Print firmware and board information */
            configFirmwareBoardInfo();
            break;
        case 'w':
            /* Set the Wh delta between saving accumulators */
            pCfg->baseCfg.whDeltaStore = utilAtoi(inBuffer + 1,
                                                  ITOA_BASE10);
            printf_("> Energy delta set to: %d\r\n",
                    pCfg->baseCfg.whDeltaStore);
            emon32EventSet(EVT_CONFIG_CHANGED);
            break;
        case 'z':
            /* Clear accumulator space */
            zeroAccumulators();
            resetReq = 1u;
            emon32EventSet(EVT_CONFIG_CHANGED);
            break;
    }

    /* Clear buffer and reset pointer */
    inBufferIdx = 0;
    (void)memset(inBuffer, 0, IN_BUFFER_W);
}


unsigned int
configTimeToCycles(const float time, const unsigned int mainsFreq)
{
    return qfp_float2uint(qfp_fmul(time, qfp_uint2float(mainsFreq)));
}
