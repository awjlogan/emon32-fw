#include <string.h>
#ifdef HOSTED
    #include <stdio.h>
    #include <stdlib.h>

    #include "configuration.h"
    #include "util.h"

    #define SERCOM_UART_DBG     0u
    #define EEPROM_WL_OFFSET    0u
    #define EEPROM_WL_SIZE      0u

    void
    qfp_str2float(float *f, const char *s, char **endptr)
    {
        *f = strtof(s, endptr);
    }

    void
    qfp_float2str(float f, char *s, unsigned int fmt)
    {
        (void)snprintf(s, 16, "%f.2", f);
        (void)fmt;
    }

    /* Dummy functions called from emon32 top level */
    void
    eepromInitBlocking(int a, int b, int c)
    {
        (void)a;
        (void)b;
        (void)c;
    }

#else

    #include "emon32_samd.h"

    #include "driver_SERCOM.h"
    #include "configuration.h"
    #include "eeprom.h"
    #include "emon32.h"
    #include "qfplib.h"
    #include "qfpio.h"
    #include "util.h"

    #include "printf.h"

#endif

#define GENBUF_W        16u

static unsigned int     valChanged;
static Emon32Config_t   *pCfg;
static char             genBuf[GENBUF_W];

/*! @brief Fetch the board's unique ID, and place in genBuf
 *         For SAMD series, there is a unique value in the IC
 */
static uint32_t
getUniqueID(unsigned int idx)
{
    #ifdef HOSTED
        return 1 << (idx * 2);
    #else
        /* Section 9.6 Serial Number */
        const uint32_t id_addr_lut[4] = {
            0x0080A00C, 0x0080A040, 0x0080A044, 0x0080A048
        };
        return *(volatile uint32_t *)id_addr_lut[idx];
    #endif
}

static void
putString(const char *s)
{
    #ifndef HOSTED
        dbgPuts(s);
    #else
        printf("%s", s);
    #endif

}

static void
putChar(const char c)
{
    #ifndef HOSTED
        uartPutcBlocking(SERCOM_UART_DBG, c);
    #else
        printf("%c", c);
    #endif
}

static void
enterConfigText()
{
    putString("Enter the channel index to configure. (b)ack\r\n");
}

static char
waitForChar()
{
#ifndef HOSTED
    while (0 == (uartInterruptStatus(SERCOM_UART_DBG) & SERCOM_USART_INTFLAG_RXC));
    return uartGetc(SERCOM_UART_DBG);
#else
    char c;
    c = getchar();
    if ('\n' == c) c = getchar();
    return c;
#endif
}

static void
getInputStr()
{
    char            *pBuf = genBuf;
    unsigned int    charCnt = 0;
    char            c = 0;

    memset(genBuf, 0, GENBUF_W);

    /* Exit when # is received, or out of bounds */
    while ('#' != c && (charCnt < GENBUF_W))
    {
        c = waitForChar();
        if ('#' != c)
        {
            *pBuf++ = c;
            charCnt++;
        }
    }
}

static char
getChar()
{
    getInputStr();
    return *genBuf;
}

static unsigned int
getValue()
{
    getInputStr();
    return utilAtoi(genBuf, ITOA_BASE10);
}

static float
getValue_float()
{
    float f;
    getInputStr();
    qfp_str2float(&f, genBuf, 0);
    return f;
}

static void
infoEdit()
{
    putString("\r\nTo edit, enter the index, the value (base 10), then #. Go (b)ack\r\n");
}

static void
putValueEnd()
{
    putString(genBuf);
    putString("\r\n");
}

static void
putValueEnd_10(unsigned int val)
{
//     memset(genBuf, 0, GENBUF_W);
    (void)utilItoa  (genBuf, val, ITOA_BASE10);
    putValueEnd     ();
}

static void
putValueEnd_Float(float val)
{
    qfp_float2str   (val, genBuf, 0);
    putValueEnd     ();
}

static void
clearTerm()
{
    putString("\033c");
}

static void
menuReset()
{
    char c = 0;

    while ('b' != c)
    {
        clearTerm();
        putString("---- RESET DEVICE ----\r\n");
        putString("0: Restore default configuration.\r\n");
        putString("1: Clear stored energy accumulators.\r\n");
        putString("2: Dump NVM conents.\r\n");
        putString("(b)ack\r\n");

        c = waitForChar();

        if ('0' == c)
        {
            while ('y' != c && 'n' != c)
            {
                putString("Restore default configuration? (y/n)\r\n");
                c = waitForChar();
            }

            if ('y' == c)
            {
                valChanged = 1u;
                configDefault(pCfg);
            }
        }
        else if ('1' == c)
        {
            while ('y' != c && 'n' != c)
            {
                putString       ("Clear stored energy accumulators? (y/n)\r\n");
                c = waitForChar ();
            }

            if ('y' == c)
            {
                (void)eepromInitBlocking(EEPROM_WL_OFFSET, 0, EEPROM_WL_SIZE);
            }
        }
        else if ('2' == c)
        {
            eepromDump();
        }
    }
}

static void
menuPulseChan(unsigned int chanP)
{
    char c = 0;
    unsigned int activeMask = (1 << chanP);
    unsigned int idxChange  = 0;
    unsigned int val        = 0;
    char         input      = 0;

    while ('b' != c)
    {
        clearTerm       ();
        putString       ("---- PULSE CHANNEL ");
        (void)utilItoa  (genBuf, chanP, ITOA_BASE10);
        putString       (genBuf);
        putString       (" ----\r\n\r\n");
        putString       ("0: Active:           ");
        if (0 == (pCfg->pulseActive & activeMask))
        {
            putString   ("N\r\n");
        }
        else
        {
            putString   ("Y\r\n");
        }
        putString       ("1: Sensitive edge:   ");
        switch(pCfg->pulseCfg[chanP].edge)
        {
            case 0:
                putString("R\r\n");
                break;
            case 1:
                putString("F\r\n");
                break;
            case 2:
                putString("B\r\n");
                break;
        }
        putString       ("2: Debounce periods: ");
        (void)utilItoa(genBuf, pCfg->pulseCfg[chanP].period, ITOA_BASE10);
        putString(genBuf);
        putString("\r\n");
        infoEdit();

        c = waitForChar();
        if ((c >= '0') && (c <= '2'))
        {
            idxChange = c - '0';
            valChanged = 1;
            if (idxChange < 2)
            {
                input = getChar();
                if (0 == idxChange)
                {
                    if ('Y' == input)
                    {
                        pCfg->pulseActive |= activeMask;
                    }
                    else if ('N' == input)
                    {
                        pCfg->pulseActive &= ~activeMask;
                    }
                }
                else
                {
                    /* Values here match the PulseEdge_t enum */
                    if ('R' == input)
                    {
                        pCfg->pulseCfg[chanP].edge = 0;
                    }
                    else if ('F' == input)
                    {
                        pCfg->pulseCfg[chanP].edge = 1;
                    }
                    else if ('B' == input)
                    {
                        pCfg->pulseCfg[chanP].edge = 2;
                    }
                }
            }
            else
            {
                val = getValue();
                pCfg->pulseCfg[chanP].period = (uint8_t)val;
            }
        }
    }
}


static void
menuPulse()
{
    char c = 0;
    while ('b' != c)
    {
        clearTerm           ();
        putString           ("---- PULSE CHANNELS ----\r\n\r\n");
        for (unsigned int i = 0; i < NUM_PULSECOUNT; i++)
        {
            (void)utilItoa  (genBuf, i, ITOA_BASE10);
            putString       (genBuf);
            putString       (": Pulse Channel ");
            putString       (genBuf);
            putString       ("\r\n");
        }
        enterConfigText();
        c = waitForChar();

        if ('b' != c)
        {
            menuPulseChan(c - '0');
        }
    }
}

static void
menuVoltageChan(unsigned int chanV)
{
    char c = 0;

    while ('b' != c)
    {
        clearTerm       ();
        putString       ("---- VOLTAGE CHANNEL ");
        (void)utilItoa  (genBuf, chanV, ITOA_BASE10);
        putString       (genBuf);
        putString       (" ----\r\n\r\n");
        putString       ("0: Conversion factor: ");
        putValueEnd_Float(pCfg->voltageCfg[chanV].voltageCal);
        infoEdit();

        c = waitForChar();

        /* (Currently) only a single option for Voltage channel */
        if ('0' == c)
        {
            valChanged = 1;
            pCfg->voltageCfg[chanV].voltageCal = getValue_float();
        }
    }
}

static void
menuVoltage()
{
    char c = 0;

    while ('b' != c)
    {
        clearTerm           ();
        putString           ("---- VOLTAGE CHANNELS ----\r\n\r\n");
        for (unsigned int idxV = 0; idxV < NUM_V; idxV++)
        {
            (void)utilItoa  (genBuf, idxV, ITOA_BASE10);
            putString       (genBuf);
            putString       (": Voltage Channel ");
            putString       (genBuf);
            putString       ("\r\n");
        }
        enterConfigText();

        c = waitForChar();

        if ('b' != c)
        {
            menuVoltageChan(c - '0');
        }
    }
}

static void
menuCTChan(unsigned int chanCT)
{
    unsigned int activeMask = (1 << chanCT);
    char         c          = 0;
    unsigned int idxChange  = 0;
    char         input      = 0;

    while ('b' != c)
    {
        clearTerm       ();
        putString       ("---- CT CHANNEL ");
        (void)utilItoa  (genBuf, chanCT, ITOA_BASE10);
        putString       (genBuf);
        putString       (" ----\r\n\r\n");
        putString       ("0: Active:              ");
        if (0 == (activeMask & pCfg->ctActive))
        {
            putString   ("N\r\n");
        }
        else
        {
            putString   ("Y\r\n");
        }
        putString       ("1: Conversion factor:   ");
        putValueEnd_Float(pCfg->ctCfg[chanCT].ctCal);
        putString       ("2: Phase calibration X: ");
        putValueEnd_10  (pCfg->ctCfg[chanCT].phaseX);
        putString       ("3: Phase calibration Y: ");
        putValueEnd_10  (pCfg->ctCfg[chanCT].phaseY);
        putString       ("4: V Channel:           ");
        putValueEnd_10  (pCfg->ctCfg[chanCT].vChan);
        infoEdit();

        c = waitForChar();

        if ((c >= '0') && (c <= '4'))
        {
            idxChange = c - '0';
            valChanged = 1u;

            if (0 == idxChange)
            {
                input = getChar();
                if ('Y' == input)
                {
                    pCfg->ctActive |= activeMask;
                }
                else if ('N' == input)
                {
                    pCfg->ctActive &= ~activeMask;
                }
            }
            else if (1 == idxChange)
            {
                pCfg->ctCfg[chanCT].ctCal = getValue_float();
            }
            else
            {
                int16_t val_fixed = getValue();
                switch (idxChange)
                {
                    case 2:
                        pCfg->ctCfg[chanCT].phaseX = val_fixed;
                        break;
                    case 3:
                        pCfg->ctCfg[chanCT].phaseY = val_fixed;
                        break;
                    case 4:
                        pCfg->ctCfg[chanCT].vChan = val_fixed;
                        break;
                    default:
                        break;
                }
            }
        }
    }
}

static void
menuCT()
{
    char c = 0;
    while ('b' != c)
    {
        clearTerm           ();
        putString           ("---- CT CHANNELS ----\r\n\r\n");
        for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
        {
            (void)utilItoa  (genBuf, idxCT, ITOA_BASE10);
            putString       (genBuf);
            putString       (": CT Channel ");
            putString       (genBuf);
            putString       ("\r\n");
        }
        enterConfigText();

        c = waitForChar();

        if ('b' != c)
        {
            menuCTChan(c - '0');
        }
    }
}

static void
menuConfiguration()
{
    char c = 0;
    unsigned int idxChange;

    while ('b' != c)
    {
        clearTerm       ();
        putString       ("---- CONFIGURATION ----\r\n\r\n");
        putString       ("0: Node ID:                    ");
        putValueEnd_10  (pCfg->baseCfg.nodeID);
        putString       ("1: Cycles to report:           ");
        putValueEnd_10  (pCfg->baseCfg.reportCycles);
        putString       ("2: Mains frequency (Hz):       ");
        putValueEnd_10  (pCfg->baseCfg.mainsFreq);
        putString       ("3: Energy delta to store (Wh): ");
        putValueEnd_10  (pCfg->baseCfg.whDeltaStore);
        infoEdit();

        c = waitForChar();

        if ((c >= '0') && (c <= '3'))
        {
            uint32_t val;
            idxChange = c - '0';
            val = getValue();

            valChanged = 1u;
            switch (idxChange)
            {
                case 0:
                    pCfg->baseCfg.nodeID = val;
                    break;
                case 1:
                    pCfg->baseCfg.reportCycles = val;
                    break;
                case 2:
                    pCfg->baseCfg.mainsFreq = val;
                    break;
                case 3:
                    pCfg->baseCfg.whDeltaStore = val;
                    break;
                default:
                    valChanged = 0;
                    break;
            }
        }
    }
}

static void
menuAbout()
{
    char c = 0;

    clearTerm       ();
    putString       ("---- ABOUT ----\r\n\r\n");
    putString       ("Firmware version: ");
    (void)utilItoa  (genBuf, VERSION_FW_MAJ, ITOA_BASE10);
    putString       (genBuf);
    putChar         ('.');
    putValueEnd_10  (VERSION_FW_MIN);

    putString("Serial number:    ");
    for (unsigned int i = 0; i < 4; i++)
    {
        utilItoa (genBuf, getUniqueID(i), ITOA_BASE16);
        putString(genBuf);
    }
    putString("\r\n\r\n");

    putString       ("Voltage channels: ");
    putValueEnd_10  (NUM_V);

    putString       ("CT channels:      ");
    putValueEnd_10  (NUM_CT);

    putString       ("\r\n(c) Angus Logan 2022-23\r\n");
    putString       ("For Bear and Moose\r\n\r\n");
    putString       ("(b)ack");

    while ('b' != c)
    {
        c = waitForChar();
    }
}

static void
menuBase()
{
    char c = 'a';

    while ('s' != c && 'e' != c)
    {
        /* Clear terminal and print menu */
        clearTerm();

        putString("== Energy Monitor 32 ==\r\n\r\n");
        putString("  0: About\r\n");
        putString("  1: Configuration\r\n");
        putString("  2: Voltage\r\n");
        putString("  3: CT\r\n");
        putString("  4: Pulse counter\r\n");
        putString("  9: Reset device\r\n");
        putString("Enter number, or (e)xit");

        if (valChanged)
        {
            putString(" do not save, or (s)ave and exit.");
        }
        putString("\r\n");

        c = waitForChar();

        switch (c)
        {
            case '0':
                menuAbout();
                break;
            case '1':
                menuConfiguration();
                break;
            case '2':
                menuVoltage();
                break;
            case '3':
                menuCT();
                break;
            case '4':
                menuPulse();
                break;
            case '9':
                menuReset();
                break;
            /* Fall through save or exit */
            case 's':
            case 'e':
                break;
            default:
                /* Terminal ping/flash */
                putChar('\a');
        }
    }

    /* Warn if the changes are going to be discarded. If yes is selected,
     * then just exit, otherwise save.
     */
    if ((0 != valChanged) && ('e' == c))
    {
        putString("Discard changes? (y/n)\r\n");
        while (('y' != c) && ('n' != c))
        {
            c = waitForChar();

        }
        c = ('y' == c) ? 'e' : 's';
    }

    /* Save configuration if requested, then reset the system */
    if ('s' == c)
    {
        #ifndef HOSTED
        pCfg->crc16_ccitt = calcCRC16_ccitt(pCfg, (sizeof(Emon32Config_t) - 2u));
        eepromInitConfig(pCfg, sizeof(Emon32Config_t));
        #endif
    }
    NVIC_SystemReset();
}


void
configEnter(Emon32Config_t *pConfig)
{
    pCfg = pConfig;
    menuBase();
}


static void
configInitialiseNVM(Emon32Config_t *pCfg)
{
    dbgPuts                 ("> Initialising NVM... ");
    configDefault           (pCfg);
    eepromInitConfig        (pCfg, sizeof(Emon32Config_t));
    (void)eepromInitBlocking(EEPROM_WL_OFFSET, 0, EEPROM_WL_SIZE);
    dbgPuts("Done!\r\n");
}


void
configLoadFromNVM(Emon32Config_t *pCfg)
{
    uint32_t        key         = 0u;
    const uint32_t  cfgSize     = sizeof(Emon32Config_t);
    uint16_t        crc16_ccitt = 0;
    char            c           = 0;

    /* Load 32bit key from "static" part of EEPROM. If the key does not match
     * CONFIG_NVM_KEY, write the default configuration to the EEPROM and zero
     * wear levelled portion. Otherwise, read configuration from EEPROM.
     */
    eepromRead(0, &key, 4u);

    if (CONFIG_NVM_KEY != key)
    {
        configInitialiseNVM(pCfg);
    }
    else
    {
        dbgPuts     ("> Reading configuration from NVM... ");
        eepromRead  (0, pCfg, cfgSize);
        dbgPuts     ("Done!\r\n");

        /* Check the CRC and raise a warning if no matched. -2 from the base
         * size to account for the stored 16 bit CRC.
         */
        crc16_ccitt = calcCRC16_ccitt(pCfg, cfgSize - 2u);
        if (crc16_ccitt != pCfg->crc16_ccitt)
        {
            printf_("    - CRC mismatch. Found: 0x%x -- expected: 0x%x\r\n", pCfg->crc16_ccitt, crc16_ccitt);
            dbgPuts("    - NVM may be corrupt. Overwrite with default? (y/n)\r\n");
            while ('y' != c && 'n' != c)
            {
                c = waitForChar();
            }
            if ('y' == c)
            {
                configInitialiseNVM(pCfg);
            }
        }
    }
}

void
configDefault(Emon32Config_t *pCfg)
{
    pCfg->key = CONFIG_NVM_KEY;

    /* Default configuration: single phase, 50 Hz, 240 VAC */
    pCfg->baseCfg.nodeID        = NODE_ID;  /* Node ID to transmit */
    pCfg->baseCfg.mainsFreq     = 50u;  /* Mains frequency */
    pCfg->baseCfg.reportCycles  = 500u; /* 10 s @ 50 Hz */
    pCfg->baseCfg.whDeltaStore  = DELTA_WH_STORE; /* 200 */
    pCfg->baseCfg.dataTx        = DATATX_UART;

    for (unsigned int idxV = 0u; idxV < NUM_V; idxV++)
    {
        pCfg->voltageCfg[idxV].voltageCal = 268.97;
    }

    /* 4.2 degree shift @ 50 Hz */
    for (unsigned int idxCT = 0u; idxCT < NUM_CT; idxCT++)
    {
        pCfg->ctCfg[idxCT].ctCal    = 90.91;
        pCfg->ctCfg[idxCT].phaseX   = 13495;
        pCfg->ctCfg[idxCT].phaseY   = 19340;
        pCfg->ctCfg[idxCT].vChan    = 0;
    }
    pCfg->ctActive = (1 << NUM_CT_ACTIVE_DEF) - 1u;

    pCfg->crc16_ccitt = calcCRC16_ccitt(pCfg, (sizeof(Emon32Config_t) - 2u));
}


typedef enum {
    CU_IDLE,
    CU_PERIOD,
    CU_SERIAL_LOG,
    CU_LINE_FREQUENCY,
    CU_CALI_AN,
    CU_ASSUMED_V,
    CU_PULSE_COUNT
} CUState_t;

#define IN_BUFFER_W 64u

static unsigned int inBufferIdx = 0;
static uint8_t      inBuffer[IN_BUFFER_W];

void
configUpdate()
{
    /* Simple state machine to progress through the configuration values like
     * the emonPi2.
     * Base commands:
     *  - l         : list settings
     *  - r         : restore defaults
     *  - s         : save settings to NVM
     *  - v         : firmware information
     *  - z         : zero energy accumulators
     *  - x         : exit, lock, and continue
     *  - ?         : show this text again
     *  - d<xx.x>   : data log period (s)
     *  - c<n>      : log to serial output. N = 0: OFF, N = 1: ON
     *  - f<xx>     : line frequency (Hz)
     *  - k<x> <yy.y> <zz.z>
     *    - Calibrate an analogue input
     *    - x:      : channel
     *  - a<xxx>    : assumed voltage if no AC detected
     *  - m<x> <yy> : pulse counting. X = 0: OFF, X = 1, ON. yy
    */

    /* Terminate buffer at index, shouldn't fill but just in case */
    if (IN_BUFFER_W == inBufferIdx)
    {
        inBuffer[IN_BUFFER_W - 1u] = '\0';
    }
    else
    {
        inBuffer[inBufferIdx] = '\0';
    }

    switch (inBuffer[0])
    {

    }
}

void
configGetChar(uint8_t c)
{
    if ('\n' == c)
    {
        configUpdate();
        inBufferIdx = 0;
        (void)memset(inBuffer, 0, IN_BUFFER_W);
    }
    else if (inBufferIdx < IN_BUFFER_W)
    {
        inBuffer[inBufferIdx++] = c;
    }
}
