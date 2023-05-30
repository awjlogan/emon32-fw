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

    void
    emon32StateSet(EmonState_t state)
    {
        (void)state;
    }

    void
    emon32DefaultConfiguration(Emon32Config_t *pCfg)
    {
        (void)pCfg;
    }

#else

    #include "qfplib.h"
    #include "qfpio.h"
    #include "emon32_samd.h"

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
        uartPutsBlocking(SERCOM_UART_DBG, s);
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
    (void)utilItoa(genBuf, val, ITOA_BASE10);
    putValueEnd();
}

static void
putValueEnd_Float(float val)
{
    qfp_float2str(val, genBuf, 0);
    putValueEnd();
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
        putString("(b)ack\r\n");

        c = waitForChar();

        if ('0' == c)
        {
            while ('Y' != c && 'N' == c)
            {
                putString("Restore default configuration? (Y/N)\r\n");
                c = waitForChar();
            }

            if ('Y' == c)
            {
                valChanged = 1u;
                emon32DefaultConfiguration(pCfg);
            }
        }
        else if ('1' == c)
        {
            while ('Y' != c && 'N' == c)
            {
                putString("Clear stored energy accumulators? (Y/N)\r\n");
                c = waitForChar();
            }

            if ('Y' == c)
            {
                (void)eepromInitBlocking(EEPROM_WL_OFFSET, 0, EEPROM_WL_SIZE);
            }
        }
    }
}

static void
menuVoltageChan(unsigned int chanV)
{
    char c = 0;

    while ('b' != c)
    {
        clearTerm();
        putString("---- VOLTAGE CHANNEL ");
        (void)utilItoa(genBuf, chanV, ITOA_BASE10);
        putString(genBuf);
        putString(" ----\r\n\r\n");
        putString("0: Conversion factor: ");
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
        clearTerm();
        putString("---- VOLTAGE CHANNELS ----\r\n\r\n");
        for (unsigned int idxV = 0; idxV < NUM_V; idxV++)
        {
            (void)utilItoa(genBuf, idxV, ITOA_BASE10);
            putString(genBuf);
            putString(": Voltage Channel ");
            putString(genBuf);
            putString("\r\n");
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
    char c = 0;
    unsigned int idxChange = 0;

    while ('b' != c)
    {
        clearTerm();
        putString("---- CT CHANNEL ");
        (void)utilItoa(genBuf, chanCT, ITOA_BASE10);
        putString(genBuf);
        putString(" ----\r\n\r\n");
        putString("0: Conversion factor:   ");
        putValueEnd_Float(pCfg->ctCfg[chanCT].ctCal);
        putString("1: Phase calibration X: ");
        putValueEnd_10(pCfg->ctCfg[chanCT].phaseX);
        putString("2: Phase calibration Y: ");
        putValueEnd_10(pCfg->ctCfg[chanCT].phaseY);
        infoEdit();

        c = waitForChar();

        if ('0' == c)
        {
            float   val_f;
            int16_t val_fixed;
            idxChange = c - '0';

            switch (idxChange)
            {
                case 0:
                    val_f = getValue_float();
                    pCfg->ctCfg[chanCT].ctCal = val_f;
                    valChanged = 1u;
                    break;
                case 1:
                    val_fixed = getValue();
                    pCfg->ctCfg[chanCT].phaseX = val_fixed;
                    valChanged = 1u;
                    break;
                case 2:
                    val_fixed = getValue();
                    pCfg->ctCfg[chanCT].phaseY = val_fixed;
                    valChanged = 1u;
                    break;
                default:
                    break;
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
        clearTerm();
        putString("---- CT CHANNELS ----\r\n\r\n");
        for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
        {
            (void)utilItoa(genBuf, idxCT, ITOA_BASE10);
            putString(genBuf);
            putString(": CT Channel ");
            putString(genBuf);
            putString("\r\n");
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
        clearTerm();
        putString("---- CONFIGURATION ----\r\n\r\n");
        putString("0: Node ID:                  ");
        putValueEnd_10(pCfg->baseCfg.nodeID);
        putString("1: Cycles to report:         ");
        putValueEnd_10(pCfg->baseCfg.reportCycles);
        putString("2: Mains frequency (Hz):     ");
        putValueEnd_10(pCfg->baseCfg.mainsFreq);
        putString("3: Energy delta to store:    ");
        putValueEnd_10(pCfg->baseCfg.whDeltaStore);
        infoEdit();

        c = waitForChar();

        if ((c >= '0') && (c <= '2'))
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

    clearTerm();
    putString("---- ABOUT ----\r\n\r\n");
    putString("Firmware version: ");

    (void)utilItoa(genBuf, VERSION_FW_MAJ, ITOA_BASE10);
    putString(genBuf);
    putChar('.');
    putValueEnd_10(VERSION_FW_MIN);

    putString("Serial number:    ");
    for (unsigned int i = 0; i < 4; i++)
    {
        utilItoa(genBuf, getUniqueID(i), ITOA_BASE16);
        putString(genBuf);
    }
    putString("\r\n\r\n");

    putString("Voltage channels: ");
    putValueEnd_10(NUM_V);

    putString("CT channels:      ");
    putValueEnd_10(NUM_CT);

    putString("\r\n(c) Angus Logan 2022-23\r\n");
    putString("For Bear and Moose\r\n\r\n");
    putString("(b)ack");

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
        putString("  9: Reset device");
        putString("\r\nEnter number, or (e)xit");

        if (valChanged)
        {
            putString(" do not save, or (s)ave and exit.\r\n");
        }
        else
        {
            putString("\r\n");
        }

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

    /* Warn if the changes are going to be discarded */
    if ((0 != valChanged) && ('e' == c))
    {
        putString("Discard changes? (y/n)\r\n");
        while (('y' != c) && ('n' != c))
        {
            c = waitForChar();

        }
        c = ('y' == c) ? 'e' : 's';
    }

    /* Save configuration if requested, then let watchdog reset the sytem */
    if ('s' == c)
    {
        #ifndef HOSTED
        eepromWrite(EEPROM_BASE_ADDR, pCfg, sizeof(Emon32Config_t));
        while (EEPROM_WR_COMPLETE != eepromWrite(0, 0, 0))
        {
            timerDelay_us(EEPROM_WR_TIME);
        }
        #endif
    }
    emon32StateSet(EMON_STATE_ACTIVE);
}

void
configEnter(Emon32Config_t *pConfig)
{
    pCfg = pConfig;
    menuBase();
}
