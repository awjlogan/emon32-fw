#include "emon32_samd.h"

#include "driver_PORT.h"
#include "driver_TIME.h"
#include "periph_DS18B20.h"

/* Driver for DS18B20 OneWire temperature sensor
 * https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf
 *
 * OneWire search algorithm adapted from:
 * https://www.analog.com/en/app-notes/1wire-search-algorithm.html
 */

/* OneWire pins and configuration */
static DS18B20_conf_t cfg;
static unsigned int grp;
static unsigned int pin;
static unsigned int t_wait_us;

/* Device address table */
static uint64_t     address[TEMP_MAX_ONEWIRE];
static unsigned int addressRemap[TEMP_MAX_ONEWIRE];

/* OneWire functions & state variables */
static uint8_t      calcCRC8            (const uint8_t crc, const uint8_t value);
static int          oneWireFirst        ();
static int          oneWireNext         ();
static unsigned int oneWireReadBit      ();
static void         oneWireReadBytes    (void *pDst, const uint8_t n);
static unsigned int oneWireReset        ();
static int          oneWireSearch       ();
static void         oneWireWriteBit     (unsigned int bit);
static void         oneWireWriteBytes   (const void *pSrc, const uint8_t n);

uint64_t    ROM_NO;
uint8_t     crc8;
int         lastDiscrepancy;
int         lastFamilyDiscrepancy;
int         lastDeviceFlag;

static uint8_t
calcCRC8(const uint8_t crc, const uint8_t value)
{
   const uint8_t dscrc_table[] = {
      0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
    157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
     35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
    190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
     70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
    219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
    101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
    248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
    140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
     17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
    175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
     50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
    202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
     87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
    233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
    116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

   return dscrc_table[crc ^ value];
}

/*! @brief: Find the first device on the 1-Wire bus
 *  @return : 1: device found, ROM number in ROM_NO buffer
 *            0: no devices present
 */
static int
oneWireFirst()
{
    /* Reset the search state */
    lastDiscrepancy         = 0;
    lastDeviceFlag          = 0;
    lastFamilyDiscrepancy   = 0;

    return oneWireSearch();
}

/*! @brief: Find the next device on the 1-Wire bus
 *  @return : 1: device found, ROM number in ROM_NO buffer
 *            0: device not found, end of search
 */
static int
oneWireNext()
{
    return oneWireSearch();
}

static unsigned int
oneWireReadBit()
{
    unsigned int result = 0;

    __disable_irq           ();
    portPinDir              (grp, pin, PIN_DIR_OUT);
    timerDelay_us           (t_wait_us);
    portPinDir              (grp, pin, PIN_DIR_IN);
    /* Max 15 us for read slot; leave 3 us slack */
    timerDelay_us           (12u - t_wait_us);
    result = portPinValue   (grp, pin);
    __enable_irq            ();

    return result;
}

static void
oneWireReadBytes(void *pDst, const uint8_t n)
{
    uint8_t *pData = (uint8_t *)pDst;

    for (uint8_t i = 0; i < n; i++)
    {
        for (uint8_t j = 0; j < 8; j++)
        {
            /* Data received LSB first */
            *pData |= (oneWireReadBit() << j);
        }
        pData++;
    }
}

static unsigned int
oneWireReset()
{
    unsigned int presence = 0;

    portPinDrv(grp, pin, PIN_DRV_CLR);
    portPinDir(grp, pin, PIN_DIR_OUT);

    timerDelay_us(500u);
    portPinDir(grp, pin, PIN_DIR_IN);
    timerDelay_us(120u);

    /* If there is any device present, the bus will be LOW */
    presence = !portPinValue(grp, pin);
    timerDelay_us(380u);

    return presence;
}

static int
oneWireSearch()
{
    /* Initialise for search */
    const uint8_t   cmdSearchRom    = 0xF0;
    int             searchDirection = 0;
    int             idBitNumber     = 1;
    int             lastZero        = 0;
    uint8_t         romByteMask     = 1;
    int             searchResult    = 0;
    int             idBit           = 0;
    int             cmpidBit        = 0;
    uint8_t         *romBuffer      = (uint8_t *)&ROM_NO;

    /* If the last call was not the last one... */
    if (!lastDeviceFlag)
    {
        /* ... reset the OneWire bus... */
        if (0 == oneWireReset())
        {
            /* Reset the search */
            lastDiscrepancy         = 0;
            lastDeviceFlag          = 0;
            lastFamilyDiscrepancy   = 0;
            return 0;
        }

        /* ...issue the search command...*/
        oneWireWriteBytes(&cmdSearchRom, 1);

        /* ...and commence the search! */
        for (unsigned int i = 0; i < 8; i++)
        {
            idBit       = oneWireReadBit();
            cmpidBit    = oneWireReadBit();

            /* Check for no devices on OneWire */
            if (idBit && cmpidBit)
            {
                break;
            }

            if (idBit != cmpidBit)
            {
                searchDirection = idBit;
            }
            else
            {
                /* If this discrepancy is before the last discrepancy on a previous
                 * next then pick the same as last time
                 */
                searchDirection =   (idBitNumber < lastDiscrepancy)
                                  ? ((*romBuffer & romByteMask) > 0)
                                  : (idBitNumber == lastDiscrepancy);

                /* If 0 was picked, record its position */
                if (0 == searchDirection)
                {
                    lastZero = idBitNumber;
                    /* and check for last discrepancy in Family */
                    if (lastZero < 9)
                    {
                        lastFamilyDiscrepancy = lastZero;
                    }
                }
            }

            /* Set or clear the bit in the ROM byte number with mask */
            if (0 == searchDirection)
            {
                *romBuffer &= ~romByteMask;
            }
            else
            {
                *romBuffer |= romByteMask;
            }

            /* Serial number search direction bit */
            oneWireWriteBit(searchDirection);
            idBitNumber++;
            romByteMask <<= 1;

            /* When the mask is 0, go to new serial number byte and reset */
            if (0 == romByteMask)
            {
                crc8 = calcCRC8(crc8, *romBuffer);
                romBuffer++;
                romByteMask = 1;
            }
        }
    }

    /* If the search was successful... */
    if (!((65 > idBitNumber) || (0 != crc8)))
    {
        lastDiscrepancy = lastZero;
        searchResult    = 1;

        /* Check for last device */
        if (0 == lastDiscrepancy)
        {
            lastDeviceFlag = 1;
        }
    }

    return searchResult;
}

static void
oneWireWriteBit(unsigned int bit)
{
    /* See timing diagrams in Figure 16. Interrupts are disabled in sections
     * where too long would break the OneWire protocol. At the end of a bit
     * transmission, a pending interrupt may be serviced, but this will only
     * extend the interbit timing, with no affect on the protocol.
     */
    __disable_irq   ();
    portPinDir      (grp, pin, PIN_DIR_OUT);
    timerDelay_us   (t_wait_us);
    if (0 == bit)
    {
        portPinDir  (grp, pin, PIN_DIR_IN);
    }
    timerDelay_us   (75u - t_wait_us);
    portPinDir      (grp, pin, PIN_DIR_IN);
    __enable_irq    ();
    timerDelay_us   (5u);
}

static void
oneWireWriteBytes(const void *pSrc, const uint8_t n)
{
    uint8_t *pData = (uint8_t *)pSrc;
    for (uint8_t i = 0; i < n; i++)
    {
        uint8_t byte = *pData++;
        for (uint8_t j = 0; j < 8; j++)
        {
            oneWireWriteBit((byte & 0x1));
            byte >>= 1;
        }
    }
}

unsigned int
ds18b20InitSensors(const DS18B20_conf_t *pCfg)
{
    unsigned int deviceCount    = 0;
    int          searchResult   = 0;

    cfg.grp         = pCfg->grp;
    cfg.pin         = pCfg->pin;
    /* If not overridden, default to 5 us pull low */
    cfg.t_wait_us   =   pCfg->t_wait_us
                      ? pCfg->t_wait_us
                      : 5u;

    /* Search for devices */
    searchResult = oneWireFirst();

    while ((0 != searchResult) && (deviceCount < TEMP_MAX_ONEWIRE))
    {
        address[deviceCount] = ROM_NO;
        deviceCount++;

        searchResult = oneWireNext();
    }

    /* REVISIT : populate remapping table from saved sensors */
    for (unsigned int i = 0; i < TEMP_MAX_ONEWIRE; i++)
    {
        addressRemap[i] = i;
    }

    return deviceCount;
}


int
ds18b20StartSample()
{
    const uint8_t cmds[2] = {0xCC, 0x44};

    /* Check for presence pulse before continuing */
    if (0 == oneWireReset())
    {
        return -1;
    }

    oneWireWriteBytes(cmds, 2u);
    return 0;
}

int16_t
ds18b20ReadSample(const unsigned int dev)
{
    const uint8_t   cmdMatchROM     = 0x55;
    const uint8_t   cmdReadScratch  = 0xBE;
    const uint64_t  *addrDev        = address + addressRemap[dev];
    int             tempData        = 0;

    /* Check for presence pulse before continuing */
    if (0 == oneWireReset())
    {
        return INT16_MIN;
    }

    oneWireWriteBytes   (&cmdMatchROM, 1);
    oneWireWriteBytes   (addrDev, 8);
    oneWireWriteBytes   (&cmdReadScratch, 1);
    /* REVISIT : can read all 9 bytes to get the CRC as well */
    oneWireReadBytes    (&tempData, 2);

    /* Second byte is the MSB, shift to top 8 */
    return (int16_t)tempData;
}
