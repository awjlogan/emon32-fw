#include "board_def.h"
#include "driver_PORT.h"
#include "driver_TIME.h"
#include "periph_DS18B20.h"

#if (BOARD_ID == BOARD_ID_LC)
    #include "samd11.h"
#else
    #include "samd21.h"
#endif /* BOARD_ID */

/* Driver for DS18B20 OneWire temperature sensor
 * https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf
 */

/* OneWire pins and configuration */
static uint8_t  grp;
static uint8_t  pin;
static uint8_t  t_wait_us;

/* Device address table */
static uint64_t address[TEMP_MAX_ONEWIRE];
static uint8_t  address_remap[TEMP_MAX_ONEWIRE];


/* OneWire functions */
static uint8_t  oneWireReset();
static void     oneWireWriteBytes(const void *pSrc, const uint8_t n);
static void     oneWireReadBytes(void *pDst, const uint8_t n);
static void     oneWireSearch();


static uint8_t
oneWireReset()
{
    uint8_t     presence = 0;

    portPinDrv(grp, pin, PIN_DRV_CLR);
    portPinDir(grp, pin, PIN_DIR_OUT);

    timerDelay_us(500u);
    portPinDir(grp, pin, PIN_DIR_IN);
    timerDelay_us(120u);

    /* If there is any device present, the bus will be LOW */
    presence = ~portPinValue(grp, pin);
    timerDelay_us(380u);

    return presence;
}


static void
oneWireWriteBytes(const void *pSrc, const uint8_t n)
{
    uint8_t *pData = (uint8_t *)pSrc;

    /* See timing diagrams in Figure 16. Interrupts are disabled in sections
     * where too long would break the OneWire protocol. At the end of a bit
     * transmission, a pending intterupt may be serviced, but this will only
     * extend the interbit timing, with no affect on the protocol.
     */
    for (uint8_t i = 0; i < n; i++)
    {
        uint8_t byte = *pData++;
        for (uint8_t j = 0; j < 8; j++)
        {
            __disable_irq();
            portPinDir(grp, pin, PIN_DIR_OUT);
            timerDelay_us(t_wait_us);
            if (!(byte & 0x1))
            {
                portPinDir(grp, pin, PIN_DIR_IN);
            }
            timerDelay_us(75u - t_wait_us);
            portPinDir(grp, pin, PIN_DIR_IN);
            __enable_irq();
            timerDelay_us(5u);
            byte >>= 1;
        }
    }
}

static void
oneWireReadBytes(void *pDst, const uint8_t n)
{
    uint8_t *pData = (uint8_t *)pDst;

    for (uint8_t i = 0; i < n; i++)
    {
        for (uint8_t j = 0; j < 8; j++)
        {
            __disable_irq();
            portPinDir(grp, pin, PIN_DIR_OUT);
            timerDelay_us(t_wait_us);
            portPinDir(grp, pin, PIN_DIR_IN);
            /* Max 15 us for read slot; leave 3 us slack */
            timerDelay_us(12u - t_wait_us);
            /* Data received LSB first */
            *pData |= (portPinValue(grp, pin) << j);
            __enable_irq();
        }
    }
}


static void
oneWireSearch()
{
    const uint8_t cmd_search_rom = 0xF0;
    for (uint8_t i = 0; i < TEMP_MAX_ONEWIRE; i++)
    {
        (void)oneWireReset();
        oneWireWriteBytes(&cmd_search_rom, 1);
    }
}


uint8_t
ds18b20InitSensors(const DS18B20_conf_t *pCfg)
{
    uint8_t     deviceCount = 0;

    grp         = pCfg->grp;
    pin         = pCfg->pin;
    /* If not overridden, default to 5 us pull low */
    t_wait_us   = pCfg->t_wait_us ? pCfg->t_wait_us : 5u;

    /* Reset and check for presence of any devices (Figure 15). */
    if (0 == oneWireReset())
    {
        return 0;
    }

    /* Search for devices and populate remapping table */
    oneWireSearch();

    return deviceCount;
}


int8_t
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
ds18b20ReadSample(const uint8_t dev)
{
    const uint8_t   cmd_match_rom = 0x55;
    /* REVISIT : apply remapping if necessary */
    const uint64_t  addr_dev = address[dev];
    const uint8_t   cmd_read_scratch = 0xBE;
    uint8_t         tempData[2] = {0};
    int16_t         retData = 0;

    /* Check for presence pulse before continuing */
    if (0 == oneWireReset())
    {
        return INT16_MIN;
    }

    oneWireWriteBytes(&cmd_match_rom, 1);
    oneWireWriteBytes(&addr_dev, 8);
    oneWireWriteBytes(&cmd_read_scratch, 1);
    /* REVISIT : can read all 9 bytes to get the CRC as well */
    oneWireReadBytes(&tempData, 2);

    /* Second byte is the MSB, shift to top 8 */
    retData = tempData[0] | (int16_t)(tempData[1] << 8);
    return retData;
}
