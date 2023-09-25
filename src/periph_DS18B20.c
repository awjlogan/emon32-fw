#include "emon32_samd.h"

static unsigned int grp;
static unsigned int pin;


int
writeBits(void *pData, const unsigned int n)
{
    static unsigned int cnt     = 0;
    static unsigned int j       = 0;
    uint8_t             *bytes  = 0;

    if (0 == cnt && n != 0)
    {
        cnt     = n;
        bytes   = (uint8_t *)pData;
    }

    /* REVISIT Check limit here... */
    for (;cnt;cnt--)
    {

    }

}


void
tempSetup(const DS18B20_conf_t *pCfg)
{
    grp = pCfg->grp;
    pin = pCfg->pin;


}

void
tempConfigure(const DS18B20_conf_t *pCfg)
{
    uint64_t *addrs = (uint64_t *)pCfg->tempAddr;

    /* DS18B20 defaults to 12 bit resolution */
    if (DS18B20_CONV_12 == pCfg->conv)
    {
        return;
    }

    while (*addrs)
    {
        /* Reset pulse in >= 480 us */
        portPinDrv(pCfg->grp, pCfg->pin, PIN_DRV_CLR);
        timerDelay_us(500);

    }
}


