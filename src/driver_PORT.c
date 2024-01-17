#include "emon32_samd.h"
#include "driver_PORT.h"

void
portSetup()
{
    extern const uint8_t pinsGPIO_Out[][2];
    extern const uint8_t pinsGPIO_In[][2];
    extern const uint8_t pinsUnused[][2];

    /* GPIO outputs */
    for (unsigned int i = 0; pinsGPIO_Out[i][0] != 0xFF; i++)
    {
        portPinDir(pinsGPIO_Out[i][0], pinsGPIO_Out[i][1], PIN_DIR_OUT);
    }

    /* GPIO inputs  - all inputs currently need pull ups, so default enable */
    for (unsigned int i = 0; pinsGPIO_In[i][0] != 0xFF; i++)
    {
        portPinDir(pinsGPIO_In[i][0], pinsGPIO_In[i][1], PIN_DIR_IN);
        portPinCfg(pinsGPIO_In[i][0], pinsGPIO_In[i][1],
                   PORT_PINCFG_PULLEN, PIN_CFG_SET);
        portPinDrv(pinsGPIO_In[i][0], pinsGPIO_In[i][1], PIN_DRV_SET);
    }

    /* Unused pins: input, pull down (Table 22-1) */
    for (unsigned int i = 0; pinsUnused[i][0] != 0xFF; i++)
    {
        portPinDir(pinsUnused[i][0], pinsUnused[i][1], PIN_DIR_IN);
        portPinCfg(pinsUnused[i][0], pinsUnused[i][1],
                   PORT_PINCFG_PULLEN, PIN_CFG_SET);
    }
}

void
portPinDir(unsigned int grp, unsigned int pin, PINDIR_t mode)
{
    if (PIN_DIR_IN == mode)
    {
        PORT->Group[grp].DIRCLR.reg = (1u << pin);
    }
    else
    {
        PORT->Group[grp].DIRSET.reg = (1u << pin);
    }
    PORT->Group[grp].PINCFG[pin].reg |= PORT_PINCFG_INEN;
}

void
portPinMux(unsigned int grp, unsigned int pin, unsigned int mux)
{
    PORT->Group[grp].PINCFG[pin].reg |= PORT_PINCFG_PMUXEN;
    if (pin & 1u)
    {
        PORT->Group[grp].PMUX[pin >> 1].bit.PMUXO = mux;
    }
    else
    {
        PORT->Group[grp].PMUX[pin >> 1].bit.PMUXE = mux;
    }
}

void
portPinCfg(unsigned int grp, unsigned int pin, unsigned int cfg, PINCFG_t cs)
{
    if (PIN_CFG_SET == cs)
    {
        PORT->Group[grp].PINCFG[pin].reg |= cfg;
    }
    else
    {
        PORT->Group[grp].PINCFG[pin].reg &= ~cfg;
    }
}

void
portPinDrv(unsigned int grp, unsigned int pin, PINDRV_t drv)
{
    switch (drv)
    {
        case PIN_DRV_CLR:
            PORT->Group[grp].OUTCLR.reg = (1u << pin);
            break;
        case PIN_DRV_SET:
            PORT->Group[grp].OUTSET.reg = (1u << pin);
            break;
        case PIN_DRV_TGL:
            PORT->Group[grp].OUTTGL.reg = (1u << pin);
            break;
    }
}

unsigned int
portPinValue(unsigned int grp, unsigned int pin)
{
    unsigned int ret;
    ret = (0u == (PORT->Group[grp].IN.reg & (1u << pin))) ? 0u : 1u;
    return ret;
}
