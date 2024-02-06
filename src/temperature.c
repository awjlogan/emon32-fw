#include "periph_DS18B20.h"
#include "temperature.h"


float
tempAsFloat(const TEMP_INTF_t intf, const int16_t tFixed)
{
    float ret = -1000.0;
    if (TEMP_INTF_ONEWIRE == intf)
    {
        ret = ds18b20SampleToCelsius(tFixed);
    }
    return ret;
}

unsigned int
tempInitSensors(const TEMP_INTF_t intf, const void *pParams)
{

    if (TEMP_INTF_ONEWIRE == intf)
    {
        return ds18b20InitSensors((DS18B20_conf_t *)pParams);
    }

    return 0;
}


int16_t
tempReadSample(const TEMP_INTF_t intf, const uint8_t dev)
{
    if (TEMP_INTF_ONEWIRE == intf)
    {
        return ds18b20ReadSample(dev);
    }
    /* Default to failure */
    return INT16_MIN;
}


int
tempStartSample(const TEMP_INTF_t intf, const uint32_t dev)
{
    if (TEMP_INTF_ONEWIRE == intf)
    {
        (void)dev;
        return ds18b20StartSample();
    }
    /* Default to failure */
    return -1;
}
