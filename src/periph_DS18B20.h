#ifndef PERIPH_DS18B20_H
#define PERIPH_DS18B20_H

#include <stdint.h>

typedef struct {
    unsigned int    grp;
    unsigned int    pin;
    unsigned int    t_wait_us;
} DS18B20_conf_t;

/*! @brief Configure the OneWire port
 *  @param [in] pCfg: pointer to the configuration struct
 */
unsigned int ds18b20InitSensors(const DS18B20_conf_t *pCfg);

/*! @brief Start a temperature conversion on all OneWire devices
 *  @return : 0 for success, -1 if no presence pulse detected
 */
int ds18b20StartSample();

/*! @brief Read the temperature data from a OneWire device
 *  @param [in] dev : index of OneWire device
 *  @return : INT16_MIN for failure (no presence response), otherwise sensor data
 */
int16_t ds18b20ReadSample(const unsigned int dev);

#endif