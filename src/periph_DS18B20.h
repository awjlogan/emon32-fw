#pragma once

#include <stdint.h>

#include <temperature.h>

typedef struct DS18B20_conf_ {
  uint8_t grp;
  uint8_t pin;
  uint8_t t_wait_us;
} DS18B20_conf_t;

typedef struct DS18B20_Res_ {
  TempStatus_t status;
  int16_t      temp;
} DS18B20_Res_t;

/*! @brief Configure the OneWire port and find sensors
 *  @param [in] pCfg: pointer to the configuration struct
 *  @return : number of sensors found
 */
unsigned int ds18b20InitSensors(const DS18B20_conf_t *pCfg);

/*! @brief Start a temperature conversion on all OneWire devices
 *  @return : 0 for success, -1 if no presence pulse detected
 */
int ds18b20StartSample(void);

/*! @brief Read the temperature data from a OneWire device
 *  @param [in] dev : index of OneWire device
 *  @return : result in struct
 */
DS18B20_Res_t ds18b20ReadSample(const unsigned int dev);

/*! @brief Convert the DS18B20 value into a float
 *  @param [in] fix : 8.4 fixed point value
 *  @return : temperature in C
 */
float ds18b20SampleToCelsius(const int16_t fix);
