#pragma once

#include <stdint.h>

#define TEMP_CONVERSION_T 750 /* Minimum temperature sample time */

typedef enum TEMP_INTF_ { TEMP_INTF_ONEWIRE, TEMP_INTF_I2C } TEMP_INTF_t;

/*! @brief Return the temperature as a float
 *  @param [in] tFixed : fixed point temperature
 *  @return : the temperature as a float
 */
float tempAsFloat(const TEMP_INTF_t intf, const int16_t tFixed);

/*! @brief Find and initialise sensors
 *  @param [in] intf : interface type
 *  @param [in] pParams : parameters for given interface type
 *  @return : number of sensors found
 */
unsigned int tempInitSensors(const TEMP_INTF_t intf, const void *pParams);

/*! @brief Read an existing temperature sample
 *  @param [in] intf : interface type
 *  @param [in] dev : device index
 *  @return : INT16_MIN for failure, otherwise temperature value
 */
int16_t tempReadSample(const TEMP_INTF_t intf, const uint8_t dev);

/*! @brief Start a temperature sample
    @param [in] intf : interface type
 *  @param [in] dev : device index
 */
int tempStartSample(const TEMP_INTF_t intf, const uint32_t dev);
