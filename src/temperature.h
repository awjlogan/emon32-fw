#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <stdint.h>

typedef enum {
    TEMP_ONEWIRE,
    TEMP_I2C
} TEMP_INTF_t;

/*! @brief Find and initialise sensors
 *  @param [in] intf : interface type
 *  @param [in] pParams : parameters for given interface type
 *  @return : number of sensors found
 */
uint8_t tempInitSensors(const TEMP_INTF_t intf, const void *pParams);

/*! @brief Start a temperature sample
    @param [in] intf : interface type
 *  @param [in] dev : device index
 */
void tempStartSample(const TEMP_INTF_t intf, const uint32_t dev);

/*! @brief Read an existing temperature sample
 *  @param [in] intf : interface type
 *  @param [in] dev : device index
 *  @return : temperature value
 */
int16_t tempReadSample(const TEMP_INTF_t intf, const uint32_t dev);

#endif