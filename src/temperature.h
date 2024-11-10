#pragma once

#include <stdint.h>

#define TEMP_CONVERSION_T 750 /* Minimum temperature sample time */

typedef enum TEMP_INTF_ { TEMP_INTF_ONEWIRE, TEMP_INTF_I2C } TEMP_INTF_t;

typedef enum TempStatus_ {
  TEMP_OK,
  TEMP_OVERRUN,
  TEMP_NO_SENSORS,
  TEMP_FAILED,
  TEMP_NO_SAMPLE,
  TEMP_OUT_OF_RANGE,
  TEMP_UNUSED,
  TEMP_BAD_CRC,
  TEMP_BAD_SENSOR
} TempStatus_t;

typedef struct TempRead_ {
  TempStatus_t status;
  int16_t      result;
} TempRead_t;

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
 *  @return : TempRead struct
 */
TempRead_t tempReadSample(const TEMP_INTF_t intf, const uint8_t dev);

/*! @brief Start a temperature sample
    @param [in] intf : interface type
 *  @param [in] dev : device index
 */
TempStatus_t tempStartSample(const TEMP_INTF_t intf, const uint32_t dev);
