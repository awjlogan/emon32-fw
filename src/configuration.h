#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "emon32.h"

/*! @brief This function enters the configuration menu.
 *  @param [in] pCfg : pointer to the emon32 configuration struct
 */

void configEnter(Emon32Config_t *pCfg);

/*! @brief Load the default configuration
 *  @param [out] pCfg : pointer to emon32 configuration struct
 */
void configDefault(Emon32Config_t *pCfg);

/*! @brief This functions loads the default configuration and, if available,
 *         the saved configuration from NVM
 *  @param [out] pCfg : pointer to emon32 configuration struct
 */
void configLoadFromNVM(Emon32Config_t *pCfg);

#endif
