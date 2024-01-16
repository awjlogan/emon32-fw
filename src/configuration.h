#pragma once

#include "emon32.h"

/*! @brief Add a character to the command stream
 *  @param [in] c : character to add
 */
void configCmdChar(const uint8_t c);

/*! @brief Print the board and firmware information to serial */
void configFirmwareBoardInfo();

/*! @brief This functions loads the default configuration and, if available,
 *         the saved configuration from NVM
 *  @param [out] pCfg : pointer to emon32 configuration struct
 */
void configLoadFromNVM(Emon32Config_t *pConfig);

/*! @brief Process a pending command from the UART */
void configProcessCmd();

/*! @brief Convert the reporting time to cycles
 *  @param [in] repTime: reporting time in seconds
 *  @param [in] mainsFreq: mains frequency in Hertz {50, 60}
 *  @return : number of cycles between reports
 */
unsigned int configTimeToCycles(const float repTime, const unsigned int mainsFreq);

