#pragma once

#include "emon32.h"

/*! @brief Add a character to the command stream
 *  @param [in] c : character to add
 */
void configCmdChar(const uint8_t c);

/*! @brief Print the board and firmware information to serial */
void configFirmwareBoardInfo(void);

/*! @brief This functions loads the default configuration and, if available,
 *         the saved configuration from NVM
 *  @param [out] pCfg : pointer to emon32 configuration struct
 */
void configLoadFromNVM(Emon32Config_t *pConfig);

/*! @brief Process a pending command from the UART */
void configProcessCmd(void);

/*! @brief Convert a time into cycles
 *  @param [in] time: time in seconds
 *  @param [in] mainsFreq: mains frequency in Hertz {50, 60}
 *  @return : number of cycles between reports
 */
unsigned int configTimeToCycles(const float time, const unsigned int mainsFreq);

/*! @brief Return one word from the SAMD's unique ID
 *  @param[in] idx : index of the word to fetch
 *  @return word idx from the unique ID
 */
uint32_t getUniqueID(unsigned int idx);
