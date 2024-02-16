#pragma once

#include "emon32.h"

/*! @brief Packs the emon_CM packet into EmonESP format.
 *         Returns the number of characters that would have been packed,
 *         regardless of the value of n. If the return value != n, then the
 *         buffer would have overflowed (similar to snprintf). Does not append
 *         a NULL. Clears data buffer in advance.
 *  @param [in] pData : pointer to the raw data
 *  @param [out] pDst : pointer to the destination buffer
 *  @param [in] m : width of the destination buffer
 *  @return : the number of the characters that would be packed
 */
unsigned int dataPackageESP_n(const Emon32Dataset_t *pData, char *pDst, const unsigned int m);

/*! @brief Pack the voltage, power, energy, temperature, and pulse data into a
 *         packed structure for transmission over RFM link.
 *  @param [in] pData : pointer to the raw data
 *  @param [out] pPacked : pointer to the destination packet
 */
void dataPackagePacked(const Emon32Dataset_t *pData, PackedData_t *pPacked);
