#ifndef DATA_H
#define DATA_H

#include "emon_CM.h"
#include "util.h"

/*! @brief Packs the emon_CM packet into EmonESP format.
 *         ct1:X,ct1:x,t1:x,t2:x
 *         Returns the number of characters that would have been packed,
 *         regardless of the value of n. If the return value != n, then the
 *         buffer would have overflowed (similar to snprintf). Does not append
 *         a NULL. Clears data buffer in advance.
 *  @param [in] pData : pointer to the raw data
 *  @param [out] pDst : pointer to the destination buffer
 *  @param [in] n : width of the destination buffer
 */
unsigned int dataPackageESP_n(const ECMSet_t *pData, char *pDst, unsigned int n);

/*! @brief Packs the emon_CM into EmonESP format with no buffer size. Use when
 *         the maximum output width is known. Returns the number of characters
 *  @param [in] pData : pointer to the raw data
 *  @param [out] pDst : pointer to the destination buffer
 */
unsigned int dataPackage(const ECMSet_t *pData, char *pDst);

#endif
