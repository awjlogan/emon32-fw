#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

typedef enum {
    ITOA_BASE10,
    ITOA_BASE16
} ITOA_BASE_t;

/*! @brief Convert integer to null terminated string. Returns the number of
 *         characters (including NULL).
 *  @param [in] pBuf : pointer to string buffer, at least 11 characters
 *  @param [in] val : value to convert
 *  @param [in] base : select base 10 or base 16 conversion
 */
unsigned int utilItoa(char *pBuf, int32_t val, ITOA_BASE_t base);

/*! @brief Convert null terminated string to integer, returns the value.
 *  @param [in] pBuf : pointer to string buffer
 *  @param [in] base : select base 10 or base 16 conversion
 */
int32_t utilAtoi(char *pBuf, ITOA_BASE_t base);

/*! @brief Convert float to null terminated base 10 string, with 2 dp.
 *         precision Returns the number of characters (including NULL).
 *  @param [in] pBuf : pointer to string buffer, at least 11 characters
 *  @param [in] val : value to convert
 */
unsigned int utilFtoa(char *pBuf, float val);

/*! @brief Convert null terminated string to float, returns the value.
 *  @param [in] pBuf : pointer to string buffer
 */
float utilAtof(char *pBuf);

/*! @brief Reverse an array (typically string)
 *  @param [in] pBuf : pointer to the buffer
 *  @param [in] len : length of buffer to reverse
 */
void utilStrReverse(char *pBuf, unsigned int len);

/*! @brief Returns the number of characters up to, but not including, NULL
 *  @param [in] pBuf : pointer to the string buffer
 */
unsigned int utilStrlen(const char *pBuf);

/*! @brief Insert a string into an existing buffer. Returns the end position
 *         of the "cursor" in the destination buffer.
 *  @param [in] pDst : pointer to the buffer to insert into
 *  @param [in] pIns : pointer to the string to insert
 *  @param [in] pos  : start position to insert
 *  @param [in] len  : length of string to insert
 */
unsigned int utilStrInsert(char *pDst, const char *pIns, unsigned int pos, unsigned int len);

#endif
