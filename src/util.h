#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum ITOA_BASE_ { ITOA_BASE10, ITOA_BASE16 } ITOA_BASE_t;
typedef struct ConvFloat_ {
  bool  valid;
  float val;
} ConvFloat_t;
typedef struct ConvInt_ {
  bool    valid;
  int32_t val;
} ConvInt_t;

/*! @brief Convert null terminated string to float, returns the value.
 *  @param [in] pBuf : pointer to string buffer
 */
ConvFloat_t utilAtof(char *pBuf);

/*! @brief Convert null terminated string to integer, returns the value.
 *  @param [in] pBuf : pointer to string buffer
 *  @param [in] base : select base 10 or base 16 conversion
 */
ConvInt_t utilAtoi(char *pBuf, ITOA_BASE_t base);

/*! @brief Indicate if a character is printable
 *  @param [in] c : character to check
 *  @return true if printable, false otherwise
 */
bool utilCharPrintable(const char c);

/*! @brief Convert float to null terminated base 10 string, with 2 dp.
 *         precision Returns the number of characters (including NULL).
 *  @param [in] pBuf : pointer to string buffer, at least 11 characters
 *  @param [in] val : value to convert
 */
unsigned int utilFtoa(char *pBuf, float val);

/*! @brief Convert integer to null terminated string. Returns the number of
 *         characters (including NULL).
 *  @param [in] pBuf : pointer to string buffer, at least 11 characters
 *  @param [in] val : value to convert
 *  @param [in] base : select base 10 or base 16 conversion
 */
unsigned int utilItoa(char *pBuf, int32_t val, ITOA_BASE_t base);

/*! @brief Returns the number of characters up to, but not including, NULL
 *  @param [in] pBuf : pointer to the NULL terminated string buffer
 */
unsigned int utilStrlen(const char *pBuf);

/*! @brief Reverse an array (typically string)
 *  @param [in] pBuf : pointer to the buffer
 *  @param [in] len : length of buffer to reverse
 */
void utilStrReverse(char *pBuf, unsigned int len);
