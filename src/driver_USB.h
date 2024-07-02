#pragma once

#include <stdbool.h>


/*! @brief Get the connection status of the USB CDC
 *  @return : true if USB is connected, false otherwise.
 */
bool usbCDCIsConnected(void);

/*! @brief Send a null-terminated string to the USB CDC channel.
 *  @param [in] s: pointer to the null-terminated string.
 *  @return : true if successful, false otherwise.
 */
bool usbCDCPutsBlocking(const char *s);

/*! @brief Check if anything is in the Rx buffer
 *  @return : true if >0, false otherwise.
 */
bool usbCDCRxAvailable(void);

/*! @brief Get a character from the Rx buffer
 *  @return : a single character. 0 for failure.
 */
uint8_t usbCDCRxGetChar(void);

/*! @brief The USB device task that must be called regularly */
void usbCDCTask(void);

/*! @brief Set up USB hardware. This does not setup tinyUSB.*/
void usbSetup(void);
