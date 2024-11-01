#pragma once

#define RFM_PALEVEL_DEF                                                        \
  0x19 /* Default power level must be below maximum                            \
        * in case there is no antenna - this can                               \
        * destroy the RFM module.                                              \
        */

typedef enum RFM_Freq_ {
  RFM_FREQ_868MHz,
  RFM_FREQ_915MHz,
  RFM_FREQ_433MHz
} RFM_Freq_t;

typedef enum RFMSend_ {
  RFM_NO_INIT,
  RFM_TIMED_OUT,
  RFM_FAILED,
  RFM_SUCCESS,
  RFM_N_TOO_LARGE
} RFMSend_t;

typedef struct RFMOpt_ {
  void        *data;
  unsigned int n;
  unsigned int node;
  unsigned int grp;
  unsigned int rf_pwr;
  unsigned int threshold;
  unsigned int timeout;
} RFMOpt_t;

/*! @brief Get a pointer to the RFM69's data buffer
 *  @return pointer to RFM69 buffer
 */
uint8_t *rfmGetBuffer(void);

RFMOpt_t *rfmGetHandle(void);

/*! @brief Initialise the RFM69 module
 *  @param [in] freq : RFM operating frequency
 *  @return : true if successful, false otherwise
 */
bool rfmInit(RFM_Freq_t freq);

/*! @brief The interrupt handler for RFM69 receive */
void rfmInterrupt(void);

/*! @brief Send data through the RFM69
 *  @param [in] : number of bytes to be sent
 *  @return : result of the attempt to send
 */
RFMSend_t rfmSendBuffer(const int_fast8_t n);

/*! @brief Sets the RFM69's address
 *  @param [in] addr : address to set the RFM69
 */
void rfmSetAddress(const uint16_t addr);

/*! @brief Set the AES key for encryption
 *  @param [in] aes : 16 character AES key. 0 disables encryption.
 */
void rfmSetAESKey(const char *aes);
