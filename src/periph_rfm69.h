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
  RFM_SUCCESS
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

RFMOpt_t *rfmGetHandle(void);

/*! @brief Initialise the RFM69 module
 *  @param [in] freq : RFM operating frequency
 */
void rfmInit(RFM_Freq_t freq);

/*! @brief Send a packet through the RFM69 module
 *  @param [in] : Pointer to the RFM packet
 *  @return : 0 for success, -1 for failure
 */
RFMSend_t rfmSend(const void *pData);

RFMSend_t rfmSendReady(uint32_t timeout);
