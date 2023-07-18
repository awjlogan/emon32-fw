#ifndef RFM69_H
#define RFM69_H

typedef enum {
    RF12_868MHz,
    RF12_915MHz,
    RF12_433MHz
} RFM_Freq_t;

typedef struct _RFMPkt {
    void            *data;
    unsigned int    n;
    unsigned int    node;
    unsigned int    grp;
    unsigned int    rf_pwr;
    unsigned int    threshold;
    unsigned int    timeout;
} RFMPkt_t;

RFMPkt_t *rfmGetHandle();

/*! @brief Initialise the RFM69 module
 *  @param [in] freq : RFM operating frequency
 */
void rfmInit(RFM_Freq_t freq);

/*! @brief Send a packet through the RFM69 module
 *  @param [in] : Pointer to the RFM packet
 *  @return : 0 for success, -1 for failure
 */
int rfmSend(const void *pData);

#endif