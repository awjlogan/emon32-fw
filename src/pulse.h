#ifndef PULSE_H
#define PULSE_H

typedef enum {
    PULSE_EDGE_RISING   = 0,
    PULSE_EDGE_FALLING  = 1,
    PULSE_EDGE_BOTH     = 2
} PulseEdge_t;

typedef struct {
    PulseEdge_t     edge;
    unsigned int    grp;
    unsigned int    pin;
    unsigned int    periods;
    unsigned int    active;
} PulseCfg_t;

/*! @brief Returns a pointer to the pulse counter configuration
 *  @param [in] index : index of the pulse counter to access.
 *  @return : pointer to configuration struct. 0 for failure
 */
PulseCfg_t *pulseGetCfg(const unsigned int index);

/*! Initialise a configured pulse counter
 *  @param [in] pCfg : pointer to configuration struct
 *  @param [in] index : pulse counter index
 */
void pulseInit(const unsigned int index);

/*! @brief Update the pulse counter(s)
 */
void pulseUpdate();

/*! @brief Sets the pulse count value
 *  @param [in] pulseCount : the value to set
 *  @param [in] index : pulse count index to set
 */
void pulseSetCount(const uint64_t value, const unsigned int index);

/*! @brief Get the current pulse count value
 *  @return : current pulse value
 */
uint64_t pulseGetCount(const unsigned int index);

#endif