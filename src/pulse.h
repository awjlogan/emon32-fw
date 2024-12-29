#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum PulseEdge_ {
  PULSE_EDGE_RISING  = 'r',
  PULSE_EDGE_FALLING = 'f',
  PULSE_EDGE_BOTH    = 'b'
} PulseEdge_t;

typedef struct PulseCfg_ {
  PulseEdge_t  edge;    /* Edge or edges to detect */
  unsigned int grp;     /* GPIO group */
  unsigned int pin;     /* GPIO pin */
  unsigned int periods; /* Blank period */
  bool         active;  /* Channel active  */
  bool         puEn;    /* Pull up enabled */
} PulseCfg_t;

/*! @brief Returns a pointer to the pulse counter configuration
 *  @param [in] index : index of the pulse counter to access.
 *  @return pointer to configuration struct. 0 for failure
 */
PulseCfg_t *pulseGetCfg(const unsigned int index);

/*! Initialise a configured pulse counter
 *  @param [in] index : pulse counter index
 */
void pulseInit(const unsigned int index);

/*! @brief Update the pulse counter(s) */
void pulseUpdate(void);

/*! @brief Sets the pulse count value
 *  @param [in] index : pulse count index to set
 *  @param [in] pulseCount : the value to set
 */
void pulseSetCount(const unsigned int index, const uint64_t value);

/*! @brief Get the current pulse count value
 *  @return current pulse value
 */
uint64_t pulseGetCount(const unsigned int index);
