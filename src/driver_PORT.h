#pragma once

#include <stdint.h>

/* Types */
typedef enum {
    PIN_DIR_IN,
    PIN_DIR_OUT
} PINDIR_t;

typedef enum {
    PIN_CFG_SET,
    PIN_CFG_CLR
} PINCFG_t;

typedef enum {
    PIN_DRV_CLR,
    PIN_DRV_SET,
    PIN_DRV_TGL
} PINDRV_t;

/*! @brief   Configure the ports.
 *           Ports for peripherals are configured in their setup functions
 */
void portSetup(void);

/*! @brief Sets a pin as input or output
 *  @param [in] grp : Group number; for SAMD1x this is always 0
 *  @param [in] pin : PIN number
 *  @param [in] mode: PIN_IN for input, PIN_OUT for output
 */
void portPinDir(unsigned int grp, unsigned int pin, PINDIR_t mode);

/*! @brief Sets the mux for pin alternate function
 *  @param [in] grp : Group number; for SAMD1x this is always 0
 *  @param [in] pin : Pin number
 *  @param [in] mux : Mux mode
 */
void portPinMux(unsigned int grp, unsigned int pin, unsigned int mux);

/*! @brief Sets the pin configuration
 *  @param [in] grp : Group number; for SAMD1x this is always 0
 *  @param [in] pin : Pin number
 *  @param [in] cfg : configuration option
 *  @param [in] cs  : clear or set configuration
 */
void portPinCfg(unsigned int grp, unsigned int pin, unsigned int cfg, PINCFG_t cs);

/*! @brief Sets the pin driver value
 *  @param [in] grp : Group number; for SAMD1x this is always 0
 *  @param [in] pin : Pin number
 *  @param [in] drv : Clear, set, or toggle pin
 */
void portPinDrv(unsigned int grp, unsigned int pin, PINDRV_t drv);

/*! @brief Returns the pin value
 *  @param [in] grp : Group number; for SAMD1x this is always 0
 *  @param [in] pin : Pin number
 */
unsigned int portPinValue(unsigned int grp, unsigned int pin);
