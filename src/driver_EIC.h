#pragma once

/*! @brief Setup the External Interrupt Controller */
void eicSetup(void);

/*! @brief Clear the zero crossing interrupt flag. */
void eicZeroXClr(void);

/*! @brief Get the status of the zero crossing interrupt flag
 *  @return : 1 if flag is set, 0 otherwise
 */
int eicZeroXStat(void);
